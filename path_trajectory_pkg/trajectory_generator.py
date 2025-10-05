#!/usr/bin/env python3

import os
import time
import yaml
import numpy as np
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
import argparse 

import rclpy
from rclpy.node import Node


class TrajectoryGenerator(Node):

    def __init__(self, v_max, a_max, a_lat_max, smooth_acceleration):
        super().__init__('trajectory_generator')
        
        # Configuration parameters
        self.input_file = 'config/smooth_path.yaml'
        self.output_file = 'config/trajectory.yaml'
        self.v_max = v_max              
        self.a_max = a_max            
        self.a_lat_max = a_lat_max          
        self.smooth_acceleration = smooth_acceleration
        
        self.path_data = None
        self.trajectory = None
        
        self.get_logger().info('Trajectory Generator Node initialized')
        
        # Load path and generate trajectory
        if self.load_path_from_file():
            self.generate_trajectory()
        else:
            self.get_logger().error('Failed to load path data!')
    
    def load_path_from_file(self):
        
        # Get full path
        cwd = os.getcwd()
        filepath = os.path.join(cwd, self.input_file)
        
        # Check if file exists
        if not os.path.exists(filepath):
            self.get_logger().error(f'Input file not found: {filepath}')
            return False
        
        # Load YAML file
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load YAML: {e}')
            return False
        
        # Extract path data
        try:
            self.path_data = {
                'x': np.array(data['path']['x']),
                'y': np.array(data['path']['y']),
                'curvature': np.array(data['path']['curvature']),
                'distances': np.array(data['path']['distances']),
                'cumulative_distances': np.array(data['path']['cumulative_distances']),
                'total_length': data['path']['total_length']
            }
            
            self.get_logger().info(
                f'Loaded path: {len(self.path_data["x"])} points, '
                f'{self.path_data["total_length"]:.3f} m'
            )
            return True
            
        except KeyError as e:
            self.get_logger().error(f'Missing key in YAML: {e}')
            return False
    
    def generate_trajectory(self):
       
        self.get_logger().info('=== Generating Trajectory ===')
        start_time = time.time()
        
        # Extract path data
        x = self.path_data['x']
        y = self.path_data['y']
        curvatures = self.path_data['curvature']
        distances = self.path_data['distances']
        
        # Step 1: Compute curvature-based velocity limits
        v_curvature_limit = self.compute_curvature_velocity_limit(curvatures)
        
        # Step 2: Forward pass (acceleration-limited)
        v_forward = self.forward_pass(v_curvature_limit, distances)
        
        # Step 3: Backward pass (deceleration-limited)
        v_backward = self.backward_pass(v_curvature_limit, distances)
        
        # Step 4: Take minimum (optimal velocity profile)
        v_optimal = np.minimum(v_forward, v_backward)
        
        # Step 5: Compute timestamps
        timestamps = self.compute_timestamps(v_optimal, distances)
        
        # Step 6: Compute accelerations
        accelerations = self.compute_accelerations(v_optimal, timestamps)
        
        if self.smooth_acceleration:
            accelerations = self.smooth_acceleration_profile(accelerations)
        
        # Store trajectory
        self.trajectory = {
            'x': x,
            'y': y,
            'time': timestamps,
            'velocity': v_optimal,
            'acceleration': accelerations,
            'curvature': curvatures
        }
        
        elapsed = time.time() - start_time
        self.get_logger().info(f'Trajectory generated in {elapsed:.4f} s')
        
        # Validate constraints
        self.validate_constraints()
        
        # Log statistics
        self.log_trajectory_stats()
        
        # Save to file
        self.save_trajectory_to_file()
        
        # Create visualization
        self.create_trajectory_plots()
    
    def compute_curvature_velocity_limit(self, curvatures):
        """Compute velocity limit based on curvature and lateral acceleration.
        
        Formula: v_max = sqrt(a_lat_max / κ)
        """
        # Avoid division by zero (straight sections)
        curvatures_safe = np.maximum(curvatures, 1e-6)
        
        # Compute curvature-based limit
        v_from_curvature = np.sqrt(self.a_lat_max / curvatures_safe)
        
        # Apply global maximum velocity
        v_limit = np.minimum(v_from_curvature, self.v_max)
        
        return v_limit
    
    def forward_pass(self, v_limit, distances):
        """Forward integration: acceleration-limited velocity profile.
        
        Starting from rest, accelerate as much as possible while respecting
        velocity limits.
        """
        n = len(v_limit)
        v_forward = np.zeros(n)
        v_forward[0] = 0.0  # Start from rest
        
        for i in range(n - 1):
            # Maximum velocity achievable by accelerating
            # v² = v₀² + 2aΔs
            v_accel = np.sqrt(v_forward[i]**2 + 2 * self.a_max * distances[i+1])
            
            # Take minimum with velocity limit
            v_forward[i + 1] = min(v_accel, v_limit[i + 1])
        
        return v_forward
    
    def backward_pass(self, v_limit, distances):
        """Backward integration: deceleration-limited velocity profile.
        
        Starting from end at rest, work backwards to determine maximum
        velocity from which we can decelerate to end state.
        """
        n = len(v_limit)
        v_backward = np.zeros(n)
        v_backward[-1] = 0.0  # End at rest
        
        for i in range(n - 1, 0, -1):
            # Maximum velocity from which we can decelerate
            v_decel = np.sqrt(v_backward[i]**2 + 2 * self.a_max * distances[i])
            
            # Take minimum with velocity limit
            v_backward[i - 1] = min(v_decel, v_limit[i - 1])
        
        return v_backward
    
    def compute_timestamps(self, velocities, distances):
        """Compute timestamps using harmonic mean for better stability."""
        n = len(velocities)
        timestamps = np.zeros(n)
        timestamps[0] = 0.0
        
        for i in range(n - 1):
            v1 = velocities[i]
            v2 = velocities[i + 1]
            
            # Use harmonic mean for better numerical stability
            if v1 > 1e-6 and v2 > 1e-6:
                v_avg = 2 * v1 * v2 / (v1 + v2)
            else:
                v_avg = (v1 + v2) / 2.0
            
            # Ensure non-zero
            v_avg = max(v_avg, 1e-6)
            
            # Time for this segment: dt = distance / v_avg
            dt = distances[i + 1] / v_avg
            timestamps[i + 1] = timestamps[i] + dt
        
        return timestamps
    
    def compute_accelerations(self, velocities, timestamps):
        """Compute accelerations from velocity profile."""
        # Compute finite differences
        dt = np.diff(timestamps)
        dv = np.diff(velocities)
        
        # Avoid division by zero
        dt = np.where(dt < 1e-6, 1e-6, dt)
        
        accelerations = dv / dt
        
        # Append zero for last point (at rest)
        accelerations = np.concatenate([accelerations, [0.0]])
        
        return accelerations
    
    def smooth_acceleration_profile(self, accelerations):
        """Smooth acceleration profile using Savitzky-Golay filter."""
        # Savitzky-Golay filter parameters
        window_length = min(21, len(accelerations) // 10 * 2 + 1)
        window_length = max(window_length, 5)
        polyorder = 3
        
        try:
            accelerations_smooth = savgol_filter(
                accelerations, 
                window_length=window_length, 
                polyorder=polyorder,
                mode='nearest'
            )
            return accelerations_smooth
        except Exception as e:
            self.get_logger().warn(f'Smoothing failed: {e}')
            return accelerations
    
    def validate_constraints(self):

        traj = self.trajectory
        
        # Check velocity constraint
        max_vel = np.max(traj['velocity'])
        if max_vel > self.v_max + 0.01:
            self.get_logger().warn(
                f'Velocity constraint violated: {max_vel:.3f} > {self.v_max:.3f}'
            )
        
        # Check acceleration constraint
        max_accel = np.max(np.abs(traj['acceleration']))
        if max_accel > self.a_max + 0.01:
            self.get_logger().warn(
                f'Acceleration constraint violated: {max_accel:.3f} > {self.a_max:.3f}'
            )
        
        # Check lateral acceleration
        for i in range(len(traj['curvature'])):
            k = traj['curvature'][i]
            v = traj['velocity'][i]
            if k > 1e-6:
                a_lat = v**2 * k
                if a_lat > self.a_lat_max + 0.05:
                    self.get_logger().warn(
                        f'Lateral acceleration exceeded at point {i}: {a_lat:.3f}'
                    )
                    break
    
    def log_trajectory_stats(self):
        
        traj = self.trajectory
        
        self.get_logger().info('=== Trajectory Statistics ===')
        self.get_logger().info(f"Total Duration:    {traj['time'][-1]:.3f} s")
        self.get_logger().info(f"Path Length:       {self.path_data['total_length']:.3f} m")
        self.get_logger().info(f"Average Speed:     {self.path_data['total_length']/traj['time'][-1]:.3f} m/s")
        self.get_logger().info(f"Max Velocity:      {np.max(traj['velocity']):.3f} m/s")
        self.get_logger().info(f"Max Acceleration:  {np.max(np.abs(traj['acceleration'])):.3f} m/s²")
        self.get_logger().info(f"Max Curvature:     {np.max(traj['curvature']):.4f} m⁻¹")
        self.get_logger().info('=============================')
    
    def save_trajectory_to_file(self):
        
        traj = self.trajectory
        
        # Prepare data
        trajectory_data = {
            'metadata': {
                'description': 'Time-parameterized trajectory with curvature-aware velocity profile',
                'algorithm': 'Forward-backward pass with curvature constraints',
                'parameters': {
                    'v_max': float(self.v_max),
                    'a_max': float(self.a_max),
                    'a_lat_max': float(self.a_lat_max),
                    'smooth_acceleration': bool(self.smooth_acceleration),
                    'num_points': int(len(traj['x']))
                },
                'statistics': {
                    'total_duration': float(traj['time'][-1]),
                    'path_length': float(self.path_data['total_length']),
                    'avg_velocity': float(self.path_data['total_length'] / traj['time'][-1]),
                    'max_velocity': float(np.max(traj['velocity'])),
                    'max_acceleration': float(np.max(np.abs(traj['acceleration']))),
                    'max_curvature': float(np.max(traj['curvature']))
                }
            },
            'trajectory': {
                'x': traj['x'].tolist(),
                'y': traj['y'].tolist(),
                'time': traj['time'].tolist(),
                'velocity': traj['velocity'].tolist(),
                'acceleration': traj['acceleration'].tolist(),
                'curvature': traj['curvature'].tolist()
            }
        }
        
        # Get full path
        cwd = os.getcwd()
        config_dir = os.path.dirname(self.output_file)
        if config_dir:
            full_config_dir = os.path.join(cwd, config_dir)
            os.makedirs(full_config_dir, exist_ok=True)
        
        filepath = os.path.join(cwd, self.output_file)
        
        # Save to file
        try:
            with open(filepath, 'w') as f:
                yaml.dump(trajectory_data, f, default_flow_style=False, sort_keys=False)
            
            if os.path.exists(filepath):
                file_size = os.path.getsize(filepath)
                self.get_logger().info(f'Trajectory saved to: {filepath}')
                self.get_logger().info(f'File size: {file_size} bytes')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save trajectory: {e}')
    
    def create_trajectory_plots(self):
       
        traj = self.trajectory
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # Plot 1: Path with velocity colormap
        ax1 = axes[0, 0]
        scatter = ax1.scatter(traj['x'], traj['y'], c=traj['velocity'], 
                             cmap='viridis', s=20, edgecolors='none')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Trajectory with Velocity Profile')
        ax1.axis('equal')
        ax1.grid(True, alpha=0.3)
        plt.colorbar(scatter, ax=ax1, label='Velocity (m/s)')
        
        # Plot 2: Velocity vs Time
        ax2 = axes[0, 1]
        ax2.plot(traj['time'], traj['velocity'], 'b-', linewidth=2, label='Velocity')
        ax2.axhline(y=self.v_max, color='r', linestyle='--', 
                   label=f'v_max = {self.v_max} m/s', alpha=0.7)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('Velocity Profile')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        ax2.set_ylim(bottom=0)
        
        # Plot 3: Curvature vs Time
        ax3 = axes[1, 0]
        ax3.plot(traj['time'], traj['curvature'], 'g-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Curvature (m⁻¹)')
        ax3.set_title('Curvature Profile')
        ax3.grid(True, alpha=0.3)
        ax3.set_ylim(bottom=0)
        
        # Plot 4: Acceleration vs Time
        ax4 = axes[1, 1]
        ax4.plot(traj['time'], traj['acceleration'], 'r-', linewidth=2, label='Acceleration')
        ax4.axhline(y=self.a_max, color='k', linestyle='--', 
                   label=f'a_max = ±{self.a_max} m/s²', alpha=0.7, linewidth=2)
        ax4.axhline(y=-self.a_max, color='k', linestyle='--', alpha=0.7, linewidth=2)
        
        # Highlight violations if any
        violations = np.abs(traj['acceleration']) > self.a_max
        if np.any(violations):
            ax4.scatter(traj['time'][violations], traj['acceleration'][violations], 
                       color='orange', s=50, zorder=5, label='Violations')
        
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Acceleration (m/s²)')
        ax4.set_title('Acceleration Profile')
        ax4.grid(True, alpha=0.3)
        ax4.legend()
        
        plt.tight_layout()
        plt.savefig('trajectory_results.png', dpi=150)
        self.get_logger().info('Trajectory plots saved: trajectory_results.png')
        
        self.get_logger().info('Displaying plots. Close window to exit...')
        plt.show(block=True)
        self.get_logger().info('Plots closed.')


def main(args=None):
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='Generate time-optimal trajectory for differential drive robot',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument(
        '--v-max',
        type=float,
        default=0.8,
        help='Maximum linear velocity (m/s)'
    )
    
    parser.add_argument(
        '--a-max',
        type=float,
        default=0.45,
        help='Maximum acceleration (m/s²)'
    )
    
    parser.add_argument(
        '--a-lat-max',
        type=float,
        default=0.7,
        help='Maximum lateral acceleration (m/s²)'
    )
    
    parser.add_argument(
        '--no-smooth',
        dest='smooth_acceleration',
        action='store_false',
        default=True,
        help='Disable acceleration smoothing'
    )
    
    #parser.set_defaults(smooth_acceleration=True)
    
    #cli_args = parser.parse_args()
    cli_args, unknown = parser.parse_known_args()
   
    rclpy.init(args=args)
    
    node = TrajectoryGenerator(
        v_max=cli_args.v_max,
        a_max=cli_args.a_max,
        a_lat_max=cli_args.a_lat_max,
        smooth_acceleration=cli_args.smooth_acceleration
        )

    node.destroy_node()
    rclpy.shutdown()
    print('Execution completed successfully.\n')


if __name__ == '__main__':
    main()
