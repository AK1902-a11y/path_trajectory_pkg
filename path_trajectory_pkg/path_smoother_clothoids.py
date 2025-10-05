#!/usr/bin/env python3

import os
import argparse
import time
import yaml
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List, Optional

import rclpy
from rclpy.node import Node

from pyclothoids import Clothoid


class ClothoidPathSmoother(Node):
    
    def __init__(self, save_to_file, num_samples_per_segment):
        super().__init__('clothoid_path_smoother')
        
        # Configuration parameters
        self.save_to_file = save_to_file
        self.num_samples_per_segment = num_samples_per_segment
        
        # Define waypoints
        self.waypoints = np.array([
            [0.0, 0.0],          # Point 1: Start at origin (spawn)
            [0.374, -6.042],     # Point 2: Southeast (obstacle-free)
            [-3.258, -5.959],    # Point 3: Southwest
        ])
        
        self.smooth_path: Optional[Tuple] = None
        self.clothoid_segments: List[Clothoid] = []
        self.metrics: dict = {}
        
        self.get_logger().info('Clothoid Path Smoother Node initialized')
        
        # Compute smooth path
        self.compute_smooth_path()
    
    def compute_smooth_path(self) -> None:

        start_time = time.time()
        
        # Compute tangent angles at each waypoint for G1 continuity
        tangent_angles = self.compute_tangent_angles(self.waypoints)
        
        # Storage for concatenated path data
        all_x: List[float] = []
        all_y: List[float] = []
        all_curvatures: List[float] = []
        all_distances: List[float] = []
        
        self.clothoid_segments = []
        
        # Generate clothoid segments between consecutive waypoints
        for i in range(len(self.waypoints) - 1):
            x0, y0 = self.waypoints[i]
            x1, y1 = self.waypoints[i + 1]
            theta0 = tangent_angles[i]
            theta1 = tangent_angles[i + 1]
            
            # Create clothoid using G1 Hermite interpolation
            clothoid = Clothoid.G1Hermite(x0, y0, theta0, x1, y1, theta1)
            self.clothoid_segments.append(clothoid)
            
            # Sample points along this clothoid
            x_seg, y_seg = clothoid.SampleXY(self.num_samples_per_segment)
            
            # Compute curvatures along segment
            curvatures_seg = self.compute_segment_curvatures(
                clothoid, 
                self.num_samples_per_segment
            )
            
            # Compute distances between consecutive points
            dx_seg = np.diff(x_seg)
            dy_seg = np.diff(y_seg)
            dist_seg = np.sqrt(dx_seg**2 + dy_seg**2)
            dist_seg = np.concatenate([[0], dist_seg])
            
            # Append to global arrays (skip first point of subsequent segments)
            if i == 0:
                all_x.extend(x_seg)
                all_y.extend(y_seg)
                all_curvatures.extend(curvatures_seg)
                all_distances.extend(dist_seg)
            else:
                all_x.extend(x_seg[1:])
                all_y.extend(y_seg[1:])
                all_curvatures.extend(curvatures_seg[1:])
                all_distances.extend(dist_seg[1:])
        
        # Convert to numpy arrays
        x_smooth = np.array(all_x)
        y_smooth = np.array(all_y)
        curvatures = np.array(all_curvatures)
        distances = np.array(all_distances)
        cumulative_distances = np.cumsum(distances)
        
        self.smooth_path = (
            x_smooth, 
            y_smooth, 
            curvatures, 
            distances, 
            cumulative_distances
        )
        
        # Calculate metrics
        self.calculate_metrics()
        self.metrics['computation_time'] = time.time() - start_time
        self.metrics['num_segments'] = len(self.clothoid_segments)
        
        # Log metrics
        self.log_metrics()
        
        # Create plot
        self.create_plot()
        
        # Save to file
        if self.save_to_file:
            self.save_path_to_file()
    
    def compute_tangent_angles(self, waypoints: np.ndarray) -> np.ndarray:
        n = len(waypoints)
        angles = np.zeros(n)
        
        for i in range(n):
            if i == 0:
                # First point: forward difference
                dx = waypoints[i + 1, 0] - waypoints[i, 0]
                dy = waypoints[i + 1, 1] - waypoints[i, 1]
            elif i == n - 1:
                # Last point: backward difference
                dx = waypoints[i, 0] - waypoints[i - 1, 0]
                dy = waypoints[i, 1] - waypoints[i - 1, 1]
            else:
                # Middle points: central difference (smoother)
                dx_prev = waypoints[i, 0] - waypoints[i - 1, 0]
                dy_prev = waypoints[i, 1] - waypoints[i - 1, 1]
                dx_next = waypoints[i + 1, 0] - waypoints[i, 0]
                dy_next = waypoints[i + 1, 1] - waypoints[i, 1]
                dx = (dx_prev + dx_next) / 2.0
                dy = (dy_prev + dy_next) / 2.0
            
            angles[i] = np.arctan2(dy, dx)
        
        return angles
    
    def compute_segment_curvatures(self, clothoid: Clothoid, num_samples: int) -> List[float]:
        
        s_samples = np.linspace(0, clothoid.length, num_samples)
        curvatures = [
            clothoid.KappaStart + clothoid.dk * s 
            for s in s_samples
        ]
        return curvatures
    
    def calculate_metrics(self) -> None:

        x_smooth, y_smooth, curvatures, _, cumulative_distances = self.smooth_path
        
        # Path lengths
        original_length = self.compute_path_length(
            self.waypoints[:, 0], 
            self.waypoints[:, 1]
        )
        smooth_length = float(cumulative_distances[-1])
        
        # Curvature statistics
        max_curvature = float(np.max(np.abs(curvatures)))
        avg_curvature = float(np.mean(np.abs(curvatures)))
        curvature_smoothness = float(np.sum(np.diff(curvatures)**2))
        
        self.metrics = {
            'original_length': original_length,
            'smooth_length': smooth_length,
            'max_curvature': max_curvature,
            'avg_curvature': avg_curvature,
            'curvature_smoothness': curvature_smoothness
        }
    
    @staticmethod
    def compute_path_length(x: np.ndarray, y: np.ndarray) -> float:

        dx = np.diff(x)
        dy = np.diff(y)
        return float(np.sum(np.sqrt(dx**2 + dy**2)))
    
    def log_metrics(self) -> None:
      
        self.get_logger().info('=== Clothoid Path Smoothing Metrics ===')
        self.get_logger().info(f"Segments: {self.metrics.get('num_segments', 0)}")
        self.get_logger().info(f"Original Length: {self.metrics['original_length']:.3f} m")
        self.get_logger().info(f"Smooth Length: {self.metrics['smooth_length']:.3f} m")
        self.get_logger().info(f"Max Curvature: {self.metrics['max_curvature']:.4f} m⁻¹")
        self.get_logger().info(f"Avg Curvature: {self.metrics['avg_curvature']:.4f} m⁻¹")
        self.get_logger().info(f"Smoothness: {self.metrics['curvature_smoothness']:.6f}")
        self.get_logger().info(f"Time: {self.metrics['computation_time']:.4f} s")
        self.get_logger().info('=' * 39)
    
    def save_path_to_file(self) -> None:
     
        x_smooth, y_smooth, curvatures, distances, cumulative_distances = self.smooth_path
        
        # Prepare data dictionary
        path_data = {
            'metadata': {
                'description': 'Smooth path using Clothoid curves (time-optimal for differential drive)',
                'algorithm': 'G1 Hermite Clothoid Interpolation',
                'library': 'pyclothoids',
                'parameters': {
                    'num_segments': int(len(self.clothoid_segments)),
                    'samples_per_segment': int(self.num_samples_per_segment),
                    'total_points': int(len(x_smooth))
                },
                'metrics': {
                    'original_length': float(self.metrics['original_length']),
                    'smooth_length': float(self.metrics['smooth_length']),
                    'max_curvature': float(self.metrics['max_curvature']),
                    'avg_curvature': float(self.metrics['avg_curvature']),
                    'smoothness': float(self.metrics['curvature_smoothness']),
                    'computation_time': float(self.metrics['computation_time'])
                }
            },
            'waypoints': {
                'x': self.waypoints[:, 0].tolist(),
                'y': self.waypoints[:, 1].tolist()
            },
            'path': {
                'x': x_smooth.tolist(),
                'y': y_smooth.tolist(),
                'curvature': curvatures.tolist(),
                'distances': distances.tolist(),
                'cumulative_distances': cumulative_distances.tolist(),
                'total_length': float(cumulative_distances[-1])
            }
        }
        
        # Get current working directory and create config directory
        cwd = os.getcwd()
        config_dir = os.path.join(cwd, 'config')
        
        try:
            os.makedirs(config_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().error(f"Failed to create config directory: {e}")
            return
        
        # Save to file
        filepath = os.path.join(config_dir, 'smooth_path.yaml')
        
        try:
            with open(filepath, 'w') as f:
                yaml.dump(path_data, f, default_flow_style=False, sort_keys=False)
        except Exception as e:
            self.get_logger().error(f"Failed to write YAML file: {e}")
            return
        
        # Verify file exists
        if os.path.exists(filepath):
            file_size = os.path.getsize(filepath)
            self.get_logger().info(f'Clothoid path saved to: {filepath}')
            self.get_logger().info(f'File size: {file_size} bytes')
        else:
            self.get_logger().error("File was not created!")
    
    def create_plot(self) -> None:

        x_smooth, y_smooth, curvatures, _, cumulative_distances = self.smooth_path
        
        plt.figure(figsize=(12, 5))
        
        # Path comparison
        plt.subplot(1, 2, 1)
        plt.plot(self.waypoints[:, 0], self.waypoints[:, 1], 
                 'ro-', label='Waypoints', markersize=8, linewidth=2)
        plt.plot(x_smooth, y_smooth, 'b-', label='Clothoid Path', linewidth=2)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Clothoid Path Smoothing')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        # Curvature profile
        plt.subplot(1, 2, 2)
        plt.plot(cumulative_distances, curvatures, 'g-', linewidth=2)
        plt.xlabel('Arc Length (m)')
        plt.ylabel('Curvature (m⁻¹)')
        plt.title('Curvature Profile (Linear per Segment)')
        plt.grid(True)
        plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        plt.savefig('path_smoothing_clothoid.png', dpi=150)
        self.get_logger().info('Plot saved: path_smoothing_clothoid.png')
        
        # Display plot
        self.get_logger().info('Displaying plot. Close window to continue...')
        plt.show(block=True)
        self.get_logger().info('Plot closed.')


def main(args=None):

    parser = argparse.ArgumentParser(
        description='Path Smoother using Clothoids for differential drive',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument(
        '--samples',
        type=int,
        default=50,
        help='number of samples per segment'
    )

    parser.add_argument(
        '--no-save',
        dest='no_save',
        action='store_false',
        default=True,
        help='Disable saving configuration parameters in yaml file'
    )

    cli_args, unknown = parser.parse_known_args()

    rclpy.init(args=args)
    
    node = ClothoidPathSmoother(
        save_to_file=cli_args.no_save,
        num_samples_per_segment = cli_args.samples
    )
    node.destroy_node()
    rclpy.shutdown()
    print('Execution completed successfully.\n')


if __name__ == '__main__':
    main()
