#!/usr/bin/env python3

import os
import sys
import time
import yaml
import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node


class PathSmoother(Node):

    def __init__(self):

        super().__init__('path_smoother')
        
        # Declare parameter for saving to file
        self.declare_parameter('save_to_file', True)
        self.save_to_file = self.get_parameter('save_to_file').value
        
        # Define waypoints

        self.waypoints = np.array([
            [0.0, 0.0],          # Point 1: Start at origin (spawn)
            [0.374, -6.042],     # Point 2: Southeast (obstacle-free)
            [-3.258, -5.959],    # Point 3: Southwest
            [-3.140, -0.708], 
            [0.0, 0.0],
        ])
        
        self.smooth_path = None
        self.metrics = {}
        
        self.get_logger().info('Path Smoother Node initialized')
        
        # Compute smooth path
        self.compute_smooth_path()

    def compute_smooth_path(self):
        start_time = time.time()
        
        x_waypoints = self.waypoints[:, 0]
        y_waypoints = self.waypoints[:, 1]
        
        # B-spline interpolation
        tck, u = splprep([x_waypoints, y_waypoints], s=0, k=3)
        
        # Generate 500 dense points
        u_new = np.linspace(0, 1, 500)
        x_smooth, y_smooth = splev(u_new, tck)
        
        self.smooth_path = (x_smooth, y_smooth, tck, u_new)
        
        # Calculate metrics
        self.calculate_metrics()
        self.metrics['computation_time'] = time.time() - start_time
        
        # Log metrics
        self.log_metrics()
        
        # Create plot
        self.create_plot()
        
        # Save to file
        if self.save_to_file:
            self.save_path_to_file()

    def calculate_metrics(self):
        x_smooth, y_smooth, tck, u_new = self.smooth_path
        
        # Path lengths
        original_length = self.compute_path_length(
            self.waypoints[:, 0], 
            self.waypoints[:, 1]
        )
        smooth_length = self.compute_path_length(x_smooth, y_smooth)
        
        # Curvature
        curvatures = self.compute_curvature(tck, u_new)
        max_curvature = np.max(np.abs(curvatures))
        avg_curvature = np.mean(np.abs(curvatures))
        curvature_smoothness = np.sum(np.diff(curvatures)**2)
        
        self.metrics = {
            'original_length': original_length,
            'smooth_length': smooth_length,
            'max_curvature': max_curvature,
            'avg_curvature': avg_curvature,
            'curvature_smoothness': curvature_smoothness
        }

    @staticmethod
    def compute_path_length(x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        return np.sum(np.sqrt(dx**2 + dy**2))

    @staticmethod
    def compute_curvature(tck, u):
        dx_du = splev(u, tck, der=1)[0]
        dy_du = splev(u, tck, der=1)[1]
        d2x_du2 = splev(u, tck, der=2)[0]
        d2y_du2 = splev(u, tck, der=2)[1]
        
        numerator = np.abs(dx_du * d2y_du2 - dy_du * d2x_du2)
        denominator = (dx_du**2 + dy_du**2)**(3/2)
        denominator = np.where(denominator < 1e-10, 1e-10, denominator)
        
        return numerator / denominator

    def log_metrics(self):

        self.get_logger().info('=== Path Smoothing Metrics ===')
        self.get_logger().info(f"Original Length: {self.metrics['original_length']:.3f} m")
        self.get_logger().info(f"Smooth Length: {self.metrics['smooth_length']:.3f} m")
        self.get_logger().info(f"Max Curvature: {self.metrics['max_curvature']:.4f} m⁻¹")
        self.get_logger().info(f"Avg Curvature: {self.metrics['avg_curvature']:.4f} m⁻¹")
        self.get_logger().info(f"Smoothness: {self.metrics['curvature_smoothness']:.6f}")
        self.get_logger().info(f"Time: {self.metrics['computation_time']:.4f} s")
        self.get_logger().info('==============================')

    def save_path_to_file(self):

        x_smooth, y_smooth, tck, u_new = self.smooth_path
        curvatures = self.compute_curvature(tck, u_new)
        
        # Calculate segment distances
        dx = np.diff(x_smooth)
        dy = np.diff(y_smooth)
        distances = np.sqrt(dx**2 + dy**2)
        distances = np.concatenate([[0], distances])
        cumulative_distances = np.cumsum(distances)
        
        # Prepare data dictionary
        path_data = {
            'metadata': {
                'description': 'Smooth path data for trajectory generation',
                'algorithm': 'B-spline interpolation (scipy.interpolate.splprep)',
                'parameters': {
                    's': 0,
                    'k': 3,
                    'num_points': int(len(x_smooth))
                },
                'metrics': {
                    'original_length': float(self.metrics['original_length']),
                    'smooth_length': float(self.metrics['smooth_length']),
                    'max_curvature': float(self.metrics['max_curvature']),
                    'avg_curvature': float(self.metrics['avg_curvature']),
                    'smoothness': float(self.metrics['curvature_smoothness'])
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
            self.get_logger().info(f'Smooth path saved to: {filepath}')
            self.get_logger().info(f'File size: {file_size} bytes')
        else:
            self.get_logger().error("File was not created!")

    def create_plot(self):

        x_smooth, y_smooth, tck, u_new = self.smooth_path
        
        plt.figure(figsize=(12, 5))
        
        # Path comparison
        plt.subplot(1, 2, 1)
        plt.plot(self.waypoints[:, 0], self.waypoints[:, 1], 
                 'ro-', label='Waypoints', markersize=8, linewidth=2)
        plt.plot(x_smooth, y_smooth, 'b-', label='Smoothed', linewidth=2)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Path Smoothing')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        # Curvature profile
        plt.subplot(1, 2, 2)
        curvatures = self.compute_curvature(tck, u_new)
        plt.plot(u_new, curvatures, 'g-', linewidth=2)
        plt.xlabel('Parameter (u)')
        plt.ylabel('Curvature (m⁻¹)')
        plt.title('Curvature Profile')
        plt.grid(True)
        
        plt.tight_layout()
        
        # Save plot
        plt.savefig('path_smoothing.png', dpi=150)
        self.get_logger().info('Plot saved: path_smoothing.png')
        
        # Display plot
        self.get_logger().info('Displaying plot. Close window to continue...')
        plt.show(block=True)
        self.get_logger().info('Plot closed.')


def main(args=None):
    rclpy.init(args=args)
    
    node = PathSmoother()
    node.destroy_node()
    rclpy.shutdown()
    print('Execution completed successfully.\n')


if __name__ == '__main__':
    main()
