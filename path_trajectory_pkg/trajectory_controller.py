#!/usr/bin/env python3

import os
import argparse
import time
import yaml
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion


class PurePursuitController(Node):

    def __init__(self, lookahead_distance, k_lookahead, min_lookahead, max_lookahead, 
                        goal_to_tolerance, control_frequency):

        super().__init__('pure_pursuit_controller')
        
        # Configuration parameters
        self.trajectory_file = 'config/trajectory.yaml'
        self.robot_namespace = 'bcr_bot'
        self.lookahead_distance = lookahead_distance   
        self.k_lookahead = k_lookahead           
        self.min_lookahead = min_lookahead        
        self.max_lookahead = max_lookahead         
        self.goal_tolerance = goal_to_tolerance       
        self.control_frequency = control_frequency    
        
        # Build topic names with namespace
        self.odom_topic = f'/{self.robot_namespace}/odom'
        self.cmd_vel_topic = f'/{self.robot_namespace}/cmd_vel'
        
        # State variables
        self.robot_pose = None
        self.robot_velocity = 0.0
        self.trajectory = None
        self.current_idx = 0
        self.goal_reached = False
        self.start_time = None
        self.last_log_time = 0
        
        # Performance tracking
        self.cross_track_errors = deque(maxlen=1000)
        self.heading_errors = deque(maxlen=1000)
        self.velocity_errors = deque(maxlen=1000)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.lookahead_pub = self.create_publisher(Marker, '/lookahead_point', 10)
        self.path_pub = self.create_publisher(Path, '/trajectory_path', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, 
            self.control_loop
        )
        
        self.get_logger().info(f'Pure Pursuit Controller initialized for {self.robot_namespace}')
        
        # Load trajectory
        if self.load_trajectory():
            self.get_logger().info('Trajectory loaded. Waiting for odometry...')
        else:
            self.get_logger().error('Failed to load trajectory!')
    
    def load_trajectory(self):
       
        cwd = os.getcwd()
        filepath = os.path.join(cwd, self.trajectory_file)
        
        if not os.path.exists(filepath):
            self.get_logger().error(f'Trajectory file not found: {filepath}')
            return False
        
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load YAML: {e}')
            return False
        
        try:
            traj = data['trajectory']
            self.trajectory = {
                'x': np.array(traj['x']),
                'y': np.array(traj['y']),
                'time': np.array(traj['time']),
                'velocity': np.array(traj['velocity']),
            }
            
            self.get_logger().info(f"Loaded trajectory: {len(self.trajectory['x'])} points")
            
            # Publish trajectory path for visualization
            self.publish_trajectory_path()
            
            return True
            
        except KeyError as e:
            self.get_logger().error(f'Missing key in trajectory: {e}')
            return False
    
    def publish_trajectory_path(self):
        
        if self.trajectory is None:
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        for x, y in zip(self.trajectory['x'], self.trajectory['y']):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def odom_callback(self, msg):
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.robot_velocity = np.sqrt(vx**2 + vy**2)
        
        self.robot_pose = (x, y, yaw)
        
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info(f'Odometry received. Robot at ({x:.2f}, {y:.2f}). Starting control...')
    
    def control_loop(self):
       
        if self.robot_pose is None or self.trajectory is None:
            return
        
        if self.goal_reached:
            self.publish_velocity(0.0, 0.0)
            return
        
        # Find closest point on trajectory
        closest_idx = self.find_closest_point()
        self.current_idx = closest_idx
        
        # Log progress periodically
        current_time = time.time()
        if current_time - self.last_log_time > 2.0:
            progress = (closest_idx / len(self.trajectory['x'])) * 100
            self.get_logger().info(
                f'Progress: {progress:.1f}% (point {closest_idx}/{len(self.trajectory["x"])})'
                f'Velocity: {self.robot_velocity:.2f} m/s'
            )
            self.last_log_time = current_time
        
        # Check if goal reached
        if self.is_goal_reached():
            self.goal_reached = True
            elapsed = time.time() - self.start_time
            self.get_logger().info(f'GOAL REACHED in {elapsed:.2f} s!')
            self.log_performance_stats()
            self.publish_velocity(0.0, 0.0)
            return
        
        # Compute lookahead point
        lookahead_idx, lookahead_point = self.compute_lookahead_point(closest_idx)
        
        if lookahead_point is None:
            self.get_logger().warn('No valid lookahead point found')
            self.publish_velocity(0.0, 0.0)
            return
        
        # Compute control commands
        v, omega = self.pure_pursuit_control(lookahead_point, lookahead_idx)
        
        # Publish commands
        self.publish_velocity(v, omega)
        
        # Visualize lookahead point
        self.publish_lookahead_marker(lookahead_point)
        
        # Track errors
        self.track_errors(closest_idx, lookahead_idx)
    
    def find_closest_point(self):
        
        x_robot, y_robot, _ = self.robot_pose
        
        dx = self.trajectory['x'] - x_robot
        dy = self.trajectory['y'] - y_robot
        distances = np.sqrt(dx**2 + dy**2)
        
        closest_idx = np.argmin(distances)
        
        return closest_idx
    
    def compute_lookahead_point(self, start_idx):
        
        x_robot, y_robot, _ = self.robot_pose
        
        # Adaptive lookahead distance
        L = self.lookahead_distance + self.k_lookahead * self.robot_velocity
        L = np.clip(L, self.min_lookahead, self.max_lookahead)
        
        # Find first point at or beyond lookahead distance
        for i in range(start_idx, len(self.trajectory['x'])):
            dx = self.trajectory['x'][i] - x_robot
            dy = self.trajectory['y'][i] - y_robot
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance >= L:
                return i, (self.trajectory['x'][i], self.trajectory['y'][i])
        
        # If no point found, use last point
        last_idx = len(self.trajectory['x']) - 1
        return last_idx, (self.trajectory['x'][last_idx], self.trajectory['y'][last_idx])
    
    def pure_pursuit_control(self, lookahead_point, lookahead_idx):
        
        x_robot, y_robot, theta_robot = self.robot_pose
        x_goal, y_goal = lookahead_point
        
        # Transform goal to robot frame
        dx = x_goal - x_robot
        dy = y_goal - y_robot
        
        dx_robot = np.cos(theta_robot) * dx + np.sin(theta_robot) * dy
        dy_robot = -np.sin(theta_robot) * dx + np.cos(theta_robot) * dy
        
        L = np.sqrt(dx_robot**2 + dy_robot**2)
        
        if L < 1e-3:
            return 0.0, 0.0
        
        # Pure Pursuit curvature
        curvature = 2.0 * dy_robot / (L ** 2)
        
        # Get desired velocity from trajectory
        v_desired = self.trajectory['velocity'][lookahead_idx]
        
        # Limit angular velocity
        max_omega = 1.0
        if abs(curvature * v_desired) > max_omega:
            v_desired = max_omega / abs(curvature) if abs(curvature) > 1e-6 else v_desired

        

        v_desired = max(v_desired, 0.15)  # Minimum 0.15 m/s
        omega = v_desired * curvature
        omega = np.clip(omega, -max_omega, max_omega)  
        
        return v_desired, omega
    
    def is_goal_reached(self):
       
        # Must have progressed through at least 90% of trajectory
        trajectory_length = len(self.trajectory['x'])
        progress_threshold = int(0.95 * trajectory_length)
        
        if self.current_idx < progress_threshold:
            return False
        
        # Check distance to goal
        x_robot, y_robot, _ = self.robot_pose
        x_goal = self.trajectory['x'][-1]
        y_goal = self.trajectory['y'][-1]
        
        distance = np.sqrt((x_goal - x_robot)**2 + (y_goal - y_robot)**2)
        
        return distance < self.goal_tolerance
    
    def track_errors(self, closest_idx, lookahead_idx):
       
        x_robot, y_robot, theta_robot = self.robot_pose
        
        # Cross-track error
        x_closest = self.trajectory['x'][closest_idx]
        y_closest = self.trajectory['y'][closest_idx]
        cross_track_error = np.sqrt(
            (x_closest - x_robot)**2 + (y_closest - y_robot)**2
        )
        
        # Heading error
        dx = self.trajectory['x'][lookahead_idx] - x_robot
        dy = self.trajectory['y'][lookahead_idx] - y_robot
        desired_heading = np.arctan2(dy, dx)
        heading_error = self.normalize_angle(desired_heading - theta_robot)
        
        # Velocity error
        v_desired = self.trajectory['velocity'][closest_idx]
        velocity_error = abs(v_desired - self.robot_velocity)
        
        self.cross_track_errors.append(cross_track_error)
        self.heading_errors.append(abs(heading_error))
        self.velocity_errors.append(velocity_error)
    
    @staticmethod
    def normalize_angle(angle):
       
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def publish_velocity(self, v, omega):
       
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_vel_pub.publish(cmd)
    
    def publish_lookahead_marker(self, lookahead_point):
       
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lookahead'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(lookahead_point[0])
        marker.pose.position.y = float(lookahead_point[1])
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.lookahead_pub.publish(marker)
    
    def log_performance_stats(self):
       
        if len(self.cross_track_errors) == 0:
            self.get_logger().warn('No performance data collected')
            return
        
        cross_track_rms = np.sqrt(np.mean(np.array(self.cross_track_errors)**2))
        cross_track_max = np.max(self.cross_track_errors)
        cross_track_mean = np.mean(self.cross_track_errors)
        
        heading_rms = np.sqrt(np.mean(np.array(self.heading_errors)**2))
        heading_max = np.max(self.heading_errors)
        heading_mean = np.mean(self.heading_errors)
        
        velocity_rms = np.sqrt(np.mean(np.array(self.velocity_errors)**2))
        
        self.get_logger().info('=== Performance Statistics ===')
        self.get_logger().info(f"Cross-track Error (RMS):  {cross_track_rms:.4f} m")
        self.get_logger().info(f"Cross-track Error (Mean): {cross_track_mean:.4f} m")
        self.get_logger().info(f"Cross-track Error (Max):  {cross_track_max:.4f} m")
        self.get_logger().info(f"Heading Error (RMS):      {np.degrees(heading_rms):.2f} deg")
        self.get_logger().info(f"Heading Error (Mean):     {np.degrees(heading_mean):.2f} deg")
        self.get_logger().info(f"Heading Error (Max):      {np.degrees(heading_max):.2f} deg")
        self.get_logger().info(f"Velocity Error (RMS):     {velocity_rms:.4f} m/s")
        self.get_logger().info('=============================')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Pure Pursuit trajectory controller for differential drive',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument(
        '--lookahead-distance',
        type = float,
        default = 0.5,
        help = 'base look ahead distance'
    )

    parser.add_argument(
        '--k-lookahead',
        type = float,
        default = 0.1,
        help = 'velocity proportional gain'

    )

    parser.add_argument(
        '--min-lookahead',
        type =  float,
        default = 0.3,
        help = 'minimum lookahead distance'
    )

    parser.add_argument(
        '--max-lookahead',
        type = float,
        default = 1.0,
        help = 'maximum lookahead' 
    )

    parser.add_argument(
        '--goal-to-tolerance',
        type = float,
        default = 0.15,
        help = 'goal reached threshold'
    )

    parser.add_argument(
        '--control-frequency',
        type = float,
        default = 50.0,
        help = 'control loop frequency'
    )

    cli_args = parser.parse_args()

    rclpy.init(args=args)

    node = PurePursuitController(
        lookahead_distance = cli_args.lookahead_distance,
        k_lookahead = cli_args.k_lookahead,
        min_lookahead = cli_args.min_lookahead,
        max_lookahead = cli_args.max_lookahead,
        goal_to_tolerance = cli_args.goal_to_tolerance,
        control_frequency = cli_args.control_frequency
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass 
    finally:
        node.get_logger().info('Shutting down controller...')
        # Try to stop the robot before shutting down
        try:
            node.publish_velocity(0.0, 0.0)
        except Exception:
            pass  # Context may already be invalid
        
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('Controller shutdown complete.\n')


if __name__ == '__main__':
    main()
