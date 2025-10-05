#!/usr/bin/env python3

import os
import argparse
import time
import yaml
import numpy as np
from collections import deque
from typing import Tuple, Optional, Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion


class RegulatedPurePursuitController(Node):
    """Regulated Pure Pursuit trajectory tracking controller for differential drive robots."""
    
    def __init__(self, lookahead_time, min_lookahead, max_lookahead,
                 desired_linear_vel, min_approach_vel, approach_vel_scaling_dist,
                 use_velocity_scaling, min_radius_for_regulation, min_regulated_speed,
                 goal_tolerance, control_frequency, max_angular_vel):
        
        super().__init__('regulated_pure_pursuit_controller')
        
        # Configuration parameters
        self.trajectory_file = 'config/trajectory.yaml'
        self.robot_namespace = 'bcr_bot'
        
        # Lookahead parameters
        self.lookahead_time = lookahead_time
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        
        # Velocity regulation parameters
        self.desired_linear_vel = desired_linear_vel
        self.min_approach_vel = min_approach_vel
        self.approach_vel_scaling_dist = approach_vel_scaling_dist
        
        # Curvature regulation parameters
        self.use_velocity_scaling = use_velocity_scaling
        self.min_radius_for_regulation = min_radius_for_regulation
        self.min_regulated_speed = min_regulated_speed
        
        # Control parameters
        self.goal_tolerance = goal_tolerance
        self.control_frequency = control_frequency
        self.max_angular_vel = max_angular_vel
        
        # Build topic names
        self.odom_topic = f'/{self.robot_namespace}/odom'
        self.cmd_vel_topic = f'/{self.robot_namespace}/cmd_vel'
        
        # State variables
        self.robot_pose: Optional[Tuple[float, float, float]] = None
        self.robot_velocity: float = 0.0
        self.trajectory: Optional[Dict] = None
        self.current_idx: int = 0
        self.goal_reached: bool = False
        self.start_time: Optional[float] = None
        self.last_log_time: float = 0.0
        
        # Performance tracking
        self.cross_track_errors: deque = deque(maxlen=1000)
        self.heading_errors: deque = deque(maxlen=1000)
        self.velocity_errors: deque = deque(maxlen=1000)
        self.regulated_velocities: deque = deque(maxlen=1000)
        
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
        
        self.get_logger().info(f'Regulated Pure Pursuit Controller initialized for {self.robot_namespace}')
        self.get_logger().info(f'Parameters: lookahead_time={self.lookahead_time}s, '
                              f'v_max={self.desired_linear_vel}m/s, regulation={self.use_velocity_scaling}')
        
        # Load trajectory
        if self.load_trajectory():
            self.get_logger().info('Trajectory loaded. Waiting for odometry...')
        else:
            self.get_logger().error('Failed to load trajectory!')
    
    def load_trajectory(self) -> bool:
        """Load trajectory from YAML file."""
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
                'curvature': np.array(traj.get('curvature', [0.0] * len(traj['x'])))
            }
            
            self.get_logger().info(
                f"Loaded trajectory: {len(self.trajectory['x'])} points, "
                f"{self.trajectory['time'][-1]:.2f} s duration"
            )
            
            self.publish_trajectory_path()
            return True
            
        except KeyError as e:
            self.get_logger().error(f'Missing key in trajectory: {e}')
            return False
    
    def publish_trajectory_path(self) -> None:
        """Publish trajectory as Path message for visualization."""
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
    
    def odom_callback(self, msg: Odometry) -> None:
        """Process odometry messages."""
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
            self.get_logger().info(
                f'Odometry received. Robot at ({x:.2f}, {y:.2f}). Starting control...'
            )
    
    def control_loop(self) -> None:
        """Main control loop - called at control frequency."""
        if self.robot_pose is None or self.trajectory is None:
            return
        
        if self.goal_reached:
            self.publish_velocity(0.0, 0.0)
            return
        
        closest_idx = self.find_closest_point()
        self.current_idx = closest_idx
        
        current_time = time.time()
        if current_time - self.last_log_time > 2.0:
            progress = (closest_idx / len(self.trajectory['x'])) * 100
            self.get_logger().info(
                f'Progress: {progress:.1f}% | Point: {closest_idx}/{len(self.trajectory["x"])} | '
                f'Velocity: {self.robot_velocity:.2f} m/s'
            )
            self.last_log_time = current_time
        
        if self.is_goal_reached():
            self.goal_reached = True
            elapsed = time.time() - self.start_time
            self.get_logger().info(f'GOAL REACHED in {elapsed:.2f} s!')
            self.log_performance_stats()
            self.publish_velocity(0.0, 0.0)
            return
        
        lookahead_dist = self.compute_adaptive_lookahead_distance()
        lookahead_idx, lookahead_point = self.compute_lookahead_point(closest_idx, lookahead_dist)
        
        if lookahead_point is None:
            self.get_logger().warn('No valid lookahead point found')
            self.publish_velocity(0.0, 0.0)
            return
        
        v_desired = self.trajectory['velocity'][lookahead_idx]
        v_regulated = self.regulate_velocity(v_desired, lookahead_idx, closest_idx, lookahead_dist)
        v_cmd, omega_cmd = self.compute_control_commands(lookahead_point, v_regulated)
        
        self.publish_velocity(v_cmd, omega_cmd)
        self.publish_lookahead_marker(lookahead_point)
        self.track_errors(closest_idx, lookahead_idx, v_regulated)
        self.regulated_velocities.append(v_regulated)
    
    def compute_adaptive_lookahead_distance(self) -> float:
        """Compute velocity-scaled adaptive lookahead distance."""
        L = self.lookahead_time * max(self.robot_velocity, 0.1)
        L = np.clip(L, self.min_lookahead, self.max_lookahead)
        return L
    
    def regulate_velocity(self, v_desired: float, lookahead_idx: int, 
                         closest_idx: int, lookahead_dist: float) -> float:
     
        v_regulated = v_desired
        
        # Regulation 1: Curvature-based velocity scaling
        if self.use_velocity_scaling:
            curvature = abs(self.trajectory['curvature'][lookahead_idx])
            
            if curvature > 1e-6:
                radius = 1.0 / curvature
                
                if radius < self.min_radius_for_regulation:
                    scaling_factor = radius / self.min_radius_for_regulation
                    v_regulated = v_regulated * scaling_factor
                    v_regulated = max(v_regulated, self.min_regulated_speed)
        
        # Regulation 2: Goal approach velocity scaling
        distance_to_goal = self.compute_distance_to_goal(closest_idx)
        
        if distance_to_goal < self.approach_vel_scaling_dist:
            scaling_factor = distance_to_goal / self.approach_vel_scaling_dist
            v_approach = self.desired_linear_vel * scaling_factor
            v_approach = max(v_approach, self.min_approach_vel)
            v_regulated = min(v_regulated, v_approach)
        
        # Regulation 3: Clamp to desired maximum velocity
        v_regulated = min(v_regulated, self.desired_linear_vel)
        v_regulated = max(v_regulated, self.min_approach_vel)
        
        return v_regulated
    
    def compute_distance_to_goal(self, current_idx: int) -> float:
        """Compute remaining distance along trajectory to goal."""
        x_robot, y_robot, _ = self.robot_pose
        
        remaining_distance = 0.0
        for i in range(current_idx, len(self.trajectory['x']) - 1):
            dx = self.trajectory['x'][i + 1] - self.trajectory['x'][i]
            dy = self.trajectory['y'][i + 1] - self.trajectory['y'][i]
            remaining_distance += np.sqrt(dx**2 + dy**2)
        
        dx_to_path = self.trajectory['x'][current_idx] - x_robot
        dy_to_path = self.trajectory['y'][current_idx] - y_robot
        remaining_distance += np.sqrt(dx_to_path**2 + dy_to_path**2)
        
        return remaining_distance
    
    def find_closest_point(self) -> int:
        """Find closest point on trajectory to robot."""
        x_robot, y_robot, _ = self.robot_pose
        dx = self.trajectory['x'] - x_robot
        dy = self.trajectory['y'] - y_robot
        distances = np.sqrt(dx**2 + dy**2)
        return int(np.argmin(distances))
    
    def compute_lookahead_point(self, start_idx: int, lookahead_dist: float) -> Tuple[int, Optional[Tuple[float, float]]]:
        """Compute lookahead point on trajectory."""
        x_robot, y_robot, _ = self.robot_pose
        
        for i in range(start_idx, len(self.trajectory['x'])):
            dx = self.trajectory['x'][i] - x_robot
            dy = self.trajectory['y'][i] - y_robot
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance >= lookahead_dist:
                return i, (self.trajectory['x'][i], self.trajectory['y'][i])
        
        last_idx = len(self.trajectory['x']) - 1
        return last_idx, (self.trajectory['x'][last_idx], self.trajectory['y'][last_idx])
    
    def compute_control_commands(self, lookahead_point: Tuple[float, float], 
                                v_regulated: float) -> Tuple[float, float]:
        """Compute Pure Pursuit control commands with regulated velocity."""
        x_robot, y_robot, theta_robot = self.robot_pose
        x_goal, y_goal = lookahead_point
        
        dx = x_goal - x_robot
        dy = y_goal - y_robot
        
        dx_robot = np.cos(theta_robot) * dx + np.sin(theta_robot) * dy
        dy_robot = -np.sin(theta_robot) * dx + np.cos(theta_robot) * dy
        
        L = np.sqrt(dx_robot**2 + dy_robot**2)
        
        if L < 1e-3:
            return 0.0, 0.0
        
        curvature = 2.0 * dy_robot / (L ** 2)
        omega = v_regulated * curvature
        omega = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)
        
        return v_regulated, omega
    
    def is_goal_reached(self) -> bool:
        trajectory_length = len(self.trajectory['x'])
        progress_threshold = int(0.95 * trajectory_length)
        
        if self.current_idx < progress_threshold:
            return False
        
        x_robot, y_robot, _ = self.robot_pose
        x_goal = self.trajectory['x'][-1]
        y_goal = self.trajectory['y'][-1]
        
        distance = np.sqrt((x_goal - x_robot)**2 + (y_goal - y_robot)**2)
        return distance < self.goal_tolerance
    
    def track_errors(self, closest_idx: int, lookahead_idx: int, v_regulated: float) -> None:
        """Track performance errors for statistics."""
        x_robot, y_robot, theta_robot = self.robot_pose
        
        x_closest = self.trajectory['x'][closest_idx]
        y_closest = self.trajectory['y'][closest_idx]
        cross_track_error = np.sqrt((x_closest - x_robot)**2 + (y_closest - y_robot)**2)
        
        dx = self.trajectory['x'][lookahead_idx] - x_robot
        dy = self.trajectory['y'][lookahead_idx] - y_robot
        desired_heading = np.arctan2(dy, dx)
        heading_error = self.normalize_angle(desired_heading - theta_robot)
        
        velocity_error = abs(v_regulated - self.robot_velocity)
        
        self.cross_track_errors.append(cross_track_error)
        self.heading_errors.append(abs(heading_error))
        self.velocity_errors.append(velocity_error)
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-π, π]."""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def publish_velocity(self, v: float, omega: float) -> None:
        """Publish velocity command."""
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_vel_pub.publish(cmd)
    
    def publish_lookahead_marker(self, lookahead_point: Tuple[float, float]) -> None:
        """Publish lookahead point marker for visualization."""
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
    
    def log_performance_stats(self) -> None:
        """Log performance statistics."""
        if len(self.cross_track_errors) == 0:
            self.get_logger().warn('No performance data collected')
            return
        
        cross_track = np.array(self.cross_track_errors)
        heading = np.array(self.heading_errors)
        velocity = np.array(self.velocity_errors)
        regulated_vels = np.array(self.regulated_velocities)
        
        cross_track_rms = np.sqrt(np.mean(cross_track**2))
        cross_track_max = np.max(cross_track)
        cross_track_mean = np.mean(cross_track)
        
        heading_rms = np.sqrt(np.mean(heading**2))
        heading_max = np.max(heading)
        heading_mean = np.mean(heading)
        
        velocity_rms = np.sqrt(np.mean(velocity**2))
        velocity_mean = np.mean(velocity)
        
        regulated_vel_mean = np.mean(regulated_vels)
        regulated_vel_max = np.max(regulated_vels)
        
        self.get_logger().info('=== Regulated Pure Pursuit Performance ===')
        self.get_logger().info(f"Cross-track Error (RMS):  {cross_track_rms:.4f} m")
        self.get_logger().info(f"Cross-track Error (Mean): {cross_track_mean:.4f} m")
        self.get_logger().info(f"Cross-track Error (Max):  {cross_track_max:.4f} m")
        self.get_logger().info(f"Heading Error (RMS):      {np.degrees(heading_rms):.2f}°")
        self.get_logger().info(f"Heading Error (Mean):     {np.degrees(heading_mean):.2f}°")
        self.get_logger().info(f"Heading Error (Max):      {np.degrees(heading_max):.2f}°")
        self.get_logger().info(f"Velocity Error (RMS):     {velocity_rms:.4f} m/s")
        self.get_logger().info(f"Velocity Error (Mean):    {velocity_mean:.4f} m/s")
        self.get_logger().info(f"Regulated Velocity (Mean): {regulated_vel_mean:.3f} m/s")
        self.get_logger().info(f"Regulated Velocity (Max):  {regulated_vel_max:.3f} m/s")
        self.get_logger().info('==========================================')


def main(args=None):
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Regulated Pure Pursuit controller for differential drive robot',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument(
        '--lookahead-time',
        type=float, default=1.5,
        help='Lookahead time (s)'
    )

    parser.add_argument(
        '--min-lookahead',
        type=float,
        default=0.3,
        help='Minimum lookahead distance (m)'
    )
    
    parser.add_argument(
        '--max-lookahead',
        type=float, 
        default=1.5,
        help='Maximum lookahead distance (m)'
    )

    parser.add_argument(
        '--desired-linear-vel', 
        type=float, 
        default=0.8,
        help='Maximum desired velocity (m/s)'
    )

    parser.add_argument(
        '--min-approach-vel', 
        type=float, 
        default=0.25,
        help='Minimum approach velocity (m/s)'
    )

    parser.add_argument(
        '--approach-vel-scaling-dist',
        type=float, 
        default=0.6,
        help='Distance to start goal approach scaling (m)'
    )

    parser.add_argument(
        '--no-velocity-scaling', 
        dest='use_velocity_scaling',
        action='store_false', 
        default=True,
        help='Disable curvature-based velocity scaling'
    )

    parser.add_argument(
        '--min-radius-for-regulation', 
        type=float, 
        default=0.5,
        help='Minimum radius for regulation (m)'
    )

    parser.add_argument(
        '--min-regulated-speed', 
        type=float, 
        default=0.15,
        help='Minimum regulated speed (m/s)'
    )

    parser.add_argument(
        '--goal-tolerance', 
        type=float, 
        default=0.15,
        help='Goal reached threshold (m)'
    )

    parser.add_argument(
        '--control-frequency', 
        type=float, 
        default=50.0,
        help='Control loop frequency (Hz)'
    )

    parser.add_argument(
        '--max-angular-vel', 
        type=float, 
        default=1.5,
        help='Maximum angular velocity (rad/s)'
    )
    
    cli_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    node = RegulatedPurePursuitController(
        lookahead_time=cli_args.lookahead_time,
        min_lookahead=cli_args.min_lookahead,
        max_lookahead=cli_args.max_lookahead,
        desired_linear_vel=cli_args.desired_linear_vel,
        min_approach_vel=cli_args.min_approach_vel,
        approach_vel_scaling_dist=cli_args.approach_vel_scaling_dist,
        use_velocity_scaling=cli_args.use_velocity_scaling,
        min_radius_for_regulation=cli_args.min_radius_for_regulation,
        min_regulated_speed=cli_args.min_regulated_speed,
        goal_tolerance=cli_args.goal_tolerance,
        control_frequency=cli_args.control_frequency,
        max_angular_vel=cli_args.max_angular_vel
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  
    finally:
        node.get_logger().info('Shutting down controller...')
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
