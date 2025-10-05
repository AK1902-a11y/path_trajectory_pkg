#!/usr/bin/env python3
"""
Launch file for Implementation 1: B-Spline Path Smoothing + Trajectory Generation
Executes path smoother and trajectory generator sequentially.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo, TimerAction
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for sequential execution."""
    
    # Package name
    package_name = 'path_trajectory_pkg'
    
    # Node 1: Path Smoother (B-Spline)
    path_smoother = Node(
        package=package_name,
        executable='path_smoother',
        name='path_smoother',
        output='screen',
        emulate_tty=True
    )
    
    # Node 2: Trajectory Generator
    # This will start automatically after path_smoother completes
    trajectory_generator = Node(
        package=package_name,
        executable='trajectory_generator',
        name='trajectory_generator',
        output='screen',
        emulate_tty=True,
    )
    
    # Event handler: Wait for path_smoother to complete, then start trajectory_generator
    trajectory_gen_after_smoother = RegisterEventHandler(
        OnProcessExit(
            target_action=path_smoother,
            on_exit=[
                LogInfo(msg='Path smoothing completed. Starting trajectory generation...'),
                TimerAction(
                    period=1.0,  # 1 second delay to ensure file is written
                    actions=[trajectory_generator]
                )
            ]
        )
    )
    
    # Success message after trajectory generation
    planning_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=trajectory_generator,
            on_exit=[
                LogInfo(msg='✓ Planning pipeline completed successfully!'),
                LogInfo(msg='  Generated files:'),
                LogInfo(msg='    - config/smooth_path.yaml'),
                LogInfo(msg='    - config/trajectory.yaml'),
                LogInfo(msg='  Ready to run controller.')
            ]
        )
    )
    
    return LaunchDescription([
        LogInfo(msg='Starting Implementation 1 Planning Pipeline...'),
        LogInfo(msg='  Stage 1: B-Spline Path Smoothing'),
        LogInfo(msg='  Stage 2: Trajectory Generation'),
        path_smoother,
        trajectory_gen_after_smoother,
        planning_complete,
    ])
