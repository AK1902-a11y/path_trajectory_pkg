#!/usr/bin/env python3
"""
Launch file for Implementation 2: Clothoid Path Smoothing + Trajectory Generation
Executes clothoid path smoother and trajectory generator sequentially.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, LogInfo, TimerAction
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    """Generate launch description for sequential execution."""
    
    package_name = 'path_trajectory_pkg'
    
    # Node 1: Clothoid Path Smoother
    clothoid_smoother = Node(
        package=package_name,
        executable='path_smoother_clothoids',
        name='clothoid_path_smoother',
        output='screen',
        emulate_tty=True,
        arguments=[
            '--samples', '50',
            # '--no-save'  # Uncomment to disable saving parameters
        ]
    )
    
    # Node 2: Trajectory Generator
    trajectory_generator = Node(
        package=package_name,
        executable='trajectory_generator',
        name='trajectory_generator',
        output='screen',
        emulate_tty=True,
        arguments=[
            '--v-max', '0.8',
            '--a-max', '0.45',
            '--a-lat-max', '0.7',
            # '--no-smooth'  # Uncomment to disable acceleration smoothing
        ]
    )
    
    # Event handler: Sequential execution
    trajectory_gen_after_smoother = RegisterEventHandler(
        OnProcessExit(
            target_action=clothoid_smoother,
            on_exit=[
                LogInfo(msg='✓ Clothoid path smoothing completed.'),
                LogInfo(msg='  Starting trajectory generation...'),
                TimerAction(
                    period=1.0,  # 1 second delay for file write
                    actions=[trajectory_generator]
                )
            ]
        )
    )
    
    # Success message
    planning_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=trajectory_generator,
            on_exit=[
                LogInfo(msg='Generated Files:'),
                LogInfo(msg='  ✓ config/smooth_path.yaml'),
                LogInfo(msg='  ✓ config/trajectory.yaml'),
            ]
        )
    )
    
    return LaunchDescription([
        clothoid_smoother,
        trajectory_gen_after_smoother,
        planning_complete,
    ])
