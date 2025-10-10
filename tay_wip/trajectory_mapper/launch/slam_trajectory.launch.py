#!/usr/bin/env python3
"""
Launch file to run SLAM Toolbox with trajectory mapper
Provides drift-corrected trajectory recording for maze navigation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get SLAM Toolbox package directory
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Path to SLAM Toolbox launch file
    slam_launch_file = os.path.join(
        slam_toolbox_dir,
        'launch',
        'online_async_launch.py'
    )
    
    return LaunchDescription([
        # Launch SLAM Toolbox for localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={
                'use_sim_time': 'true',
            }.items()
        ),
        
        # Launch trajectory mapper (records SLAM-corrected path)
        Node(
            package='trajectory_mapper',
            executable='trajectory_mapper_node',
            name='trajectory_mapper',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),
        
        # Launch maze visualizer
        Node(
            package='trajectory_mapper',
            executable='maze_visualizer.py',
            name='maze_visualizer',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),
    ])