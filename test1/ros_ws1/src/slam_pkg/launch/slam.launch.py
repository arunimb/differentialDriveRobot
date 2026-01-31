#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Get package directories
    slam_pkg_dir = get_package_share_directory('slam_pkg')
    diffrobot_pkg_dir = get_package_share_directory('diffRobot')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                slam_pkg_dir, 'config', 'slam_config.yaml'
            ]),
            description='Path to SLAM parameters file'
        ),

        
        # Odom TF Broadcaster Node
        Node(
            package='slam_pkg',
            executable='odom_tf_broadcaster',
            name='odom_tf_broadcaster',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        

        
        # SLAM Toolbox Node - This is the core SLAM component
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('/scan', '/scan'),  # Must match your bridged lidar topic
                ('/map', '/map'),
                ('/odom', '/odometry/filtered'),  # CRITICAL: Use fused odometry!
            ]
        ),
    ])