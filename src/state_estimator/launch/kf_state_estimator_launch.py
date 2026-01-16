#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='kf_state_estimator.yaml',
        description='Name of the config file to use'
    )
    
    # Get package share directory
    # Note: Replace 'your_package_name' with actual package name
    package_name = 'state_estimator'  # Change this to your package name
    
    # Path to config file
    config_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        LaunchConfiguration('config_file')
    ])
    
    # Create the node
    kf_state_estimator_node = Node(
        package=package_name,
        executable='kf_state_estimator',
        name='kf_state_estimator',
        output='screen',
        parameters=[config_file],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        kf_state_estimator_node,
    ])