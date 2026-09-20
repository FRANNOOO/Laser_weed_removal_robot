#!/usr/bin/env python3
"""
Launch file for weed_fsm_node. Parameters are loaded from a YAML config
file rather than individual launch arguments.

Usage:
    ros2 launch <your_package> weed_fsm_launch.py
    ros2 launch <your_package> weed_fsm_launch.py params_file:=/path/to/other_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('state_machine'),  
        'config',
        'weed_fsm_params.yaml',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the YAML file with weed_fsm_node parameters',
    )

    weed_fsm_node = Node(
        package='state_machine',        
        executable='weed_fsm_node',          
        name='weed_fsm_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        weed_fsm_node,
    ])