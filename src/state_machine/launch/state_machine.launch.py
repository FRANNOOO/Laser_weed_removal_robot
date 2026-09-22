"""Launch file for the weed removal state machine and navigation coordinator."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for state_machine and nav_integration nodes."""
    default_params_file = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'config',
        'params.yaml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS 2 parameters YAML file'
    )

    launch_nav_integration_arg = DeclareLaunchArgument(
        'launch_nav_integration',
        default_value='true',
        description='Whether to launch the Nav2 navigation coordinator node'
    )

    state_machine_node = Node(
        package='state_machine',
        executable='state_machine_node',
        name='state_machine_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    nav_integration_node = Node(
        package='state_machine',
        executable='nav_integration_node',
        name='nav_integration_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_nav_integration')),
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_file_arg,
        launch_nav_integration_arg,
        state_machine_node,
        nav_integration_node,
    ])
