"""Standalone launch file for the Nav2 navigation coordinator node."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for nav_integration_node."""
    domain_id_arg = DeclareLaunchArgument(
        'domain_id',
        default_value=os.environ.get('ROS_DOMAIN_ID', '1'),
        description='ROS domain ID (1 for simulation, 2 for robot)'
    )
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id'))

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock time'
    )
    mock_nav2_arg = DeclareLaunchArgument(
        'mock_nav2',
        default_value='false',
        description='Simulate Nav2 pause/resume services when Nav2 is not active'
    )
    tracked_weeds_topic_arg = DeclareLaunchArgument(
        'tracked_weeds_topic',
        default_value='/tracked_weeds',
        description='Topic publishing 3D weed coordinates'
    )
    robot_stopped_topic_arg = DeclareLaunchArgument(
        'robot_stopped_topic',
        default_value='/robot_stopped',
        description='Topic publishing robot stop status (Bool)'
    )
    workspace_min_x_arg = DeclareLaunchArgument(
        'workspace_min_x',
        default_value='0.290',
        description='Back edge X coordinate of arm workspace in robot_base_link (m)'
    )
    back_edge_margin_x_arg = DeclareLaunchArgument(
        'back_edge_margin_x',
        default_value='0.020',
        description='Safety margin before back edge (m)'
    )
    min_resume_distance_arg = DeclareLaunchArgument(
        'min_resume_distance_m',
        default_value='0.30',
        description='Distance robot must travel after resume before stopping again (m)'
    )

    nav_integration_node = Node(
        package='state_machine',
        executable='nav_integration_node',
        name='nav_integration_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'mock_nav2': LaunchConfiguration('mock_nav2'),
            'tracked_weeds_topic': LaunchConfiguration('tracked_weeds_topic'),
            'robot_stopped_topic': LaunchConfiguration('robot_stopped_topic'),
            'workspace_min_x': LaunchConfiguration('workspace_min_x'),
            'back_edge_margin_x': LaunchConfiguration('back_edge_margin_x'),
            'min_resume_distance_m': LaunchConfiguration('min_resume_distance_m'),
        }]
    )

    return LaunchDescription([
        domain_id_arg,
        set_rmw,
        set_domain_id,
        use_sim_time_arg,
        mock_nav2_arg,
        tracked_weeds_topic_arg,
        robot_stopped_topic_arg,
        workspace_min_x_arg,
        back_edge_margin_x_arg,
        min_resume_distance_arg,
        nav_integration_node,
    ])
