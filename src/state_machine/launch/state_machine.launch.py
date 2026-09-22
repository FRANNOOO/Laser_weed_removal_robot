"""Launch file for the weed removal state machine and navigation coordinator."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for state_machine and nav_integration nodes."""
    
    # state_machine_node arguments
    duration_sec_arg = DeclareLaunchArgument(
        'duration_sec',
        default_value='2.0',
        description='Duration for arm trajectory execution in seconds'
    )
    laser_duration_us_arg = DeclareLaunchArgument(
        'laser_duration_us',
        default_value='500000',
        description='Duration for laser activation in microseconds'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock time'
    )
    approx_weed_topic_arg = DeclareLaunchArgument(
        'approx_weed_topic',
        default_value='/tracked_weeds',
        description='Topic publishing 3D weed coordinates from first camera / YOLO pipeline'
    )
    robot_stopped_topic_arg = DeclareLaunchArgument(
        'robot_stopped_topic',
        default_value='/robot_stopped',
        description='Topic publishing robot stop status (Bool) to gate weeding'
    )
    auto_start_weeding_arg = DeclareLaunchArgument(
        'auto_start_weeding',
        default_value='false',
        description='If true, treat robot as stopped and begin weeding immediately upon detection'
    )
    simulate_precise_camera_arg = DeclareLaunchArgument(
        'simulate_precise_camera',
        default_value='true',
        description='Simulate second camera detection with slight offset in simulation'
    )

    # nav_integration_node arguments
    launch_nav_integration_arg = DeclareLaunchArgument(
        'launch_nav_integration',
        default_value='true',
        description='Whether to launch the Nav2 navigation coordinator node'
    )
    mock_nav2_arg = DeclareLaunchArgument(
        'mock_nav2',
        default_value='false',
        description='Simulate Nav2 pause/resume services when Nav2 is not active'
    )
    workspace_min_x_arg = DeclareLaunchArgument(
        'workspace_min_x',
        default_value='0.290',
        description='Front edge X coordinate of arm workspace in robot_base_link (m)'
    )
    workspace_max_x_arg = DeclareLaunchArgument(
        'workspace_max_x',
        default_value='0.400',
        description='Back edge X coordinate of arm workspace in robot_base_link (m)'
    )
    workspace_min_y_arg = DeclareLaunchArgument(
        'workspace_min_y',
        default_value='-0.070',
        description='Left/negative Y boundary of arm workspace in robot_base_link (m)'
    )
    workspace_max_y_arg = DeclareLaunchArgument(
        'workspace_max_y',
        default_value='0.070',
        description='Right/positive Y boundary of arm workspace in robot_base_link (m)'
    )
    dedup_radius_arg = DeclareLaunchArgument(
        'dedup_radius',
        default_value='0.03',
        description='Spatial deduplication radius (m) for tracking weeds'
    )
    back_edge_margin_x_arg = DeclareLaunchArgument(
        'back_edge_margin_x',
        default_value='0.040',
        description='Safety margin before back edge (m)'
    )
    stop_delay_sec_arg = DeclareLaunchArgument(
        'stop_delay_sec',
        default_value='0.80',
        description='Estimated stopping delay including communication and deceleration (s)'
    )
    min_resume_distance_arg = DeclareLaunchArgument(
        'min_resume_distance_m',
        default_value='0.05',
        description='Distance robot must travel after resume before stopping again (m)'
    )
    provide_start_stop_service_arg = DeclareLaunchArgument(
        'provide_start_stop_service',
        default_value='false',
        description='Whether to host the /start_stop_robot service'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic for robot pose and velocity'
    )

    state_machine_node = Node(
        package='state_machine',
        executable='state_machine_node',
        name='state_machine_node',
        output='screen',
        parameters=[{
            'duration_sec': LaunchConfiguration('duration_sec'),
            'laser_duration_us': LaunchConfiguration('laser_duration_us'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'approx_weed_topic': LaunchConfiguration('approx_weed_topic'),
            'robot_stopped_topic': LaunchConfiguration('robot_stopped_topic'),
            'auto_start_weeding': LaunchConfiguration('auto_start_weeding'),
            'simulate_precise_camera': LaunchConfiguration('simulate_precise_camera'),
            'workspace_min_x': LaunchConfiguration('workspace_min_x'),
            'workspace_max_x': LaunchConfiguration('workspace_max_x'),
            'workspace_min_y': LaunchConfiguration('workspace_min_y'),
            'workspace_max_y': LaunchConfiguration('workspace_max_y'),
            'dedup_radius': LaunchConfiguration('dedup_radius'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }]
    )

    nav_integration_node = Node(
        package='state_machine',
        executable='nav_integration_node',
        name='nav_integration_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_nav_integration')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'mock_nav2': LaunchConfiguration('mock_nav2'),
            'approx_weed_topic': LaunchConfiguration('approx_weed_topic'),
            'tracked_weeds_topic': LaunchConfiguration('approx_weed_topic'),
            'robot_stopped_topic': LaunchConfiguration('robot_stopped_topic'),
            'workspace_min_x': LaunchConfiguration('workspace_min_x'),
            'workspace_max_x': LaunchConfiguration('workspace_max_x'),
            'workspace_min_y': LaunchConfiguration('workspace_min_y'),
            'workspace_max_y': LaunchConfiguration('workspace_max_y'),
            'dedup_radius': LaunchConfiguration('dedup_radius'),
            'back_edge_margin_x': LaunchConfiguration('back_edge_margin_x'),
            'stop_delay_sec': LaunchConfiguration('stop_delay_sec'),
            'min_resume_distance_m': LaunchConfiguration('min_resume_distance_m'),
            'provide_start_stop_service': LaunchConfiguration('provide_start_stop_service'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }]
    )

    return LaunchDescription([
        duration_sec_arg,
        laser_duration_us_arg,
        use_sim_time_arg,
        approx_weed_topic_arg,
        robot_stopped_topic_arg,
        auto_start_weeding_arg,
        simulate_precise_camera_arg,
        launch_nav_integration_arg,
        mock_nav2_arg,
        workspace_min_x_arg,
        workspace_max_x_arg,
        workspace_min_y_arg,
        workspace_max_y_arg,
        dedup_radius_arg,
        back_edge_margin_x_arg,
        stop_delay_sec_arg,
        min_resume_distance_arg,
        provide_start_stop_service_arg,
        odom_topic_arg,
        state_machine_node,
        nav_integration_node,
    ])
