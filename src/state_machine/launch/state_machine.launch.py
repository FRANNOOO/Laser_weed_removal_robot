"""Launch file for the weed removal state machine node."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for state_machine_node."""
    domain_id_arg = DeclareLaunchArgument(
        'domain_id',
        default_value=os.environ.get('ROS_DOMAIN_ID', '1'),
        description='ROS domain ID (1 for simulation, 2 for robot)'
    )
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id'))

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
        }]
    )

    return LaunchDescription([
        domain_id_arg,
        set_rmw,
        set_domain_id,
        duration_sec_arg,
        laser_duration_us_arg,
        use_sim_time_arg,
        approx_weed_topic_arg,
        robot_stopped_topic_arg,
        auto_start_weeding_arg,
        state_machine_node,
    ])
