"""Launch file for the weed removal state machine node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for state_machine_node."""
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '0')

    target_x_arg = DeclareLaunchArgument(
        'target_x',
        default_value='0.35',
        description='Target X position in meters (robot_base_link frame)'
    )
    target_y_arg = DeclareLaunchArgument(
        'target_y',
        default_value='0.0',
        description='Target Y position in meters (robot_base_link frame)'
    )
    target_z_arg = DeclareLaunchArgument(
        'target_z',
        default_value='-0.10',
        description='Target Z position in meters (robot_base_link frame)'
    )
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
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Whether to automatically trigger weed removal on node start'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock time'
    )

    state_machine_node = Node(
        package='state_machine',
        executable='state_machine_node',
        name='state_machine_node',
        output='screen',
        parameters=[{
            'target_x': LaunchConfiguration('target_x'),
            'target_y': LaunchConfiguration('target_y'),
            'target_z': LaunchConfiguration('target_z'),
            'duration_sec': LaunchConfiguration('duration_sec'),
            'laser_duration_us': LaunchConfiguration('laser_duration_us'),
            'auto_start': LaunchConfiguration('auto_start'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        set_rmw,
        set_domain_id,
        target_x_arg,
        target_y_arg,
        target_z_arg,
        duration_sec_arg,
        laser_duration_us_arg,
        auto_start_arg,
        use_sim_time_arg,
        state_machine_node,
    ])
