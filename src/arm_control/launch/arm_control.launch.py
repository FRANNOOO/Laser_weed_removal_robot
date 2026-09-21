from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Automatically configure middleware and domain ID to match the simulation
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
        description='Duration for trajectory execution in seconds'
    )
    auto_move_arg = DeclareLaunchArgument(
        'auto_move',
        default_value='false',
        description='Whether to automatically trigger motion to target on start'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock time'
    )

    arm_controller_node = Node(
        package='arm_control',
        executable='arm_controller',
        name='arm_controller_node',
        output='screen',
        parameters=[{
            'target_x': LaunchConfiguration('target_x'),
            'target_y': LaunchConfiguration('target_y'),
            'target_z': LaunchConfiguration('target_z'),
            'duration_sec': LaunchConfiguration('duration_sec'),
            'auto_move': LaunchConfiguration('auto_move'),
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
        auto_move_arg,
        use_sim_time_arg,
        arm_controller_node,
    ])
