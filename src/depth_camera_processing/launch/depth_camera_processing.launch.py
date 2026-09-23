import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('depth_camera_processing')
    default_params_file = os.path.join(pkg_share, 'config', 'depth_camera_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to ROS 2 parameters file for depth_camera_node',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    mock_yolo_arg = DeclareLaunchArgument(
        'mock_yolo',
        default_value='false',
        description='Run with simulated YOLO detections for offline testing',
    )

    depth_node = Node(
        package='depth_camera_processing',
        executable='depth_camera_node',
        name='depth_camera_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'mock_yolo': LaunchConfiguration('mock_yolo')},
        ],
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        mock_yolo_arg,
        depth_node,
    ])
