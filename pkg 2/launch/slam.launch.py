import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aircore_description')
    default_params_path = os.path.join(
        pkg_share, 'config', 'mapper_params_online_async.yaml')

    params_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=default_params_path,
        description='slam_toolbox 파라미터 yaml 경로')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        params_arg,
        slam_toolbox_node,
    ])