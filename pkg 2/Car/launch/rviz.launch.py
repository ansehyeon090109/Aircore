import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aircore_description')
    default_rviz_path = os.path.join(pkg_share, 'config', 'aircore.rviz')

    rvizconfig_arg = DeclareLaunchArgument(
        'rvizconfig', default_value=default_rviz_path,
        description='rviz2 설정 파일 경로')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([rvizconfig_arg, rviz])