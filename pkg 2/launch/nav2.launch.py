import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('aircore_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    default_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params_path,
        description='Nav2 파라미터 yaml 경로')

    # amcl/map_server 없이 slam_toolbox가 만든 /map, map->odom tf를 그대로 씀.
    # controller_server, planner_server, smoother_server, behavior_server,
    # bt_navigator, waypoint_follower, velocity_smoother + costmap 2개가 뜸.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        params_arg,
        navigation,
    ])