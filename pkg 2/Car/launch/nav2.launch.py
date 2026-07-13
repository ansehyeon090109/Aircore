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
    # 시뮬레이션에서는 true, 실제 로봇(Jetson Nano 등)에서 돌릴 때는
    # false로 넘겨야 함 (실기는 /clock을 발행하지 않으므로 true로 두면
    # 시간이 흐르지 않아 전체 bringup이 멈춘 것처럼 보임).
    # ex) ros2 launch aircore_description nav2.launch.py use_sim_time:=false
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='true=Gazebo 시뮬레이션, false=실제 로봇')

    # amcl/map_server 없이 slam_toolbox가 만든 /map, map->odom tf를 그대로 씀.
    # controller_server, planner_server, smoother_server, behavior_server,
    # bt_navigator, waypoint_follower, velocity_smoother + costmap 2개가 뜸.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        params_arg,
        sim_time_arg,
        navigation,
    ])