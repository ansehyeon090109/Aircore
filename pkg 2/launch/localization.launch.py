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
    default_map_path = os.path.join(pkg_share, 'maps', 'aircore_map.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params_path,
        description='Nav2 파라미터 yaml 경로 (amcl 섹션 포함)')
    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map_path,
        description='map_saver_cli로 저장한 map yaml 경로')

    # slam_toolbox 대신 저장된 지도를 map_server가 발행하고, amcl이 /scan,
    # /odom, tf(odom->base_link)를 이용해 map->odom tf와 위치를 추정함.
    # slam.launch.py와 동시에 켜면 map->odom을 두 노드가 동시에 발행하게 되므로
    # 반드시 둘 중 하나만 실행할 것.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        params_arg,
        map_arg,
        localization,
    ])
