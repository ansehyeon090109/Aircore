"""Real-robot bringup: base + odometry + URDF TF + RPLIDAR C1 + RViz.

  ros2 launch aircore_base bringup.launch.py                  # everything + rviz
  ros2 launch aircore_base bringup.launch.py rviz:=false
  ros2 launch aircore_base bringup.launch.py lidar:=false     # without the LiDAR
  ros2 launch aircore_base bringup.launch.py base:=false      # LiDAR + URDF TF only (no STM32)

TF tree:
  odom -> base_footprint                  (odometry, base_frame in params_file)
  base_footprint -> base_link -> laser .. (robot_state_publisher, aircore.urdf)
  wheel / lidar_head joints               (joint_state_publisher, fixed at 0)
"""
import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

URDF_ROOT = 'base_footprint'


def _check_bringup_params(context, *args, **kwargs):
    logger = get_logger('bringup')
    lidar_on = LaunchConfiguration('lidar').perform(context).lower() in ('true', '1')
    if lidar_on:
        # Fail here, before any process starts: a missing package later in the launch aborts it
        # and leaves already-started nodes (incl. base_controller on the serial port) orphaned.
        try:
            get_package_share_directory('sllidar_ros2')
        except PackageNotFoundError:
            raise RuntimeError(
                "sllidar_ros2 패키지를 찾을 수 없음 → 노드를 시작하지 않고 중단. "
                "이 터미널에서 'source ~/aircore_ws/install/local_setup.bash' 를 다시 실행하거나 "
                "lidar:=false 로 실행하세요.")

    path = LaunchConfiguration('params_file').perform(context)
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as e:
        logger.warning(f'params_file 을 읽을 수 없음: {path} ({e})')
        return []

    odom = (data.get('odometry') or {}).get('ros__parameters') or {}
    base_frame = odom.get('base_frame', 'base_link')
    if base_frame != URDF_ROOT:
        logger.warning(f'odometry.base_frame={base_frame} 이지만 URDF 루트는 {URDF_ROOT} → '
                       f'TF 트리가 끊기거나 {base_frame} 의 부모가 둘이 됨. base_frame 을 {URDF_ROOT} 로.')

    if lidar_on:
        lidar =(data.get('sllidar_node') or {}).get('ros__parameters') or {}
        lidar_port = lidar.get('serial_port')
        base_port = ((data.get('base_controller') or {}).get('ros__parameters') or {}).get('serial_port')
        if not lidar_port:
            logger.warning('sllidar_node.serial_port 가 params_file 에 없음 (드라이버 기본값 /dev/ttyUSB0 사용됨)')
        elif lidar_port == base_port:
            logger.error(f'LiDAR 와 base_controller 가 같은 포트를 씀: {lidar_port}')
        elif not os.path.exists(lidar_port):
            logger.warning(f'LiDAR 포트 없음: {lidar_port} (USB 연결 확인)')
    return []


def generate_launch_description() -> LaunchDescription:
    base_share = get_package_share_directory('aircore_base')
    desc_share = get_package_share_directory('aircore_description')
    default_params = os.path.join(base_share, 'config', 'real_params.yaml')
    rviz_config = os.path.join(base_share, 'rviz', 'bringup.rviz')
    with open(os.path.join(desc_share, 'urdf', 'aircore.urdf'), encoding='utf-8') as f:
        robot_description = f.read()

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='base_controller / odometry / sllidar_node 파라미터 yaml'),
        DeclareLaunchArgument('rviz', default_value='true', description='rviz2 실행 여부'),
        DeclareLaunchArgument('lidar', default_value='true', description='RPLIDAR 드라이버 실행 여부'),
        DeclareLaunchArgument('base', default_value='true',
                              description='base_controller + odometry 실행 여부 (false: LiDAR/TF만 확인)'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true면 base_controller 가 시리얼에 쓰지 않음'),
        OpaqueFunction(function=_check_bringup_params),
        # base_controller + odometry (+ unmeasured-key check), without its own rviz.
        # Scoped group: in Humble, launch_arguments of an include leak into this file's scope
        # (rviz:='false' would otherwise turn off the rviz node below).
        GroupAction(
            scoped=True,
            condition=IfCondition(LaunchConfiguration('base')),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(base_share, 'launch', 'odom_view.launch.py')),
                launch_arguments={
                    'params_file': params_file,
                    'dry_run': LaunchConfiguration('dry_run'),
                    'rviz': 'false',
                }.items(),
            )],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            # continuous joints (wheels, lidar head) at 0 so their TFs exist
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[params_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('lidar')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
