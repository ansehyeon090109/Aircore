"""base_controller + odometry (base.launch.py) + RViz odometry view.

  ros2 launch aircore_base odom_view.launch.py                 # real_params.yaml + rviz
  ros2 launch aircore_base odom_view.launch.py rviz:=false     # no rviz
  ros2 launch aircore_base odom_view.launch.py params_file:=/path/to/params.yaml

At startup the params file is checked and any unmeasured (0) key is reported as a warning.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Keys that must be > 0. /odom needs all odometry.* keys; base_controller copies are for consistency.
REQUIRED_KEYS = {
    'odometry': ('wheel_radius', 'wheel_separation', 'ticks_per_rev_left', 'ticks_per_rev_right'),
    'base_controller': ('wheel_radius', 'wheel_separation'),
}
FLOAT_KEYS = ('wheel_radius', 'wheel_separation')
SHARED_KEYS = ('wheel_radius', 'wheel_separation')


def _section(data: dict, node: str) -> dict:
    return ((data.get(node) or {}).get('ros__parameters') or {})


def _check_params(context, *args, **kwargs):
    logger = get_logger('odom_view')
    path = LaunchConfiguration('params_file').perform(context)
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as e:
        logger.warning(f'params_file 을 읽을 수 없어 미측정 키 검사를 건너뜀: {path} ({e})')
        return []

    unset, type_errors = [], []
    for node, keys in REQUIRED_KEYS.items():
        params = _section(data, node)
        for key in keys:
            value = params.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
                unset.append(f'{node}.{key}')
            elif key in FLOAT_KEYS and not isinstance(value, float):
                type_errors.append(f'{node}.{key}={value} (실수로 표기 필요, 예: {value}.0)')
            elif key not in FLOAT_KEYS and not isinstance(value, int):
                type_errors.append(f'{node}.{key}={value} (정수여야 함)')

    logger.info(f'params_file: {path}')
    if unset:
        odom_unset = [k for k in unset if k.startswith('odometry.')]
        effect = ('/odom, TF(odom->base_link) 는 publish 되지 않음 (enc_raw/enc_zeroed 는 정상)'
                  if odom_unset else 'odometry 섹션은 채워져 있으므로 /odom 은 publish 됨')
        logger.warning(f'미측정(0) 키: {", ".join(unset)} → {effect}. '
                       'scripts/apply_measurements.py 로 채우세요.')
    if type_errors:
        logger.warning('파라미터 타입 오류 (노드가 시작되지 않을 수 있음): ' + ', '.join(type_errors))

    bc, od = _section(data, 'base_controller'), _section(data, 'odometry')
    for key in SHARED_KEYS:
        if bc.get(key, 0) and od.get(key, 0) and bc.get(key) != od.get(key):
            logger.warning(f'{key} 가 두 섹션에서 다름: base_controller={bc.get(key)} '
                           f'odometry={od.get(key)}')
    if not unset and not type_errors:
        logger.info('실측값 모두 설정됨 → /odom, TF publish 예정')
    return []


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('aircore_base')
    default_params = os.path.join(share, 'config', 'real_params.yaml')
    rviz_config = os.path.join(share, 'rviz', 'odom.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='base_controller / odometry 파라미터 yaml'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='true면 rviz2 (rviz/odom.rviz) 도 실행'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true면 시리얼에 쓰지 않고 로그만 (base.launch.py 로 전달)'),
        OpaqueFunction(function=_check_params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'base.launch.py')),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'dry_run': LaunchConfiguration('dry_run'),
            }.items(),
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
