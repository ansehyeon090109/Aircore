"""Gazebo Fortress (ign gazebo 6): smart-farm map (+x floor extension) + 4 perimeter walls + aircore vehicle.

The world is regenerated at launch time from the STL / URDF with the launch arguments below,
into $ROS_HOME/aircore_sim/generated (default ~/.ros/aircore_sim/generated).
Default values live in aircore_sim/farm_geometry.py (map) and aircore_sim/vehicle.py (spawn) only.

Vehicle (gz topics, no ROS bridge yet):
  drive : ign topic -t /model/aircore/cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0.1}'
  odom  : /model/aircore/odometry     lidar: /scan
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration

from aircore_sim import farm_geometry as fg
from aircore_sim import vehicle as vh


def _flag(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in ('1', 'true', 'yes')


def _setup(context):
    log = get_logger('sim_world')
    scale = float(LaunchConfiguration('map_scale').perform(context))
    floor_w = float(LaunchConfiguration('floor_width_x').perform(context))
    wall_h = float(LaunchConfiguration('wall_height').perform(context))
    wall_t = float(LaunchConfiguration('wall_thickness').perform(context))
    headless = _flag(context, 'headless')
    with_vehicle = _flag(context, 'with_vehicle')
    spawn = [float(LaunchConfiguration(k).perform(context)) for k in ('spawn_x', 'spawn_y', 'spawn_yaw')]

    if shutil.which('ign') is None:
        log.error('`ign` 명령 없음: Gazebo Fortress 미설치. '
                  'sudo apt install ignition-fortress ros-humble-ros-gz  (Harmonic `gz sim`으로 대체하지 않음)')
        return [Shutdown(reason='Gazebo Fortress not installed')]

    log.warning(f'벽 높이/두께는 임시값입니다: wall_height={wall_h} m, wall_thickness={wall_t} m '
                f'(실측 후 aircore_sim/farm_geometry.py 기본값 수정)')

    share = get_package_share_directory('aircore_sim')
    stl = os.path.join(share, 'models', fg.MODEL_NAME, 'meshes', fg.STL_NAME)
    geom = fg.load_farm_geometry(stl, scale, floor_w)
    for w in fg.validate(geom, wall_h, wall_t):
        log.warning(w)
    for line in fg.describe(geom):
        log.info(line)

    ros_home = os.environ.get('ROS_HOME', os.path.join(os.path.expanduser('~'), '.ros'))
    out_root = os.path.join(ros_home, 'aircore_sim', 'generated')
    resource = [os.path.join(out_root, 'models')]
    includes = []
    if with_vehicle:
        desc_share = get_package_share_directory('aircore_description')
        car = vh.build_vehicle(os.path.join(desc_share, 'urdf', 'aircore.urdf'))
        vh.write_vehicle_model(resource[0], car)
        resource.append(os.path.dirname(desc_share))          # model://aircore_description/meshes/...
        x, y, yaw = spawn
        walls = fg.wall_boxes(geom.plate, wall_h, wall_t)
        for w in vh.check_spawn(geom.plate, list(geom.blocks) + walls, x, y, car.footprint_radius):
            log.warning(w)
        includes.append((vh.MODEL_NAME, (x, y, 0.0, 0.0, 0.0, yaw)))
        log.info(f'vehicle {vh.MODEL_NAME}: spawn x={x} y={y} yaw={yaw}, '
                 f'footprint r={car.footprint_radius:.3f} m')
        log.warning(f'시뮬 DiffDrive wheel_separation={car.wheel_separation:.4g} m, '
                    f'wheel_radius={car.wheel_radius:.4g} m 는 URDF(CAD) 값입니다 (실측값 아님)')

    files = fg.write_world_files(out_root, geom, wall_h, wall_t, stl,
                                 includes=includes, sensors=with_vehicle)
    log.info(f'world: {files.world_sdf}')

    if os.environ.get('IGN_GAZEBO_RESOURCE_PATH'):
        resource.append(os.environ['IGN_GAZEBO_RESOURCE_PATH'])
    cmd = ['ign', 'gazebo', '-r'] + (['-s'] if headless else []) + [files.world_sdf]
    return [ExecuteProcess(cmd=cmd, additional_env={'IGN_GAZEBO_RESOURCE_PATH': os.pathsep.join(resource)},
                           output='screen')]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('map_scale', default_value=str(fg.MAP_SCALE),
                              description='STL 단위 -> m (원본 mm)'),
        DeclareLaunchArgument('floor_width_x', default_value=str(fg.FLOOR_WIDTH_X),
                              description='[m] 전체 바닥 x폭 (STL 바닥판보다 넓으면 +x쪽으로 확장)'),
        DeclareLaunchArgument('wall_height', default_value=str(fg.WALL_HEIGHT),
                              description='[m] 벽 높이 (임시값)'),
        DeclareLaunchArgument('wall_thickness', default_value=str(fg.WALL_THICKNESS),
                              description='[m] 벽 두께 (임시값)'),
        DeclareLaunchArgument('with_vehicle', default_value='true',
                              description='aircore 차량 모델 포함'),
        DeclareLaunchArgument('spawn_x', default_value=str(vh.SPAWN_X), description='[m] 차량 시작 x'),
        DeclareLaunchArgument('spawn_y', default_value=str(vh.SPAWN_Y), description='[m] 차량 시작 y'),
        DeclareLaunchArgument('spawn_yaw', default_value=str(vh.SPAWN_YAW), description='[rad] 차량 시작 방향'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true면 GUI 없이 서버만 (ign gazebo -s)'),
        OpaqueFunction(function=_setup),
    ])
