import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory('aircore_base'), 'config', 'base_params.yaml')

    params_file = LaunchConfiguration('params_file')
    dry_run = LaunchConfiguration('dry_run')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='base_controller / odometry 파라미터 yaml'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true면 시리얼에 쓰지 않고 로그만'),
        Node(
            package='aircore_base',
            executable='base_controller',
            name='base_controller',
            output='screen',
            emulate_tty=True,
            # yaml 로드 후 dry_run을 launch 인자로 덮어씀
            parameters=[params_file, {'dry_run': ParameterValue(dry_run, value_type=bool)}],
        ),
        Node(
            package='aircore_base',
            executable='odometry',
            name='odometry',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),
    ])
