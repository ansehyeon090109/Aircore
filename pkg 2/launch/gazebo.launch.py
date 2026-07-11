import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('aircore_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_model_path = os.path.join(pkg_share, 'urdf', 'aircore.xacro')
    default_world_path = os.path.join(pkg_share, 'worlds', 'aircore_world.sdf')
    default_bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')

    model_arg = DeclareLaunchArgument(
        'model', default_value=default_model_path,
        description='xacro/urdf 파일 경로')
    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world_path,
        description='gz-sim 월드(sdf) 파일 경로')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [LaunchConfiguration('world'), ' -r']}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'aircore',
            '-z', '0.05',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': default_bridge_config,
            'use_sim_time': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])