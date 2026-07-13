import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('aircore_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'aircore.xacro')
    default_rviz_path = os.path.join(pkg_share, 'config', 'aircore.rviz')

    model_arg = DeclareLaunchArgument(
        'model', default_value=default_model_path,
        description='xacro/urdf 파일 경로')
    rviz_arg = DeclareLaunchArgument(
        'rvizconfig', default_value=default_rviz_path,
        description='rviz2 설정 파일 경로')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])