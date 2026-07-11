from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    wander_node = Node(
        package='aircore_description',
        executable='wander.py',
        name='wander',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([wander_node])