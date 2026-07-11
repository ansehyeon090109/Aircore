import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_share = get_package_share_directory('aircore_description')
    default_params_path = os.path.join(
        pkg_share, 'config', 'mapper_params_online_async.yaml')

    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')

    params_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=default_params_path,
        description='slam_toolbox 파라미터 yaml 경로')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='slam_toolbox를 자동으로 configure/activate 시킬지 여부')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='별도 lifecycle manager(nav2 등)로 관리할지 여부')

    # async_slam_toolbox_node는 LifecycleNode라서, 그냥 Node로 띄우면
    # "unconfigured" 상태로 영원히 멈춰 있고 /scan 구독도 /map 발행도
    # 시작하지 않는다(실제로 이 버그를 겪음 - ros2 node info로 확인해보니
    # /clock, /parameter_events만 구독하고 있고 /scan 구독 자체가 없었음).
    # slam_toolbox 패키지가 공식 제공하는 online_async_launch.py를 그대로
    # 참고해서 configure -> activate 이벤트를 명시적으로 걸어줘야 함.
    slam_toolbox_node = LifecycleNode(
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'use_lifecycle_manager': use_lifecycle_manager,
                'use_sim_time': True,
            },
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[slam.launch.py] slam_toolbox configure 완료 -> activate 진행'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    return LaunchDescription([
        params_arg,
        declare_autostart_cmd,
        declare_use_lifecycle_manager,
        slam_toolbox_node,
        configure_event,
        activate_event,
    ])