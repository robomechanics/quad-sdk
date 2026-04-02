from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, RegisterEventHandler
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
import launch
import lifecycle_msgs.msg


def generate_launch_description():

    namespace = DeclareLaunchArgument('namespace', default_value='mocap_node')
    config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mocap4r2_optitrack_driver'),
            'config', 'mocap4r2_optitrack_driver_params.yaml'
        ])
    )

    mocap_node = LifecycleNode(
        name='mocap4r2_optitrack_driver_node',
        namespace=LaunchConfiguration('namespace'),
        package='mocap4r2_optitrack_driver',
        executable='mocap4r2_optitrack_driver_main',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    mocap_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(mocap_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    mocap_activate_event = RegisterEventHandler(
        OnProcessStart(
            target_action=mocap_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=launch.events.matchers.matches_action(mocap_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                ))
            ]
        )
    )

    return LaunchDescription([
        namespace,
        config_file,
        mocap_node,
        mocap_configure_event,
        mocap_activate_event,
    ])
