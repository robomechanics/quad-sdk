from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from datetime import datetime
import os

def launch_bag_recording(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    bag_name = LaunchConfiguration('bag_name').perform(context)

    quad_logger_src = os.environ.get('QUAD_LOGGER_SRC')
    if quad_logger_src is None:
        raise RuntimeError("QUAD_LOGGER_SRC env variable not set")

    timestamp = datetime.now().strftime('%Y%m%d_%H%M')
    full_name = f"{namespace}_{bag_name}_{robot_type}_{timestamp}"

    topic_prefix = f"/{namespace}"
    topic_list_1 = [
        f"{topic_prefix}/state",
        f"{topic_prefix}/state/imu",
        f"{topic_prefix}/state/trajectory",
        f"{topic_prefix}/state/ground_truth",
        f"{topic_prefix}/state/ground_truth_body_frame",
        f"{topic_prefix}/state/estimate",
        f"{topic_prefix}/state/grfs",
        f"{topic_prefix}/mocap_node/quad/pose",
        f"{topic_prefix}/global_plan",
        f"{topic_prefix}/local_plan",
        f"{topic_prefix}/control/grfs",
        f"{topic_prefix}/control/joint_command",
        f"{topic_prefix}/control/leg_command",
        f"{topic_prefix}/control/mode",
        f"{topic_prefix}/foot_plan_continuous",
        f"{topic_prefix}/foot_plan_discrete",
        f"{topic_prefix}/body_force/joint_torques",
        f"{topic_prefix}/body_force/toe_forces",
        f"{topic_prefix}/state/foot_contact",
        f"{topic_prefix}/cmd_vel_stamped",
        "/terrain_map",
        "/clock"
    ]

    topic_list_2 = [
        f"{topic_prefix}/state/joints",
        f"{topic_prefix}/state/imu",
        f"{topic_prefix}/state/trajectory",
        f"{topic_prefix}/state/ground_truth",
        f"{topic_prefix}/state/ground_truth_body_frame",
        f"{topic_prefix}/state/estimate",
        f"{topic_prefix}/state/grfs",
        f"{topic_prefix}/mocap_node/quad/pose",
        f"{topic_prefix}/global_plan",
        f"{topic_prefix}/local_plan",
        f"{topic_prefix}/control/grfs",
        f"{topic_prefix}/control/joint_command",
        f"{topic_prefix}/control/leg_command",
        f"{topic_prefix}/control/mode",
        f"{topic_prefix}/foot_plan_continuous",
        f"{topic_prefix}/foot_plan_discrete",
        f"{topic_prefix}/body_force/joint_torques",
        f"{topic_prefix}/body_force/toe_forces",
        f"{topic_prefix}/state/foot_contact",
        f"{topic_prefix}/cmd_vel_stamped",
        "/terrain_map",
        "/clock"
    ]

    return [
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o',
                f"{quad_logger_src}/bags/{full_name}",
                '--include-hidden-topics',
                *topic_list_1
            ],
            shell=False
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o',
                f"{quad_logger_src}/bags/archive/{full_name}",
                '--include-hidden-topics',
                *topic_list_2
            ],
            shell=False
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('bag_name', default_value='quad_log'),
        DeclareLaunchArgument('robot_type', default_value='go2'),
        OpaqueFunction(function=launch_bag_recording),
    ])