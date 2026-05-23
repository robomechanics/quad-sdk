from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from datetime import datetime
import os


def launch_bag_recording(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    bag_name = LaunchConfiguration('bag_name').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    full_name = f"{namespace}_{bag_name}_{robot_type}_{timestamp}"
    bag_path = os.path.join(output_dir, full_name)

    tp = f"/{namespace}"

    # Time-synchronized topics (from time_sync_node)
    synced_topics = [
        f"{tp}/synced/state/ground_truth",
        f"{tp}/synced/control/joint_command",
        f"{tp}/synced/control/grfs",
        f"{tp}/synced/local_plan",
        f"{tp}/synced/state/estimate",
    ]

    # Raw topics (full-rate, for reference / richer per-iteration logs)
    raw_topics = [
        f"{tp}/state/ground_truth",
        f"{tp}/state/estimate",
        f"{tp}/state/imu",
        f"{tp}/state/joints",
        f"{tp}/state/trajectory",
        f"{tp}/control/joint_command",
        f"{tp}/control/grfs",
        f"{tp}/control/mode",
        f"{tp}/local_plan",
        f"{tp}/global_plan",
        f"{tp}/foot_plan_continuous",
        f"{tp}/foot_plan_discrete",
        f"{tp}/body_force/joint_torques",
        f"{tp}/body_force/toe_forces",
        f"{tp}/cmd_vel_stamped",
        "/terrain_map",
    ]

    return [
        # Bag 1: synced topics only (clean time-aligned bundles)
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', f"{bag_path}_synced",
                '--include-hidden-topics',
                *synced_topics,
            ],
            shell=False,
        ),
        # Bag 2: all raw topics (full-rate reference)
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', f"{bag_path}_raw",
                '--include-hidden-topics',
                *raw_topics,
            ],
            shell=False,
        ),
    ]


def generate_launch_description():
    default_output = os.path.join(os.getcwd(), 'iteration_bags')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('robot_type', default_value='go2'),
        DeclareLaunchArgument('bag_name', default_value='iteration'),
        DeclareLaunchArgument('output_dir', default_value=default_output,
                              description='Directory to store bag files'),
        OpaqueFunction(function=launch_bag_recording),
    ])
