"""Per-robot CBS-debugging bag recorder.

This launch records the minimum needed to compute:

  - Reference vs. actual body trajectory (tracking error)
  - Reference yaw / yaw-rate continuity (validates unwrap chain)
  - Reference velocity profile (looks for plan-induced infeasibility)
  - NMPC joint-effort output (saturation symptoms)
  - GRF estimate divergence (body force estimator health)

Usage from multi_robot.py:
    ros2 launch quad_utils multi_robot.py logging_cbs:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from datetime import datetime
import os


def launch_bag_recording(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    bag_name = LaunchConfiguration('bag_name').perform(context)

    quad_logger_src = os.environ.get('QUAD_LOGGER_SRC')
    if quad_logger_src is None:
        raise RuntimeError(
            "QUAD_LOGGER_SRC env variable not set. Export it to point at the "
            "directory under which CBS-debug bags should be written, e.g. "
            "`export QUAD_LOGGER_SRC=$HOME/quad_logger`.")

    timestamp = datetime.now().strftime('%Y%m%d_%H%M')
    full_name = f"{namespace}_{bag_name}_{robot_type}_{timestamp}"
    out_dir = f"{quad_logger_src}/bags/cbs/{full_name}"

    topic_prefix = f"/{namespace}"
    # Focused topic list for the CBS diagnosis script.
    topic_list = [
        f"{topic_prefix}/global_plan",
        f"{topic_prefix}/local_plan",
        f"{topic_prefix}/state/ground_truth",
        f"{topic_prefix}/state/estimate",
        f"{topic_prefix}/state/joints",
        f"{topic_prefix}/state/grfs",
        f"{topic_prefix}/control/grfs",
        f"{topic_prefix}/control/joint_command",
        f"{topic_prefix}/control/mode",
    ]

    return [
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', out_dir,
                '--include-hidden-topics',
                *topic_list,
            ],
            shell=False,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('bag_name', default_value='cbs_diag'),
        DeclareLaunchArgument('robot_type', default_value='go2'),
        OpaqueFunction(function=launch_bag_recording),
    ])
