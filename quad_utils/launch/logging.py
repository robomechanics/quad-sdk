from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from datetime import datetime
import os

# Offboard RealSense D456 (on the remote computer). Absolute namespace so the
# topics don't pick up the robot namespace planning.py pushes around this file.
CAMERA_NS = '/camera'
CAMERA_NAME = 'd456'
CAMERA_PREFIX = f"{CAMERA_NS}/{CAMERA_NAME}"


def launch_camera(context, *args, **kwargs):
    if LaunchConfiguration('camera').perform(context) != 'true':
        return []
    try:
        get_package_share_directory('realsense2_camera')
    except PackageNotFoundError:
        return [LogInfo(msg="[logging] camera:=true but realsense2_camera is not installed "
                            "(sudo apt install ros-jazzy-realsense2-camera); recording without it")]

    return [
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace=CAMERA_NS,
            name=CAMERA_NAME,
            parameters=[{
                'enable_color': True,
                'enable_depth': False,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'rgb_camera.color_profile': LaunchConfiguration('camera_profile').perform(context),
                # Stamp frames on this computer's clock (same clock as mocap
                # and the bag), not the camera's internal hardware clock.
                'global_time_enabled': True,
            }],
        )
    ]


def camera_topics(context):
    if LaunchConfiguration('camera').perform(context) != 'true':
        return []
    topics = [
        f"{CAMERA_PREFIX}/color/image_raw/compressed",
        f"{CAMERA_PREFIX}/color/camera_info",
    ]
    if LaunchConfiguration('camera_raw').perform(context) == 'true':
        topics.append(f"{CAMERA_PREFIX}/color/image_raw")
    return topics


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
        f"{topic_prefix}/state/applied_joint_torques",
        f"{topic_prefix}/state/foot_net_forces",
        *[f"{topic_prefix}/gazebo/toe{i}_contact_states" for i in range(4)],
        f"{topic_prefix}/policy/observations",
        f"{topic_prefix}/policy/raw_actions",
        f"{topic_prefix}/cmd_vel_stamped",
        f"{topic_prefix}/cmd_vel",  # raw teleop; the only command record in stock-controller runs
        "/terrain_map",
        "/clock",
        *camera_topics(context)
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
        f"{topic_prefix}/state/applied_joint_torques",
        f"{topic_prefix}/state/foot_net_forces",
        *[f"{topic_prefix}/gazebo/toe{i}_contact_states" for i in range(4)],
        f"{topic_prefix}/policy/observations",
        f"{topic_prefix}/policy/raw_actions",
        f"{topic_prefix}/cmd_vel_stamped",
        f"{topic_prefix}/cmd_vel",  # raw teleop; the only command record in stock-controller runs
        "/terrain_map",
        "/clock",
        *camera_topics(context)
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
        DeclareLaunchArgument('camera', default_value='true',
                              description='Start the RealSense D456 and bag its color stream'),
        DeclareLaunchArgument('camera_profile', default_value='640,480,30',
                              description='Color stream width,height,fps'),
        DeclareLaunchArgument('camera_raw', default_value='false',
                              description='Also bag uncompressed color (~28 MB/s at 640x480x30)'),
        OpaqueFunction(function=launch_camera),
        OpaqueFunction(function=launch_bag_recording),
    ])