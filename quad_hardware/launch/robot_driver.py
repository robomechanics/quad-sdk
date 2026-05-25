"""Unified on-robot bring-up for the ros2_control quad-sdk stack.

Single hardware entry point replacing the per-robot go2_bringup /
go2_estimator / *_hardware_readonly launches. It:

  * processes the robot URDF with the HARDWARE backend (use_sim:=false) for
    robot_description,
  * starts robot_state_publisher,
  * starts controller_manager (ros2_control_node) with the composed config
    (controllers.yaml + <robot>.yaml + hardware.yaml),
  * spawns joint_state_broadcaster + imu_sensor_broadcaster,
  * unless read_only, also spawns the five switchable mode controllers
    (sit active, the rest inactive), the mode_supervisor (FSM that switches
    controllers per /control/mode), and the state estimator.

Everything is pushed under `namespace` (default robot_1), so the
controller_manager lives at /<namespace>/controller_manager.

    # safe read-only bench test (broadcasters only, no torque):
    ros2 launch quad_hardware robot_driver.py robot_type:=go2 read_only:=true

    # full control bring-up:
    ros2 launch quad_hardware robot_driver.py robot_type:=go2
    ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 1" --once  # READY

Choose the READY-mode law via locomotion_controller's `controller` param in
quad_utils/config/<robot>.yaml. Estimator type is the `estimator` arg
(comp_filter | ekf).
"""

import os

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


# Maps robot_type -> (description package, urdf dir name, urdf file).
# Mirrors robot_bringup.py's resolution. go2 also accepts a network_interface
# xacro arg (Unitree DDS link); other robots' xacros ignore it, so it is only
# passed for go2.
_ROBOTS = {
    "spirit": ("spirit_description", "spirit", "spirit.urdf.xacro"),
    "a1": ("a1_description", "a1", "a1.urdf.xacro"),
    "go1": ("go1_description", "go1", "go1.urdf.xacro"),
    "go2": ("go2_description", "go2", "go2.urdf.xacro"),
    "go2w": ("go2w_description", "go2w", "go2w.urdf.xacro"),
    "b2": ("b2_description", "b2", "b2.urdf.xacro"),
    "spot": ("spot_description", "spot", "spot.urdf.xacro"),
    "vision60": ("vision60_description", "vision60", "vision60.urdf.xacro"),
}


def _spawner(controller, namespace, active=True):
    args = [controller,
            "--controller-manager", f"/{namespace}/controller_manager",
            "--controller-manager-timeout", "60"]
    if not active:
        args.append("--inactive")
    return Node(package="controller_manager", executable="spawner",
                output="screen", arguments=args)


def setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    read_only = LaunchConfiguration("read_only").perform(context)
    estimator = LaunchConfiguration("estimator").perform(context)
    net_iface = LaunchConfiguration("network_interface").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")

    if robot_type not in _ROBOTS:
        raise RuntimeError(f"[robot_driver] Unsupported robot type: {robot_type}")
    desc_pkg, urdf_dir, urdf_file = _ROBOTS[robot_type]

    desc_path = FindPackageShare(desc_pkg).perform(context)
    urdf_path = os.path.join(desc_path, "models", urdf_dir, "urdf", urdf_file)
    # Hardware backend: use_sim:=false. go2's xacro also takes network_interface.
    mappings = {"use_sim": "false", "read_only": read_only}
    if robot_type == "go2":
        mappings["network_interface"] = net_iface
    urdf = xacro.process_file(urdf_path, mappings=mappings).toxml()

    # Effective config = NODE file (controllers.yaml, robot-/context-agnostic)
    # + ROBOT file (quad_utils/config/<robot>.yaml: gains/poses/joints/
    # kinematics) + HARDWARE context overlay (hardware.yaml: interface_mode
    # motor, is_hardware true). ros2_control merges param files in order, later
    # overriding earlier.
    quad_controllers_config = os.path.join(
        FindPackageShare("quad_controllers").perform(context), "config")
    controllers_yaml = os.path.join(quad_controllers_config, "controllers.yaml")
    hardware_yaml = os.path.join(quad_controllers_config, "hardware.yaml")
    robot_yaml = os.path.join(
        FindPackageShare("quad_utils").perform(context), "config",
        f"{robot_type}.yaml")

    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf, "use_sim_time": use_sim_time}])

    controller_manager = Node(
        package="controller_manager", executable="ros2_control_node",
        output="screen",
        parameters=[controllers_yaml, robot_yaml, hardware_yaml,
                    {"use_sim_time": use_sim_time}])

    nodes = [robot_state_publisher, controller_manager]

    broadcasters = [_spawner("joint_state_broadcaster", namespace),
                    _spawner("imu_sensor_broadcaster", namespace)]

    if read_only == "true":
        # Broadcasters only: no controller claims a command interface, so no
        # torque is ever commanded. This is the safe bench test.
        nodes.append(TimerAction(period=2.0, actions=broadcasters))
        return [GroupAction([PushRosNamespace(namespace)] + nodes)]

    # Full control: sit_controller ACTIVE so the robot holds the sit pose from
    # spawn (matches robot_driver's control_mode_ = SIT init); the rest load
    # inactive and the supervisor switches them per mode.
    mode_controllers = [
        _spawner("sit_controller", namespace, active=True),
        _spawner("locomotion_controller", namespace, active=False),
        _spawner("safety_controller", namespace, active=False),
        _spawner("sit_to_ready_controller", namespace, active=False),
        _spawner("ready_to_sit_controller", namespace, active=False),
    ]

    # mode_supervisor reads its controller-name mappings + timings from the NODE
    # file and its is_hardware flag from the hardware overlay; the explicit
    # controller_manager service name targets this robot's namespace.
    supervisor = Node(
        package="quad_supervisor", executable="mode_supervisor",
        name="mode_supervisor", output="screen",
        parameters=[controllers_yaml, hardware_yaml,
                    {"controller_manager": f"/{namespace}/controller_manager",
                     "use_sim_time": use_sim_time}])

    # State estimator (quad_estimators): exe selected by the `estimator` arg.
    # Fed estimator tuning (estimator.yaml) + robot kinematics (<robot>.yaml,
    # for QuadKD2's leg/frame params) + robot_description; imu remapped to the
    # imu_sensor_broadcaster output, as go2_estimator.launch.py did.
    estimator_exe = ("ekf_estimator_node" if estimator == "ekf"
                     else "comp_filter_estimator_node")
    estimator_yaml = os.path.join(
        FindPackageShare("quad_estimators").perform(context), "config",
        "estimator.yaml")
    estimator_node = Node(
        package="quad_estimators", executable=estimator_exe, output="screen",
        parameters=[estimator_yaml, robot_yaml,
                    {"robot_description": urdf, "use_sim_time": use_sim_time}],
        remappings=[("imu", "/imu_sensor_broadcaster/imu")])

    nodes.append(TimerAction(period=2.0,
                             actions=broadcasters + mode_controllers))
    nodes.append(TimerAction(period=4.0, actions=[supervisor, estimator_node]))

    return [GroupAction([PushRosNamespace(namespace)] + nodes)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_type", default_value="go2",
            description="Robot type (go2 | spirit | a1 | go1 | go2w | b2 | "
                        "spot | vision60)."),
        DeclareLaunchArgument(
            "namespace", default_value="robot_1",
            description="Robot namespace; controller_manager lives at "
                        "/<namespace>/controller_manager."),
        DeclareLaunchArgument(
            "read_only", default_value="false",
            description="If true, broadcasters only -- no torque is ever "
                        "commanded (safe bring-up)."),
        DeclareLaunchArgument(
            "estimator", default_value="comp_filter",
            description="State estimator type (comp_filter | ekf)."),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use the simulation clock (false on hardware)."),
        DeclareLaunchArgument(
            "network_interface", default_value="eth0",
            description="Robot interface used by the Unitree DDS link (go2)."),
        OpaqueFunction(function=setup),
    ])
