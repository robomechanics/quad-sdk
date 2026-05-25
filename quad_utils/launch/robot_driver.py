"""The quad-sdk control stack (ros2_control).

This is the shared core for BOTH worlds:

  * Hardware (default, use_sim:=false): the single file you run on the robot.
    Builds the URDF with the hardware backend, starts robot_state_publisher +
    controller_manager (ros2_control_node) + the state estimator, and spawns the
    broadcasters + switchable mode controllers + mode_supervisor. Pushes its own
    namespace.

  * Simulation (use_sim:=true): included by robot_bringup.py AFTER it has spawned
    the Gazebo model (whose gz_ros2_control plugin IS the controller_manager) and
    robot_state_publisher. Here we only spawn the broadcasters + mode controllers
    + mode_supervisor against that running CM. No namespace push (quad_gazebo
    already pushed it); no ros2_control_node / RSP / estimator (Gazebo provides
    them).

Hardware usage:
    ros2 launch quad_utils robot_driver.py robot_type:=go2                 # full
    ros2 launch quad_utils robot_driver.py robot_type:=go2 read_only:=true # safe bench
    ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 1" --once   # READY

The READY-mode leg-control law comes from <robot>.yaml's `controller:` param;
`controller:=<law>` overrides it. Estimator type is the `estimator` arg.
"""

import os
import tempfile

import xacro
import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, TimerAction,
                            ExecuteProcess, RegisterEventHandler, GroupAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


# robot_type -> (description package, urdf dir, urdf file). Mirrors robot_bringup.
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

# Valid READY-mode laws the `controller` arg may name (empty = use <robot>.yaml).
_VALID_LAWS = {"inverse_dynamics", "grf_pid", "joint", "underbrush",
               "inertia_estimation", "learned"}


def _config_dir(context, pkg):
    return os.path.join(FindPackageShare(pkg).perform(context), "config")


def _law_override_file(context):
    """A temp --param-file overriding locomotion_controller's law from the
    `controller` arg, or None when empty/not a valid law (then <robot>.yaml's
    `controller:` stands)."""
    controller_id = LaunchConfiguration("controller").perform(context)
    if controller_id not in _VALID_LAWS:
        return None
    override = {"/**/locomotion_controller":
               {"ros__parameters": {"controller": controller_id}}}
    fd, path = tempfile.mkstemp(prefix="quad_law_override_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(override, f, default_flow_style=False)
    return path


def _supervisor_node(context, namespace, context_yaml):
    """mode_supervisor: controller-name mappings + timings from controllers.yaml,
    is_hardware from the context overlay; absolute CM service name."""
    qc = _config_dir(context, "quad_controllers")
    return Node(
        package="quad_supervisor", executable="mode_supervisor",
        name="mode_supervisor", output="screen",
        parameters=[os.path.join(qc, "controllers.yaml"), context_yaml,
                    {"controller_manager": f"/{namespace}/controller_manager",
                     "use_sim_time": LaunchConfiguration("use_sim_time")}])


def _setup_sim(context):
    """Spawn the control stack against the already-running gz CM. Reproduces
    robot_bringup's former inline spawning exactly. No namespace push (quad_gazebo
    pushed it); merged config is handed in via `sim_config_file`."""
    namespace = LaunchConfiguration("namespace").perform(context)
    merged_config = LaunchConfiguration("sim_config_file").perform(context)

    common = ["--controller-manager", f"/{namespace}/controller_manager",
              "--controller-manager-timeout", "120", "--switch-timeout", "180",
              "--param-file", merged_config]
    law_override = _law_override_file(context)
    if law_override:
        common += ["--param-file", law_override]

    jsb = ExecuteProcess(cmd=[
        "ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster",
        "--controller-manager", f"/{namespace}/controller_manager",
        "--controller-manager-timeout", "120", "--switch-timeout", "180"])
    # sit ACTIVE so the robot holds sit from spawn; the rest inactive until the
    # supervisor switches them.
    spawn_sit = ExecuteProcess(cmd=[
        "ros2", "run", "controller_manager", "spawner", "sit_controller", *common])
    spawn_inactive = ExecuteProcess(cmd=[
        "ros2", "run", "controller_manager", "spawner",
        "locomotion_controller", "safety_controller",
        "sit_to_ready_controller", "ready_to_sit_controller", "--inactive",
        *common])
    chain_after_jsb = RegisterEventHandler(OnProcessExit(
        target_action=jsb, on_exit=[spawn_sit, spawn_inactive]))

    return [
        TimerAction(period=0.5, actions=[chain_after_jsb, jsb]),
        _supervisor_node(context, namespace,
                         os.path.join(_config_dir(context, "quad_controllers"),
                                      "sim.yaml")),
    ]


def _spawner_node(controller, namespace, active=True):
    args = [controller, "--controller-manager",
            f"/{namespace}/controller_manager",
            "--controller-manager-timeout", "60"]
    if not active:
        args.append("--inactive")
    return Node(package="controller_manager", executable="spawner",
                output="screen", arguments=args)


def _setup_hardware(context):
    """Full on-robot bring-up: build the hardware URDF, start RSP + CM +
    estimator, spawn broadcasters + controllers + supervisor. Pushes its own
    namespace (this launch is the top-level entry on the robot)."""
    robot_type = LaunchConfiguration("robot_type").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    read_only = LaunchConfiguration("read_only").perform(context)
    estimator = LaunchConfiguration("estimator").perform(context)
    net_iface = LaunchConfiguration("network_interface").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")

    if robot_type not in _ROBOTS:
        raise RuntimeError(f"[robot_driver] Unsupported robot type: {robot_type}")
    desc_pkg, urdf_dir, urdf_file = _ROBOTS[robot_type]
    urdf_path = os.path.join(FindPackageShare(desc_pkg).perform(context),
                             "models", urdf_dir, "urdf", urdf_file)
    mappings = {"use_sim": "false", "read_only": read_only}
    if robot_type == "go2":
        mappings["network_interface"] = net_iface
    urdf = xacro.process_file(urdf_path, mappings=mappings).toxml()

    qc = _config_dir(context, "quad_controllers")
    robot_yaml = os.path.join(_config_dir(context, "quad_utils"),
                              f"{robot_type}.yaml")
    cfg = [os.path.join(qc, "controllers.yaml"), robot_yaml,
           os.path.join(qc, "hardware.yaml")]

    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf, "use_sim_time": use_sim_time}])
    controller_manager = Node(
        package="controller_manager", executable="ros2_control_node",
        output="screen", parameters=[*cfg, {"use_sim_time": use_sim_time}])

    nodes = [robot_state_publisher, controller_manager]
    broadcasters = [_spawner_node("joint_state_broadcaster", namespace),
                    _spawner_node("imu_sensor_broadcaster", namespace)]

    if read_only == "true":
        # Broadcasters only -> no command interface claimed -> no torque.
        nodes.append(TimerAction(period=2.0, actions=broadcasters))
        return [GroupAction([PushRosNamespace(namespace)] + nodes)]

    mode_controllers = [
        _spawner_node("sit_controller", namespace, active=True),
        _spawner_node("locomotion_controller", namespace, active=False),
        _spawner_node("safety_controller", namespace, active=False),
        _spawner_node("sit_to_ready_controller", namespace, active=False),
        _spawner_node("ready_to_sit_controller", namespace, active=False),
    ]
    estimator_exe = ("ekf_estimator_node" if estimator == "ekf"
                     else "comp_filter_estimator_node")
    estimator_node = Node(
        package="quad_estimators", executable=estimator_exe, output="screen",
        parameters=[os.path.join(_config_dir(context, "quad_estimators"),
                                 "estimator.yaml"), robot_yaml,
                    {"robot_description": urdf, "use_sim_time": use_sim_time}],
        remappings=[("imu", "/imu_sensor_broadcaster/imu")])

    nodes.append(TimerAction(period=2.0,
                             actions=broadcasters + mode_controllers))
    nodes.append(TimerAction(
        period=4.0,
        actions=[_supervisor_node(context, namespace,
                                  os.path.join(qc, "hardware.yaml")),
                 estimator_node]))
    return [GroupAction([PushRosNamespace(namespace)] + nodes)]


def setup(context, *args, **kwargs):
    if LaunchConfiguration("use_sim").perform(context) == "true":
        return _setup_sim(context)
    return _setup_hardware(context)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_type", default_value="go2",
            description="go2 | spirit | a1 | go1 | go2w | b2 | spot | vision60"),
        DeclareLaunchArgument("namespace", default_value="robot_1",
            description="CM lives at /<namespace>/controller_manager."),
        DeclareLaunchArgument("use_sim", default_value="false",
            description="true = spawn against Gazebo's CM (set by robot_bringup); "
                        "false = hardware (start ros2_control_node + estimator)."),
        DeclareLaunchArgument("read_only", default_value="false",
            description="Hardware only: broadcasters only, no torque (safe bring-up)."),
        DeclareLaunchArgument("estimator", default_value="comp_filter",
            description="State estimator (comp_filter | ekf); hardware only."),
        DeclareLaunchArgument("controller", default_value="",
            description="READY-mode law override; empty = use <robot>.yaml."),
        DeclareLaunchArgument("network_interface", default_value="eth0",
            description="NIC for the Unitree DDS link (go2 hardware)."),
        DeclareLaunchArgument("use_sim_time", default_value="false",
            description="Use the simulation clock."),
        DeclareLaunchArgument("sim_config_file", default_value="",
            description="(sim) merged controller config passed by robot_bringup."),
        OpaqueFunction(function=setup),
    ])
