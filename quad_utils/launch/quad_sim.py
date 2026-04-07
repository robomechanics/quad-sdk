from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

import os
import json
# ---------------------------------------------------
# --------------- Gazebo launch files ---------------
# ---------------------------------------------------

def launch_ignition_world(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    verbose = LaunchConfiguration('verbose').perform(context).lower() == 'true'

    pkg_share = FindPackageShare('gazebo_scripts').perform(context)
    world_path = os.path.join(pkg_share, 'worlds', f"{world_name}")  
    model_path = os.path.join(pkg_share, 'models')

    # Build the command for `ign gazebo`
    cmd = ['gz', 'sim', world_path, '-r']
    if not gui:
        cmd.append('-s')
    if verbose:
        cmd.extend(['-v', '4'])  # Set verbosity level explicitly if requested

    return [
        GroupAction([
            PushRosNamespace('remote'),
            ExecuteProcess(
                cmd=cmd,
                output='log',
                additional_env={'GZ_SIM_RESOURCE_PATH': (EnvironmentVariable('GZ_SIM_RESOURCE_PATH')),
                                'GZ_SIM_SYSTEM_PLUGIN_PATH': (EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH'))}
            )
        ])
    ]

def bridge_global_clock(context, *args, **kwargs):
    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            namespace='',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            # output='screen'
        )
    ]



# ---------------------------------------------------
# --------------- Mujoco launch files ---------------
# ---------------------------------------------------

def launch_mujoco_world(context, *args, **kwargs):
    """
    MuJoCo doesn't have a separate 'world' process in the same way —
    the world mesh is loaded by each robot's mujoco_ros2_control node.
    This function is the hook for anything truly global at sim startup,
    e.g. a shared clock publisher or a headless viewer node.
    Right now it just publishes /clock so use_sim_time works identically
    to the Gazebo path.
    """
    mujoco_scripts_path = FindPackageShare('mujoco_sim').perform(context)

    # If you add a global MuJoCo viewer node, launch it here.
    # For now, just a sim-time clock publisher.
    clock_node = Node(
        package='mujoco_sim',
        executable='mujoco_clock_publisher',
        name='mujoco_clock',
        namespace='',
        output='log',
    )
    return [clock_node]


# ---------------------------------------------------
# ------------------ Shared files -------------------
# ---------------------------------------------------

def launch_obstacles(context, *args, **kwargs):
    # Need to add mesh spawning for mujoco obstacles
    sim_backend = LaunchConfiguration('sim_backend').perform(context)
    if sim_backend == 'mujoco':
        return []

    obstacle_launch_path = PathJoinSubstitution([
        FindPackageShare('quad_utils'), 'launch', 'spawn_obstacles.py'
    ])
    return [
        GroupAction([
            PushRosNamespace('remote'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(obstacle_launch_path),
                launch_arguments={
                    'scenario': LaunchConfiguration('scenario'),
                    'obstacles': LaunchConfiguration('obstacles'),
                }.items()
            )
        ])
    ]


def launch_robot_mapping(context, *args, **kwargs):
    mapping_launch_path = PathJoinSubstitution([
        FindPackageShare('quad_utils'), 'launch', 'mapping.py'
    ])
    return [
        GroupAction([
            PushRosNamespace('mapping'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch_path),
                launch_arguments={
                    'input_type': 'mesh',
                    'world': LaunchConfiguration('world'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()
            )
        ])
    ]


def launch_robot_group(context, *args, **kwargs):
    """
    Same as quad_gazebo.py, but now passes sim_backend down to robot_bringup.py
    so each robot knows which hardware plugin to load.
    """
    robot_configs_raw = LaunchConfiguration('robot_configs').perform(context)
    sim_backend       = LaunchConfiguration('sim_backend').perform(context)

    try:
        robot_configs = json.loads(robot_configs_raw)
    except json.JSONDecodeError as e:
        raise RuntimeError(f"Invalid JSON in 'robot_configs': {e}")

    robot_launch_file = PathJoinSubstitution([
        FindPackageShare('quad_utils'), 'launch', 'robot_bringup.py'
    ])

    groups = []
    for config in robot_configs:
        group = GroupAction([
            PushRosNamespace(config['name']),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch_file),
                launch_arguments={
                    'robot_type':   TextSubstitution(text=config['type']),
                    'namespace':    TextSubstitution(text=config['name']),
                    'controller':   TextSubstitution(text=config['controller']),
                    'init_pose':    TextSubstitution(text=config['init_pose']),
                    'world':        LaunchConfiguration('world'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'sim_backend':  TextSubstitution(text=sim_backend),  # <-- new
                }.items()
            )
        ])
        groups.append(group)

    return groups


def launch_visualization(context, *args, **kwargs):
    visualization_launch_path = PathJoinSubstitution([
        FindPackageShare('quad_utils'), 'launch', 'quad_visualization.py'
    ])
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visualization_launch_path),
            launch_arguments={
                'live_plot':    LaunchConfiguration('live_plot'),
                'dash':         LaunchConfiguration('dash'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        )
    ]


# ---------------------------------------------------
# ---------- Router; Picks Gazebo/MuJoCO ------------
# ---------------------------------------------------
def launch_sim_backend(context, *args, **kwargs):
    """
    Single OpaqueFunction that reads sim_backend and fires the right
    world-level launch functions. This is the only place the branch lives.
    """
    sim_backend = LaunchConfiguration('sim_backend').perform(context)

    if sim_backend == 'gazebo':
        gz_actions = launch_ignition_world(context)
        clock_actions = bridge_global_clock(context)
        return gz_actions + clock_actions

    elif sim_backend == 'mujoco':
        return launch_mujoco_world(context)

    else:
        raise RuntimeError(f"[quad_sim] Unknown sim_backend: '{sim_backend}'. Use 'gazebo' or 'mujoco'.")

def generate_launch_description():
    return LaunchDescription([
        # ── world / sim args ──────────────────────────────────────
        DeclareLaunchArgument('sim_backend', default_value='gazebo',
                              description="Simulator backend: 'gazebo' or 'mujoco'"),
        DeclareLaunchArgument('world',       default_value='flat.sdf',
                              description='World file name (SDF for Gazebo, mesh name for MuJoCo)'),
        DeclareLaunchArgument('gui',         default_value='true',
                              description='Launch simulator GUI (Gazebo only)'),
        DeclareLaunchArgument('verbose',     default_value='false',
                              description='Verbose simulator output'),

        # ── robot args ────────────────────────────────────────────
        DeclareLaunchArgument(
            'robot_configs',
            default_value='[{"name":"robot_1","type":"go2","controller":"inverse_dynamics","init_pose":"-x 0.0 -y 0.0 -z 5"}]',
            description='JSON list of robot configs'
        ),
        DeclareLaunchArgument('scenario',  default_value='None'),
        DeclareLaunchArgument('obstacles', default_value='[]'),

        # ── shared args ───────────────────────────────────────────
        DeclareLaunchArgument('live_plot',    default_value='false'),
        DeclareLaunchArgument('dash',         default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('logging',      default_value='false'),

        # ── launch sequence ───────────────────────────────────────
        OpaqueFunction(function=launch_sim_backend),    # world — branched
        OpaqueFunction(function=launch_obstacles),      # obstacles — branched internally
        OpaqueFunction(function=launch_robot_mapping),  # same either way
        OpaqueFunction(function=launch_robot_group),    # same either way, passes sim_backend down
        OpaqueFunction(function=launch_visualization),  # same either way
    ])