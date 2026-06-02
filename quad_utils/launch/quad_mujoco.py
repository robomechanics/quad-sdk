"""
Main Ros2 launch file for Mujoco simulation with quadrupeds. 
"""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription, ExecuteProcess, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

import os
import json
import math
import shutil
import xacro
import xml.etree.ElementTree as ET
from datetime import datetime


def _parse_init_pose(s):
    """Parse "-x X -y Y -z Z [-R r -P p -Y yaw]" into a flag->float dict."""
    out = {}
    tokens = s.split()
    i = 0
    while i + 1 < len(tokens):
        flag = tokens[i]
        if flag in ('-x', '-y', '-z', '-R', '-P', '-Y'):
            try:
                out[flag] = float(tokens[i + 1])
            except ValueError:
                pass
            i += 2
        else:
            i += 1
    return out


def _rpy_to_quat_wxyz(roll, pitch, yaw):
    """ZYX intrinsic RPY -> MuJoCo (w, x, y, z) quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz


def _patch_mjcf_keyframe(src_mjcf_path, init_pose_str, keyframe_name='home', out_dir='/tmp'):
    """Write a copy of the per-robot MJCF whose `<key name=keyframe_name>`
    qpos has its first 3 (xyz) and, if RPY supplied, next 4 (wxyz quat)
    values overridden from `init_pose_str`. Returns the patched file path,
    or the original path if no override is needed / keyframe is missing.

    The MJCF is included by the world MJCF via `<include file=...>`, so
    patching it is what makes mujoco_ros2_control's `initial_keyframe`
    apply at the requested spawn pose instead of the model default."""
    pose = _parse_init_pose(init_pose_str) if init_pose_str else {}
    if not pose:
        return src_mjcf_path

    tree = ET.parse(src_mjcf_path)
    root = tree.getroot()
    key_el = None
    for k in root.findall('.//keyframe/key'):
        if k.get('name') == keyframe_name:
            key_el = k
            break
    if key_el is None:
        return src_mjcf_path

    vals = (key_el.get('qpos') or '').split()
    if len(vals) < 7:
        return src_mjcf_path

    if '-x' in pose: vals[0] = repr(pose['-x'])
    if '-y' in pose: vals[1] = repr(pose['-y'])
    if '-z' in pose: vals[2] = repr(pose['-z'])
    if any(f in pose for f in ('-R', '-P', '-Y')):
        qw, qx, qy, qz = _rpy_to_quat_wxyz(
            pose.get('-R', 0.0), pose.get('-P', 0.0), pose.get('-Y', 0.0))
        vals[3:7] = [repr(qw), repr(qx), repr(qy), repr(qz)]

    key_el.set('qpos', ' '.join(vals))

    out_path = os.path.join(out_dir, f'_quad_robot_mjcf_{os.path.basename(src_mjcf_path)}')
    tree.write(out_path)
    return out_path


def prepare_world(context, *args, **kwargs):
    """Resolve the `world` arg into a full MJCF path, xacro-processing if needed.

    If `<world>.xacro` exists alongside the world file, process it with
    robot-specific paths derived from the first robot in `robot_configs`,
    write to /tmp, and store the resulting absolute path in `world_path`.
    Otherwise, set `world_path` to the static install path."""
    world = LaunchConfiguration('world').perform(context)
    worlds_dir = os.path.join(
        FindPackageShare('quad_sim_scripts').perform(context), 'worlds'
    )
    static_path = os.path.join(worlds_dir, world)
    xacro_path = static_path + '.xacro'

    if not os.path.isfile(xacro_path):
        return [SetLaunchConfiguration('world_path', static_path)]

    robot_configs = json.loads(LaunchConfiguration('robot_configs').perform(context))
    if not robot_configs:
        raise RuntimeError("'robot_configs' must contain at least one robot")
    robot_type = robot_configs[0]['type']
    init_pose = robot_configs[0].get('init_pose', '')

    desc_share = FindPackageShare(f'{robot_type}_description').perform(context)
    mjcf_dir = os.path.join(desc_share, 'models', robot_type, f'{robot_type}_mjc')

    world_name = world.rsplit('.xml', 1)[0]
    sim_share = FindPackageShare('quad_sim_scripts').perform(context)
    meshes_dir = os.path.join(sim_share, 'models', world_name, 'meshes')
    terrain_mesh = os.path.join(meshes_dir, f'{world_name}.stl')

    # Hfield worlds (rough_*, slope_*) consume `terrain_heightmap` as an
    # absolute path to a PNG or MuJoCo binary .bin. Convention: the file
    # lives next to the STL as `<world>.{bin,png}` — prefer .bin (native
    # MuJoCo hfield, faster load, no rasterisation drift) and fall back
    # to .png. If neither exists pass the empty string; flat-style worlds
    # default the arg and ignore it, so this stays backward-compatible.
    terrain_heightmap = ''
    for ext in ('bin', 'png'):
        candidate = os.path.join(meshes_dir, f'{world_name}.{ext}')
        if os.path.isfile(candidate):
            terrain_heightmap = candidate
            break

    robot_mjcf_path = _patch_mjcf_keyframe(
        os.path.join(mjcf_dir, f'{robot_type}.xml'), init_pose)

    processed = xacro.process_file(xacro_path, mappings={
        'meshdir': os.path.join(mjcf_dir, 'assets'),
        'mjcf_path': robot_mjcf_path,
        'terrain_mesh': terrain_mesh,
        'terrain_heightmap': terrain_heightmap,
    }).toxml()

    out_path = os.path.join('/tmp', f'_quad_world_{robot_type}_{world}')
    with open(out_path, 'w') as f:
        f.write(processed)

    return [SetLaunchConfiguration('world_path', out_path)]

def launch_robot_mapping(context, *args, **kwargs):
    mapping_launch_path = PathJoinSubstitution([
        FindPackageShare('quad_utils'),
        'launch',
        'mujoco_mapping.py'
    ])
    return [
        GroupAction([
            PushRosNamespace('mapping'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch_path),
                launch_arguments={
                    'input_type': 'mjcf',
                    'world': LaunchConfiguration('world'),
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items()
            )
        ])
    ]

def launch_robot_group(context, *args, **kwargs):
    robot_configs_raw = LaunchConfiguration('robot_configs').perform(context)
    try:
        robot_configs = json.loads(robot_configs_raw)
    except json.JSONDecodeError as e:
        raise RuntimeError(f"Invalid JSON in 'robot_configs': {e}")

    robot_groups = []

    for config in robot_configs:
        robot_ns = config["name"]
        robot_type = config["type"]
        controller = config["controller"]
        init_pose = config["init_pose"]

        robot_launch_file = PathJoinSubstitution([
            FindPackageShare('quad_utils'),
            'launch',
            'quad_mujoco_bringup.py'
        ])

        group = GroupAction([
            PushRosNamespace(robot_ns),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch_file),
                launch_arguments={
                    'robot_type': TextSubstitution(text=robot_type),
                    'namespace': TextSubstitution(text=robot_ns),
                    'controller': TextSubstitution(text=controller),
                    'init_pose' : TextSubstitution(text=init_pose),
                    'world': LaunchConfiguration('world'),
                    'world_path': LaunchConfiguration('world_path'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'simulator': TextSubstitution(text='mujoco')  # NEW: Specify simulator type
                }.items()
            )
        ])
        robot_groups.append(group)

    return robot_groups

def launch_visualization(context, *args, **kwargs):
    visualization_launch_path = PathJoinSubstitution([
        FindPackageShare('quad_utils'),
        'launch',
        'quad_visualization.py'
    ])
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visualization_launch_path),
            launch_arguments={
                'live_plot' : LaunchConfiguration('live_plot'),
                'dash' : LaunchConfiguration('dash'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items(),
        )
    ]

def launch_plot_juggler(context, *args, **kwargs):
    live_plot = LaunchConfiguration('live_plot').perform(context).lower() == 'true'

    if not live_plot:
        return []

    return [
        ExecuteProcess(
            cmd=['plotjuggler'],
            # output='screen',
            shell=False
        )
    ]


def launch_mujoco_world(context, *args, **kwargs):
    robot_configs_raw = LaunchConfiguration('robot_configs').perform(context)
    try:
        robot_configs = json.loads(robot_configs_raw)
    except json.JSONDecodeError as e:
        raise RuntimeError(f"Invalid JSON in 'robot_configs': {e}")
    if not robot_configs:
        raise RuntimeError("'robot_configs' must contain at least one robot")
    first = robot_configs[0]
    robot_ns = first['name']
    robot_type = first['type']

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('quad_sim_scripts'),
        'config',
        'quad_control.yaml'
    ])
    robot_yaml = os.path.join(
        FindPackageShare('quad_utils').perform(context),
        'config',
        f'{robot_type}.yaml',
    )

    return [
        Node(
            package='mujoco_ros2_control',
            executable='ros2_control_node',
            name='controller_manager',
            namespace=robot_ns,
            output='screen',
            parameters=[
                {'use_sim_time': True},
                controllers_yaml,
                robot_yaml,
            ],
            remappings=[
                ('robot_description', f'/{robot_ns}/robot_description'),
            ]
        )
    ]

def _resolve_quad_logger_src(context):
    """Return the quad_logger SOURCE dir (not install/share).

    Recordings live in `<src>/quad_logger/logs/` so they survive
    `colcon build` (which wipes install/). Prefer the QUAD_LOGGER_SRC env
    var (same convention as logging.py / logging_cbs.py); fall back to
    walking up from the install share to find `<ws>/src/.../quad_logger`.
    """
    quad_logger_src = os.environ.get('QUAD_LOGGER_SRC')
    if quad_logger_src and os.path.isdir(quad_logger_src):
        return quad_logger_src  

    share = FindPackageShare('quad_logger').perform(context)
    # share = <ws>/install/quad_logger/share/quad_logger
    ws_root = os.path.normpath(os.path.join(share, '..', '..', '..', '..'))
    for root, dirs, _ in os.walk(os.path.join(ws_root, 'src')):
        if os.path.basename(root) == 'quad_logger' and 'package.xml' in os.listdir(root):
            return root
        # Don't descend into build artefacts / vendored deps.
        dirs[:] = [d for d in dirs if d not in ('build', 'install', 'log', '.git')]
    raise RuntimeError(
        "Could not locate quad_logger source dir. Set QUAD_LOGGER_SRC, "
        "e.g. `export QUAD_LOGGER_SRC=$HOME/ros2_ws/src/quad-sdk/quad_logger`."
    )


def launch_recording(context, *args, **kwargs):
    recording = LaunchConfiguration('recording').perform(context).lower() == 'true'
    if not recording:
        return []

    if shutil.which('ffmpeg') is None:
        raise RuntimeError(
            "recording=true but `ffmpeg` is not installed. "
            "Install it with `sudo apt install ffmpeg`."
        )

    log_dir = os.path.join(_resolve_quad_logger_src(context), 'logs')
    os.makedirs(log_dir, exist_ok=True)

    robot_configs = json.loads(LaunchConfiguration('robot_configs').perform(context))
    if not robot_configs:
        raise RuntimeError("'robot_configs' must contain at least one robot")
    first = robot_configs[0]
    robot_type = first['type']
    robot_ns = first['name']

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_file = os.path.join(log_dir, f"mujoco_{robot_type}_{timestamp}.mp4")

    world_path = LaunchConfiguration('world_path').perform(context)

    # Pull the per-robot joint map out of mujoco_profiles.py and pass it
    # to the C++ recorder as two parallel string arrays (the C++ node
    # doesn't import the Python profile). Same source of truth as the
    # rest of the MuJoCo URDF injection in mujoco_urdf_utils.py.
    import sys as _sys
    _sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from mujoco_profiles import get_profile  # noqa: E402
    profile = get_profile(robot_type)
    joint_map_ros = [ros for ros, _ in profile['joint_map']]
    joint_map_mjc = [mjc for _, mjc in profile['joint_map']]

    # Offscreen recorder: shadow-loads the same MJCF, mirrors live state
    # from /<ns>/odom and /<ns>/joint_states into its own mjData, and
    # encodes frames straight to mp4 via libmujoco (mujoco_vendor) +
    # GLFW. No screen capture, no display window, no race with the
    # viewer. The node closes its ffmpeg pipe in its destructor so
    # Ctrl+C produces a finalized mp4.
    return [
        Node(
            package='quad_utils',
            executable='mujoco_recorder',
            name='mujoco_recorder',
            output='screen',
            parameters=[{
                'mjcf_path': world_path,
                'namespace': robot_ns,
                'output_path': out_file,
                'width': 1280,
                'height': 720,
                'fps': 30,
                'camera_track_robot': True,
                'odom_free_joint_name': profile['odom_free_joint_name'],
                'joint_map_ros': joint_map_ros,
                'joint_map_mjc': joint_map_mjc,
                'use_sim_time': True,
            }],
        )
    ]




def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('world', default_value='flat.xml', description='MJCF world file name to load into simulation'),
        DeclareLaunchArgument('gui', default_value='true', description='Whether to launch the MuJoCo GUI'),
        DeclareLaunchArgument('paused', default_value='false', description='Whether to start the simulation in a paused state'),
        DeclareLaunchArgument('verbose', default_value='false', description='Launch the simulator in verbose mode'),
        DeclareLaunchArgument('live_plot', default_value='false', description='Launch Plot Juggler'),
        DeclareLaunchArgument('dash', default_value='false', description='Launch RQT Dashboard'),
        DeclareLaunchArgument('logging', default_value='false', description='Enable/Disable ROS2 Logging' ),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Whether to use Computer Clock or Sim Clock'),
        DeclareLaunchArgument(
            'robot_configs',
            default_value='[{"name": "robot_1", "type": "go2", "controller": "inverse_dynamics", "init_pose" : "-x 0.0 -y 0.0 -z 5"}]',
            description='A JSON List of robot configurations: MUST specify name, type, controller, and spawn pose'
        ),
        DeclareLaunchArgument('scenario', default_value="None", description='Custom Obstacle Scenario to Spawn e.g. Underbrush, Procedural Underbrush)'),
        DeclareLaunchArgument('obstacles', default_value='[]',
            description= 'A JSON List of obstacles to spawn (e.g {"name": "box", "init_pose" : "-x 3.0 -y 0.0 -z 2"})'),
        DeclareLaunchArgument('recording', default_value='false', description='Whether to log a video of the mujoco scene'),
    ]

    return LaunchDescription(declared_args + [
        OpaqueFunction(function=prepare_world),
        OpaqueFunction(function=launch_mujoco_world),
        # OpaqueFunction(function=launch_obstacles),
        # OpaqueFunction(function=bridge_mujoco_clock),
        OpaqueFunction(function=launch_robot_mapping),
        OpaqueFunction(function=launch_robot_group),
        OpaqueFunction(function=launch_visualization),
        OpaqueFunction(function=launch_plot_juggler),
        OpaqueFunction(function=launch_recording),
    ])


# Example Usage, for Running Multiple Robots
# ros2 launch quad_utils quad_mujoco.py robot_configs:='[{"name": "robot_1", "type": "spirit", "controller": "inverse_dynamics",  "init_pose": "-x 0.0 -y 0.0 -z 15"}, {"name": "robot_2", "type": "go2", "controller": "inverse_dynamics",  "init_pose": "-x 2.0 -y 0.0 -z 15"}]'