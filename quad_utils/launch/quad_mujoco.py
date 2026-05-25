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
import shutil
import xacro
from datetime import datetime


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

    desc_share = FindPackageShare(f'{robot_type}_description').perform(context)
    mjcf_dir = os.path.join(desc_share, 'models', robot_type, f'{robot_type}_mjc')

    world_name = world.rsplit('.xml', 1)[0]
    sim_share = FindPackageShare('quad_sim_scripts').perform(context)
    terrain_mesh = os.path.join(sim_share, 'models', world_name, 'meshes', f'{world_name}.stl')

    processed = xacro.process_file(xacro_path, mappings={
        'meshdir': os.path.join(mjcf_dir, 'assets'),
        'mjcf_path': os.path.join(mjcf_dir, f'{robot_type}.xml'),
        'terrain_mesh': terrain_mesh,
    }).toxml()

    out_path = os.path.join('/tmp', f'_quad_world_{robot_type}_{world}')
    with open(out_path, 'w') as f:
        f.write(processed)

    return [SetLaunchConfiguration('world_path', out_path)]

def launch_robot_mapping(context, *args, **kwargs):
    mapping_launch_path = PathJoinSubstitution([
        FindPackageShare('quad_utils'),
        'launch',
        'mjcf_mapping.py'
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

# def launch_mujoco_world(context, *args, **kwargs):
#     world_name = LaunchConfiguration('world').perform(context)
    
#     world_path = PathJoinSubstitution([
#         FindPackageShare('quad_sim_scripts'),
#         'models',
#         world_name
#     ])
    
#     return [
#         Node(
#             package='mujoco_ros2_control',
#             executable='ros2_control_node',
#             name='mujoco',
#             parameters=[
#                 {'robot_description': ''},  # filled by robot_bringup
#                 {'use_sim_time': LaunchConfiguration('use_sim_time')},
#                 {'mujoco_model_path': world_path},
#             ],
#             output='screen'
#         )
#     ]


# def launch_recording(context, *args, **kwargs):
#     recording = LaunchConfiguration('recording').perform(context).lower() == 'true'
#     if not recording:
#         return []

#     if shutil.which('ffmpeg') is None:
#         raise RuntimeError(
#             "recording=true but `ffmpeg` is not installed. "
#             "Install it with `sudo apt install ffmpeg` (and optionally "
#             "`xdotool` to crop the capture to the MuJoCo window)."
#         )

#     quad_logger_src = os.environ.get('QUAD_LOGGER_SRC')
#     if quad_logger_src is None:
#         raise RuntimeError(
#             "recording=true but QUAD_LOGGER_SRC env variable is not set. "
#             "Point it at the quad_logger source dir, e.g. "
#             "`export QUAD_LOGGER_SRC=$HOME/ros2_ws/src/quad-sdk/quad_logger`."
#         )

#     logs_dir = os.path.join(quad_logger_src, 'logs')
#     os.makedirs(logs_dir, exist_ok=True)

#     robot_configs = json.loads(LaunchConfiguration('robot_configs').perform(context))
#     robot_type = robot_configs[0]['type'] if robot_configs else 'mujoco'
#     timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
#     output_path = os.path.join(logs_dir, f'mujoco_{robot_type}_{timestamp}.mp4')

#     display = os.environ.get('DISPLAY', ':0')

#     # Capture via x11grab. The MuJoCo viewer takes a moment to come up, so we
#     # poll for its window with xdotool and then grab that window's region —
#     # this avoids recording the desktop while the viewer is still loading and
#     # crops out everything but MuJoCo. If xdotool isn't available or the
#     # window isn't found within the retry budget, fall back to full-screen
#     # capture so the user still gets a recording.
#     #
#     # `exec` replaces bash with ffmpeg so SIGTERM from launch reaches ffmpeg
#     # directly; ffmpeg writes the mp4 trailer on signal so the file is
#     # playable. sigterm_timeout gives it room to finalize before SIGKILL.
#     record_script = f'''
# set -e
# out={output_path!r}
# disp={display!r}
# geom=""
# if command -v xdotool >/dev/null 2>&1; then
#     for _ in $(seq 1 20); do
#         wid=$(xdotool search --name "MuJoCo" 2>/dev/null | head -n1 || true)
#         if [ -n "$wid" ]; then
#             eval "$(xdotool getwindowgeometry --shell "$wid")"
#             geom="-video_size ${{WIDTH}}x${{HEIGHT}} -i ${{disp}}+${{X}},${{Y}}"
#             break
#         fi
#         sleep 0.5
#     done
# fi
# if [ -z "$geom" ]; then
#     echo "[launch_recording] capturing full display ${{disp}} (xdotool/MuJoCo window unavailable)" >&2
#     geom="-i ${{disp}}"
# fi
# exec ffmpeg -hide_banner -loglevel warning -y \\
#     -f x11grab -framerate 30 $geom \\
#     -c:v libx264 -preset ultrafast -pix_fmt yuv420p "$out"
# '''

#     return [
#         ExecuteProcess(
#             cmd=['bash', '-lc', record_script],
#             output='screen',
#             shell=False,
#             sigterm_timeout='10',
#         )
#     ]



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



















































# def  launch_mujoco_world(context, *args, **kwargs):
#     world_name = LaunchConfiguration('world').perform(context)
#     gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
#     verbose = LaunchConfiguration('verbose').perform(context).lower() == 'true'

#     pkg_share = FindPackageShare('mujoco_scripts').perform(context)
#     world_path = os.path.join(pkg_share, 'worlds', f"{world_name}")  
#     model_path = os.path.join(pkg_share, 'models')

    
#     # Build the command for `mujoco_sim` (or your MuJoCo simulator executable)
#     cmd = ['python3', '-m', 'mujoco.viewer', '--mjcf', world_path]


#     if not gui:
#         cmd.append('--headless')
#     if verbose:
#         cmd.extend(['--verbose', '4'])

#     return [
#         GroupAction([
#             PushRosNamespace('remote'),
#             ExecuteProcess(
#                 cmd=cmd,
#                 output='log',
#                 additional_env={'MUJOCO_MODEL_PATH': (EnvironmentVariable('MUJOCO_MODEL_PATH')),
#                                 'MUJOCO_PLUGIN_PATH': (EnvironmentVariable('MUJOCO_PLUGIN_PATH'))}
#             )
#         ])
#     ]

# def launch_obstacles(context, *args, **kwargs):
#     obstacle_launch_path = PathJoinSubstitution([
#         FindPackageShare('quad_utils'),
#         'launch',
#         'spawn_obstacles.py'
#         ])
#     return[
#         GroupAction([
#             PushRosNamespace('remote'),
#             IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource(obstacle_launch_path),
#                 launch_arguments={
#                     'scenario' : LaunchConfiguration('scenario'),
#                     'obstacles':LaunchConfiguration('obstacles')
#                 }.items()
#             )
#         ])
#     ]

# def bridge_mujoco_clock(context, *args, **kwargs):
#     return [
#         Node(
#             package='mujoco_ros2_control',
#             executable='parameter_bridge',
#             name='mujoco_clock_bridge',
#             namespace='',
#             arguments=['/clock@rosgraph_msgs/msg/Clock[mujoco.msgs.Clock]'],
#             # output='screen'
#         )
#     ]

# def bridge_mujoco_clock(context, *args, **kwargs):
#     return [
#         Node(
#             package='mujoco_ros2_control',
#             executable='ros2_control_node',
#             namespace='robot_1',          # ← must match your robot namespace
#             parameters=[
#                 {'use_sim_time': True},
#             ],
#             remappings=[
#                 ('robot_description', '/robot_1/robot_description')  # ← find the live topic
#             ]
#         )
#     ]