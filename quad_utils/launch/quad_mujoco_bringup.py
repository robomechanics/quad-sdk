"""
MuJoCo-side ROS bringup, used by quad_mujoco.py to launch the per-robot stack

What this brings up (under <namespace>, pushed by the parent
quad_mujoco.py via PushRosNamespace):
    - robot_state_publisher  (URDF -> /tf, /joint_states)
    - terrain_map relay      (/mapping/terrain_map -> /<ns>/terrain_map)
    - joint_state_broadcaster + joint_controller spawners (chained)
    - robot_driver.py        (the controller node + helpers)
    - contact_state_publisher_node
    - visualization_plugins.py include
    - mujoco_estimator       (ground-truth odom/imu from mjData)
"""

import os
import sys
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
    SetLaunchConfiguration, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Launch files are loaded by ros2 launch via importlib spec, so this
# directory isn't on sys.path by default. Make sibling helper modules
# (e.g. mujoco_urdf_utils) importable.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mujoco_urdf_utils  # noqa: E402



_ROBOT_FILES = {
    'spirit':        ('spirit_description', 'spirit.urdf.xacro',  'spirit.xml',  'spirit.yaml'),
    'spirit_rotors': ('spirit_description', 'spirit.urdf.xacro',  'spirit.xml',  'spirit.yaml'),
    'a1':            ('a1_description',     'a1.urdf.xacro',      'a1.xml',      'a1.yaml'),
    'a2':            ('a2_description',     'a2.urdf.xacro',      'a2.xml',      'a2.yaml'),
    'go1':           ('go1_description',    'go1.urdf.xacro',     'go1.xml',     'go1.yaml'),
    'go2':           ('go2_description',    'go2.urdf.xacro',     'go2.xml',     'go2.yaml'),
    'go2w':          ('go2w_description',   'go2w.urdf.xacro',    'go2w.xml',    'go2w.yaml'),
    'b2':            ('b2_description',     'b2.urdf.xacro',      'b2.xml',      'b2.yaml'),
    'spot':          ('spot_description',   'spot.urdf.xacro',    'spot.xml',    'spot.yaml'),
}


def load_robot_params(context, *args, **kwargs):
    """Resolve per-robot file paths and synthesize the MuJoCo URDF.

    Stores three launch configurations the downstream functions read:
      - robot_urdf       : the synthesized URDF XML string
      - robot_urdf_path  : the source xacro path (for visualization)
      - robot_mjcf_path  : the per-robot MJCF (assets directory derives
                           from this; the world MJCF includes it)
    """
    robot_type = LaunchConfiguration('robot_type').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)

    if robot_type not in _ROBOT_FILES:
        raise RuntimeError(f"[quad_mujoco_bringup] Unsupported robot type: {robot_type}")
    desc_pkg, urdf_file, mjcf_file, _config_file = _ROBOT_FILES[robot_type]

    desc_path = FindPackageShare(desc_pkg).perform(context)
    urdf_path = os.path.join(desc_path, 'models', robot_type, 'urdf', urdf_file)
    mjcf_path = os.path.join(desc_path, 'models', robot_type, f'{robot_type}_mjc', mjcf_file)

    # `world_path` is a full MJCF path xacro-processed by quad_mujoco.py's
    # prepare_world. Fall back to constructing from `world` so standalone
    # launches still resolve.
    world_path = LaunchConfiguration('world_path').perform(context)
    if not world_path:
        world = LaunchConfiguration('world').perform(context)
        world_path = os.path.join(
            FindPackageShare('quad_sim_scripts').perform(context), 'worlds', world)

    plugin_params_path = os.path.join(
        FindPackageShare('quad_sim_scripts').perform(context),
        'config', 'quad_control.yaml')

    urdf = mujoco_urdf_utils.build_mujoco_urdf(
        robot_type=robot_type,
        urdf_path=urdf_path,
        desc_path=desc_path,
        world_path=world_path,
        namespace=namespace,
        plugin_params_path=plugin_params_path,
    )

    return [
        SetLaunchConfiguration('robot_urdf', urdf),
        SetLaunchConfiguration('robot_urdf_path', urdf_path),
        SetLaunchConfiguration('robot_mjcf_path', mjcf_path),
    ]


def launch_robot_state_publisher(context, *args, **kwargs):
    urdf = LaunchConfiguration('robot_urdf').perform(context)

    set_qos_env = SetEnvironmentVariable(
        name='RMW_QOS_PROFILE_SENSOR_DATA',
        value='rmw_qos_profile_default',
    )

    robot_state_urdf_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': urdf,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    return [set_qos_env, robot_state_urdf_node]


def access_terrain_map(context, *args, **kwargs):
    """Relay /mapping/terrain_map into the robot namespace.

    quad_mujoco.py launches the raw publisher + filter chain under
    /mapping; the local_planner subscribes to <ns>/terrain_map, so we
    fan it into each robot's namespace via topic_tools relay.
    """
    return [
        Node(
            package='topic_tools',
            executable='relay',
            name='terrain_map_relay',
            arguments=['/mapping/terrain_map', 'terrain_map'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
    ]


def spawn_controller_broadcasters(context, *args, **kwargs):
    """Spawn joint_state_broadcaster, then joint_controller, on the
    per-robot controller_manager.

    ros2_control's spawner uses a single PROCESS-WIDE lock to serialise
    controller load/activation calls. With N robots x 2 controllers, 2N
    spawners race for that lock — at six robots that's twelve
    contenders, and even with bumped per-attempt timeouts the loser
    of the race can hit its retry budget and silently exit, leaving
    the robot whose lock-attempt failed with a controller_manager
    but ZERO loaded controllers. Robot stands at spawn, NMPC ticks,
    leg commands fall on the floor.

    Fix: chain joint_controller's spawn behind joint_state_broadcaster
    via OnProcessExit so for each robot the second spawner only fires
    after the first one exits (success or failure). This cuts the
    worst-case lock contention from 2N to N, and combined with the
    bumped 120 s timeouts gives every robot's controllers a fair shot
    even at the six-robot hexagon-swap demo.
    """
    namespace = LaunchConfiguration('namespace').perform(context)

    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_state_broadcaster',
            '--controller-manager', f'/{namespace}/controller_manager',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '180',
        ],
    )

    spawn_joint_controller = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_controller',
            '--controller-manager', f'/{namespace}/controller_manager',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '180',
        ],
    )

    chain_jc_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_joint_controller],
        )
    )


    return [
        TimerAction(
            period=0.5,
            actions=[
                chain_jc_after_jsb,
                spawn_joint_state_broadcaster,
            ],
        )
    ]


def launch_robot_driver(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    estimator = LaunchConfiguration('estimator').perform(context)
    urdf = LaunchConfiguration('robot_urdf').perform(context)
    quad_utils_path = FindPackageShare('quad_utils').perform(context)

    robot_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(quad_utils_path, 'launch', 'robot_driver.py')
        ),
        launch_arguments={
            'robot_type': robot_type,
            'controller': controller,
            'estimator': estimator,
            'mocap': 'false',
            'is_hardware': 'false',
            'namespace': namespace,
            'robot_description': urdf,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )
    return [robot_driver_node]


def launch_contact_state_publisher(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    quad_utils_path = FindPackageShare('quad_utils').perform(context)
    topics_config_file = os.path.join(quad_utils_path, 'config', 'topics_robot.yaml')
    robot_config_file = os.path.join(quad_utils_path, 'config', f'{robot_type}.yaml')

    return [
        Node(
            package='gazebo_scripts',
            executable='contact_state_publisher_node',
            parameters=[
                topics_config_file,
                robot_config_file,
                {
                    'namespace': namespace,
                    'world': world_name,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        )
    ]


def launch_visualization_plugins(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    urdf = LaunchConfiguration('robot_urdf').perform(context)
    urdf_path = LaunchConfiguration('robot_urdf_path').perform(context)
    quad_utils_path = FindPackageShare('quad_utils').perform(context)

    visualization_plugins_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(quad_utils_path, 'launch', 'visualization_plugins.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'robot_type': robot_type,
            'controller': controller,
            'robot_description': urdf,
            'robot_urdf_path': urdf_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )
    return [visualization_plugins_launch]


def launch_mujoco_ground_truth(context, *args, **kwargs):
    """Spawn the mujoco_estimator that publishes ground-truth odom/imu.

    MuJoCo doesn't have the Gazebo IMU/contact sensor stack, so we read
    the floating-base joint pose/velocity straight from mjData inside
    mujoco_plugins/mujoco_estimator. Robot-specific config (joint map,
    odom_free_joint_name) comes from the per-robot yaml.
    """
    robot_type = LaunchConfiguration('robot_type').perform(context)
    quad_utils_path = FindPackageShare('quad_utils').perform(context)
    config_file = os.path.join(quad_utils_path, 'config', f'{robot_type}.yaml')

    ground_truth_node = Node(
        package='mujoco_plugins',
        executable='mujoco_estimator',
        name='mujoco_estimator',
        parameters=[config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    return [ground_truth_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='flat.xml',
                              description='MJCF world file name'),
        DeclareLaunchArgument('world_path', default_value='',
                              description='Full MJCF path (set by quad_mujoco.py)'),
        DeclareLaunchArgument('robot_type', default_value='spirit',
                              description='Robot type'),
        DeclareLaunchArgument('namespace', default_value='robot_1',
                              description='Robot namespace'),
        DeclareLaunchArgument('controller', default_value='inverse_dynamics',
                              description='Controller type'),
        DeclareLaunchArgument('estimator', default_value='comp_filter',
                              description='State estimator type (comp_filter or ekf_filter)'),
        DeclareLaunchArgument('init_pose', default_value='-x 0.0 -y 0.0 -z 0.5',
                              description='Initial robot position'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        OpaqueFunction(function=load_robot_params),
        OpaqueFunction(function=launch_robot_state_publisher),
        OpaqueFunction(function=access_terrain_map),
        OpaqueFunction(function=spawn_controller_broadcasters),
        OpaqueFunction(function=launch_robot_driver),
        OpaqueFunction(function=launch_contact_state_publisher),
        OpaqueFunction(function=launch_visualization_plugins),
        OpaqueFunction(function=launch_mujoco_ground_truth),
    ])
