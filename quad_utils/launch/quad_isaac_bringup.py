r"""Isaac-side ROS bringup: terrain pipeline + robot_state_publisher + driver.

What this brings up:
    Terrain pipeline (replaces Gazebo's mesh-derived terrain map):
        - flat_terrain_publisher (synthesized raw GridMap)
        - grid_map_filters       (raw -> filtered layers under /mapping)
        - static TF world->map
        - terrain_map relay      (/mapping/terrain_map -> /<ns>/terrain_map)
    Per-robot bringup (under <namespace>):
        - robot_state_publisher  (URDF -> /tf, /joint_states)
        - robot_driver.py        (the controller node + helpers)

What this does NOT bring up:
    - The Isaac simulator      (run run_isaac_bridge.sh in another shell)
    - The planning stack       (run quad_isaac_planning.py separately)

Typical launch order:
    Terminal 1 -- Isaac sim + bridge
        bash <path>/run_isaac_bridge.sh --robot go2 --terrain rough_25cm
    Terminal 2 -- this launch (bringup)
        ros2 launch quad_utils quad_isaac_bringup.py \
            namespace:=robot_1 robot_type:=go2 controller:=inverse_dynamics \
            terrain:=rough_25cm
    Terminal 3 -- planning (sibling launch)
        ros2 launch quad_utils quad_isaac_planning.py \
            namespace:=robot_1 robot_type:=go2

The bridge's --robot and --terrain must match robot_type= and terrain=
on this launch -- bridge loads the STL as collision geometry, and this
launch's mesh_to_grid_map_node reads the same STL's .ply as the
planner's terrain heightmap. They have to be the same name or the
planner's map disagrees with the bridge's physics.

Supported robot_type values (must match an entry in
isaac_plugins/scripts/isaac_bridge.py ROBOT_REGISTRY): spirit, go2.
"""

import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction,
    IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _resolve_urdf(context, *args, **kwargs):
    """Expand the robot xacro into the 'robot_description' LaunchConfig."""
    robot_type = LaunchConfiguration('robot_type').perform(context)
    desc_share = FindPackageShare(f'{robot_type}_description').perform(context)
    urdf_path = (
        f'{desc_share}/models/{robot_type}/urdf/{robot_type}.urdf.xacro'
    )
    urdf = xacro.process_file(urdf_path).toxml()
    return [SetLaunchConfiguration('robot_description', urdf)]


def launch_terrain(context, *args, **kwargs):
    """Raw terrain publisher + grid_map_filters + world->map TF + relay.

    Raw publisher switches based on the `terrain` launch arg:
      - terrain unset (or 'flat'): use isaac_plugins/flat_terrain_publisher.py
        which synthesizes a flat z=0 GridMap. Fine for flat ground.
      - terrain set to a name with a matching quad_sim_scripts model: use
        quad_utils mesh_to_grid_map_node, the same node Gazebo's
        mapping.py uses. It reads
        quad_sim_scripts/share/.../models/<terrain>/meshes/<terrain>.ply
        and publishes real terrain heights so the planner's
        getZRelToTerrain matches the physics under Isaac's loaded STL.

    The `terrain` arg here must match the bridge's `--terrain` flag,
    otherwise the planner's map and the bridge's physical terrain
    disagree (planner thinks ground is flat, physics has bumps -- robot
    sits above h_max relative to a flat ground reference and the planner
    rejects "Invalid start state").
    """
    quad_utils_pkg = FindPackageShare('quad_utils')
    terrain = LaunchConfiguration('terrain').perform(context).strip()
    use_mesh = bool(terrain) and terrain.lower() != 'flat'

    if use_mesh:
        # mesh_to_grid_map_node strips a trailing .sdf if present, but
        # accepts either form. Pass the bare terrain name.
        raw_publisher = GroupAction([
            PushRosNamespace('mapping'),
            Node(
                package='quad_utils',
                executable='mesh_to_grid_map_node',
                name='mesh_to_grid_map_node',
                parameters=[{
                    'frame_id_mesh_loaded': 'map',
                    'grid_map_resolution': 0.05,
                    'layer_name': 'z',
                    'latch_grid_map_pub': True,
                    'verbose': True,
                    'world': terrain,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
            ),
        ])
    else:
        raw_publisher = ExecuteProcess(
            cmd=[
                'python3',
                PathJoinSubstitution([
                    FindPackageShare('isaac_plugins'),
                    'scripts', 'flat_terrain_publisher.py',
                ]),
                '--ros-args',
                '-p', 'use_sim_time:=true',
            ],
            name='flat_terrain_publisher',
        )

    # filter_chain.yaml keys parameters under mapping.grid_map_filters.*,
    # so the node must be in the /mapping namespace with name
    # grid_map_filters for parameter lookup to match.
    grid_map_filter_group = GroupAction([
        PushRosNamespace('mapping'),
        Node(
            package='quad_utils',
            executable='grid_map_filters_demo',
            name='grid_map_filters',
            parameters=[
                PathJoinSubstitution([
                    quad_utils_pkg, 'config', 'filter_chain.yaml'
                ]),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])

    static_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Relay /mapping/terrain_map -> /<ns>/terrain_map (the namespaced
    # topic the local_planner subscribes to).
    terrain_relay_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='topic_tools',
            executable='relay',
            name='terrain_map_relay',
            arguments=['/mapping/terrain_map', 'terrain_map'],
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),
    ])

    return [
        raw_publisher,
        grid_map_filter_group,
        static_world_to_map,
        terrain_relay_group,
    ]


def launch_robot_state_publisher(context, *args, **kwargs):
    """Per-robot robot_state_publisher under the robot namespace."""
    return [GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    LaunchConfiguration('robot_description'),
                    value_type=str,
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[('joint_states', 'state/joints')],
        ),
    ])]


def launch_robot_driver(context, *args, **kwargs):
    """Per-robot robot_driver.py include under the robot namespace.

    robot_driver.py has two conditional groups: hardware_group pushes
    its own PushRosNamespace, sim_group expects the namespace to
    already be on the stack (because in the gazebo flow it is included
    from robot_bringup.py inside `PushRosNamespace(robot_ns)`). With
    is_hardware=false we hit sim_group, so we must wrap the include in
    a PushRosNamespace here -- otherwise robot_driver_node spawns at
    root and never sees /<ns>/state/ground_truth or publishes
    /<ns>/control/joint_command.
    """
    return [GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('quad_utils'),
                    'launch', 'robot_driver.py',
                ])
            ),
            launch_arguments={
                'robot_type': LaunchConfiguration('robot_type'),
                'namespace': LaunchConfiguration('namespace'),
                'controller': LaunchConfiguration('controller'),
                'is_hardware': 'false',
                'mocap': 'false',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': LaunchConfiguration('robot_description'),
            }.items(),
        ),
    ])]


def launch_visualization_plugins(context, *args, **kwargs):
    """Always-on visualization plugins: TF + ground-truth state publishers.

    Spawns three nodes:
      - ground_truth_state_publisher : robot_state_publisher in
        /<ns>/ground_truth namespace, frame_prefix <ns>_ground_truth/,
        subscribes to joint_states remapped to visualization/joint_states.
        Publishes leg link TFs (body -> hip -> upper -> lower -> toe).
      - trajectory_state_publisher : same setup under /<ns>/trajectory
        for the planner's trajectory visualization.
      - rviz_interface_node : subscribes to /<ns>/state/ground_truth,
        publishes TF map -> <ns>_ground_truth/body + the
        visualization/joint_states topic the two state publishers
        subscribe to.

    Why we don't `IncludeLaunchDescription` visualization_plugins.py
    here: that launch runs xacro on <robot>.urdf.xacro which emits
    NUMERIC joint names (0..11) for spirit. The Isaac bridge publishes
    state with RENAMED joint names (j_abad_*, j_hip_*, j_knee_*) -- the
    output of rename_joints.py that produces the *_isaac_clean.urdf the
    bridge loads. When robot_state_publisher's URDF disagrees with the
    incoming joint_states names, kinematic TF lookups silently fail
    (joint defaults to 0) and the leg TFs don't get published. The fix
    is to feed the SAME renamed clean URDF that the bridge uses, so
    the joint name mapping is consistent end-to-end.
    """
    namespace_val = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    desc_share = FindPackageShare(f'{robot_type}_description').perform(context)
    clean_urdf_path = (
        f'{desc_share}/models/{robot_type}/urdf/'
        f'{robot_type}_isaac_clean.urdf'
    )
    with open(clean_urdf_path, 'r') as f:
        clean_urdf = f.read()

    quad_utils_share = FindPackageShare('quad_utils').perform(context)
    rviz_yaml = f'{quad_utils_share}/config/rviz_visualization.yaml'
    robot_yaml = f'{quad_utils_share}/config/{robot_type}.yaml'

    return [GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='ground_truth',
            parameters=[{
                'robot_description': ParameterValue(
                    clean_urdf, value_type=str
                ),
                'frame_prefix': f'{namespace_val}_ground_truth/',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[('joint_states', 'visualization/joint_states')],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='trajectory',
            parameters=[{
                'robot_description': ParameterValue(
                    clean_urdf, value_type=str
                ),
                'frame_prefix': f'{namespace_val}_trajectory/',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[('joint_states', 'visualization/joint_states')],
        ),
        Node(
            package='quad_utils',
            executable='rviz_interface_node',
            name='rviz_interface',
            parameters=[
                rviz_yaml,
                robot_yaml,
                {
                    'tf_prefix': LaunchConfiguration('namespace'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
    ])]


def launch_rviz(context, *args, **kwargs):
    """Optional RViz with quad-sdk's standard viewer config.

    Useful for comparing what the planner sees (filtered
    /mapping/terrain_map) against what the bridge physically loaded
    (the STL collision mesh): if those disagree, the planner rejects
    states the robot is physically in. Loads quad_utils/rviz/
    quad_viewer.rviz which has the relevant displays (TF, robot model,
    terrain GridMap, body trace, etc.) preconfigured.
    """
    if LaunchConfiguration('rviz').perform(context).lower() != 'true':
        return []
    return [Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('quad_utils'),
                'rviz', 'quad_viewer.rviz',
            ]),
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )]


def generate_launch_description() -> LaunchDescription:
    """Build the Isaac-side bringup: terrain + rsp + driver under <namespace>."""
    args = [
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('robot_type', default_value='spirit'),
        DeclareLaunchArgument(
            'controller', default_value='inverse_dynamics',
            description='inverse_dynamics | underbrush | learned',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_description', default_value=''),
        DeclareLaunchArgument(
            'terrain', default_value='',
            description=(
                'Terrain name matching the bridge\'s --terrain flag '
                '(e.g. flat, big_flat, rough_25cm, step_20cm). When '
                'set, the raw map publisher reads the actual STL\'s '
                'heightmap via mesh_to_grid_map_node (matching Gazebo). '
                'Leave empty to fall back to flat_terrain_publisher, '
                'which synthesizes a 20x20m z=0 map without needing '
                'a mesh model.'
            ),
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description=(
                'Spawn RViz2 with quad_utils/rviz/quad_viewer.rviz. '
                'Useful for comparing the planner\'s terrain map '
                'against the bridge\'s physical STL terrain side by '
                'side.'
            ),
        ),
    ]

    return LaunchDescription(
        args + [
            OpaqueFunction(function=_resolve_urdf),
            OpaqueFunction(function=launch_terrain),
            OpaqueFunction(function=launch_robot_state_publisher),
            OpaqueFunction(function=launch_robot_driver),
            OpaqueFunction(function=launch_visualization_plugins),
            OpaqueFunction(function=launch_rviz),
        ]
    )
