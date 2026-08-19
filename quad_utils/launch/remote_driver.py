from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import PushRosNamespace, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import xacro


def launch_remote_heartbeat(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    quad_utils_pkg = FindPackageShare('quad_utils').perform(context)

    return [
        Node(
            package='quad_utils',
            executable='remote_heartbeat_node',
            name='remote_heartbeat',
            namespace=namespace,
            output='screen',
            parameters=[
                os.path.join(quad_utils_pkg, 'config', 'remote_heartbeat.yaml'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        )
    ]


def launch_robot_mapping(context, *args, **kwargs):
    mapping_launch_path = PathJoinSubstitution([
        FindPackageShare('quad_utils'),
        'launch',
        'mapping.py'
    ])

    # track_beam:=false is the ground-test case: there is no physical beam for
    # mocap to report, so the mesh is pinned at mesh_pose from the start. The
    # beam world, its grid map and the centreline markers are all unchanged --
    # the robot still plans and walks a beam, it just isn't chasing a rigid
    # body that will never appear (which otherwise costs a track_timeout of
    # TF warnings before the node falls back to the same static pose).
    track_beam = LaunchConfiguration('track_beam').perform(context).lower() == 'true'
    track_frame = LaunchConfiguration('track_frame').perform(context) if track_beam else ''

    return [
        GroupAction([
            PushRosNamespace('mapping'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch_path),
                launch_arguments={
                    # Terrain mesh is loaded by mesh_to_grid_map_node from
                    # quad_sim_scripts/models/<world>/meshes/<world>.ply.
                    # Setting track_frame makes that mesh follow a mocap rigid
                    # body instead of sitting at a fixed pose.
                    'input_type': 'mesh',
                    'world': LaunchConfiguration('world'),
                    'track_frame': track_frame,
                    'mesh_pose': LaunchConfiguration('mesh_pose'),
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items()
            )
        ])
    ]

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

def access_terrain_map(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    return [
        Node(
            package='topic_tools',
            executable='relay',
            name='terrain_map_relay',
            arguments=['/mapping/terrain_map', f'/{namespace}/terrain_map'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
    ]

def launch_visualization_plugins(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)

    desc_pkg_map = {
        'spirit': 'spirit_description',
        'a1': 'a1_description',
        'go1': 'go1_description',
        'go2': 'go2_description',
        'go2w': 'go2w_description',
        'b2': 'b2_description',
        'spot': 'spot_description',
        'vision60'  : 'vision60_description' 
    }
    desc_pkg = desc_pkg_map[robot_type]
    desc_path = FindPackageShare(desc_pkg).perform(context)
    urdf_path = os.path.join(desc_path, 'models', robot_type, 'urdf', f'{robot_type}.urdf.xacro')
    urdf = xacro.process_file(urdf_path).toxml()

    quad_utils_path = FindPackageShare('quad_utils').perform(context)
    viz_launch = PythonLaunchDescriptionSource(
        os.path.join(quad_utils_path, 'launch', 'visualization_plugins.py')
    )

    return [
        GroupAction([
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                viz_launch,
                launch_arguments={
                    'namespace': namespace,
                    'robot_type': robot_type,
                    'robot_description': urdf,
                    'robot_urdf_path': urdf_path,
                    'use_sim_time': 'false',
                }.items()
            )
        ])
    ]

def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('logging', default_value = 'true', description='Whether to enable logging of the simulation data'),
        DeclareLaunchArgument('live_plot', default_value = 'false', description='Whether to enable live plotting of the simulation data'),
        DeclareLaunchArgument('dash', default_value = 'false', description='Whether to enable the dashboard for visualizing the simulation data'),
        DeclareLaunchArgument('use_sim_time', default_value = 'false', description='Whether to use simulation time'),
        DeclareLaunchArgument('world', default_value = 'beam_world_15cm.sdf', description='SDF world file name to load into simulation'),
        # Defaults false: testing runs on the ground, with the beam existing
        # only as a terrain map, so there is no physical beam for mocap to
        # report. Set true again if a tracked beam goes back on the floor.
        DeclareLaunchArgument('track_beam', default_value = 'false', description='Whether the terrain mesh follows the mocap-tracked beam. False (the ground-test default) leaves the beam world loaded with the mesh pinned at mesh_pose.'),
        DeclareLaunchArgument('track_frame', default_value = 'beam', description='TF frame the terrain mesh follows (e.g. a mocap rigid body). Ignored when track_beam is false.'),
        # Placement used whenever the mesh is not tracking -- track_beam:=false,
        # or before the tracked frame appears (and if mocap never comes up).
        # The beam_world_* meshes all share this footprint (they differ only in
        # waist width) and their origin is the centre of the top face, so this
        # puts the start platform under the robot spawn rather than the middle
        # of the beam.
        DeclareLaunchArgument('mesh_pose', default_value = '[1.3716, 0.0, 0.0127, 0.0, 0.0, 0.0]', description='Static terrain mesh placement [x y z roll pitch yaw] used when not tracking'),
        DeclareLaunchArgument('namespace', default_value = 'robot_1', description='Robot namespace'),
        DeclareLaunchArgument('robot_type', default_value = 'go2', description='Robot type'),
    ]

    return LaunchDescription(declared_args + [
        OpaqueFunction(function=launch_remote_heartbeat),
        OpaqueFunction(function=launch_robot_mapping),
        OpaqueFunction(function=access_terrain_map),
        OpaqueFunction(function=launch_visualization_plugins),
        OpaqueFunction(function=launch_visualization)
    ])
