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
    return [
        GroupAction([
            PushRosNamespace('mapping'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch_path),
                launch_arguments={
                    'input_type': 'mesh',
                    'world': LaunchConfiguration('world'),
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
        DeclareLaunchArgument('logging', default_value = 'false', description='Whether to enable logging of the simulation data'),
        DeclareLaunchArgument('live_plot', default_value = 'false', description='Whether to enable live plotting of the simulation data'),
        DeclareLaunchArgument('dash', default_value = 'false', description='Whether to enable the dashboard for visualizing the simulation data'),
        DeclareLaunchArgument('use_sim_time', default_value = 'false', description='Whether to use simulation time'),
        DeclareLaunchArgument('world', default_value = 'flat.sdf', description='SDF world file name to load into simulation'),
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
