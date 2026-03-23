from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import PushRosNamespace, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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

def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('logging', default_value = 'false', description='Whether to enable logging of the simulation data'),
        DeclareLaunchArgument('live_plot', default_value = 'false', description='Whether to enable live plotting of the simulation data'),
        DeclareLaunchArgument('dash', default_value = 'false', description='Whether to enable the dashboard for visualizing the simulation data'),
        DeclareLaunchArgument('use_sim_time', default_value = 'false', description='Whether to use simulation time'),
        DeclareLaunchArgument('world', default_value = 'flat.sdf', description='SDF world file name to load into simulation'),
    ]
    
    return LaunchDescription(declared_args + [
        OpaqueFunction(function=launch_robot_mapping),
        OpaqueFunction(function=launch_visualization)
    ])