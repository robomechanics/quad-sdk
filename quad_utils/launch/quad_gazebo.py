from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

import os
import json

def launch_ignition_world(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    verbose = LaunchConfiguration('verbose').perform(context).lower() == 'true'

    pkg_share = FindPackageShare('gazebo_scripts').perform(context)
    world_path = os.path.join(pkg_share, 'worlds', f"{world_name}")
    model_path = os.path.join(pkg_share, 'models')

    # Build the command for `ign gazebo`
    cmd = ['ign', 'gazebo', world_path, '-r']
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
                additional_env={'IGN_GAZEBO_RESOURCE_PATH': (EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH'))}
            )
        ])
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
                    'robot_configs': LaunchConfiguration('robot_configs'),
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

        robot_launch_file = PathJoinSubstitution([
            FindPackageShare('quad_utils'), # May Need Changing, Maybe a Launch File that Contains Quad_Spawn
            'launch',
            'robot_bringup.py'
        ])

        group = GroupAction([
            PushRosNamespace(robot_ns),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch_file),
                launch_arguments={
                    'robot_type': TextSubstitution(text=robot_type),
                    'namespace': TextSubstitution(text=robot_ns),
                    'controller': TextSubstitution(text=controller),
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
            }.items(),
        )
    ]

def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('world', default_value='flat.sdf', description='SDF world file name to load into simulation'),
        DeclareLaunchArgument('gui', default_value='true', description='Whether to launch the Gazebo GUI'),
        DeclareLaunchArgument('paused', default_value='false', description='Whether to start the simulation in a paused state'),
        DeclareLaunchArgument('verbose', default_value='false', description='Launch the simulator in verbose mode'),
        DeclareLaunchArgument('live_plot', default_value='false', description='Launch Plot Juggler'),
        DeclareLaunchArgument('dash', default_value='false', description='Launch RQT Dashboard'),
        DeclareLaunchArgument('logging',default_value='false', description='Enable/Disable ROS2 Logging' ),
        DeclareLaunchArgument(
            'robot_configs',
            default_value='[{"name": "robot_1", "type": "spirit", "controller": "inverse_dynamics"}]',
            description='A JSON List of robot configurations: MUST specifiy name, type, and controller'
        ),
    ]

    return LaunchDescription(declared_args + [
        OpaqueFunction(function=launch_ignition_world),
        OpaqueFunction(function=launch_robot_mapping),
        OpaqueFunction(function=launch_robot_group),
        OpaqueFunction(function=launch_visualization)
    ])


# Example Usage, for Running Multiple Robots
# ros2 launch quad_utils quad_gazebo_multi.py \
#     robot_configs:='[
#         {"name": "robot_1", "type": "spirit", "controller": "inverse_dynamics"},
#         {"name": "robot_2", "type": "go1", "controller": "underbrush"},
#         {"name": "robot_3", "type": "aliengo", "controller": "impedance"}
#     ]'