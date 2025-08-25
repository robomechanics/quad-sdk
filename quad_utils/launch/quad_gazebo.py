from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node
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

def launch_obstacles(context, *args, **kwargs):
    obstacle_launch_path = PathJoinSubstitution([
        FindPackageShare('quad_utils'),
        'launch',
        'spawn_obstacles.py'
        ])
    return[
        GroupAction([
            PushRosNamespace('remote'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(obstacle_launch_path),
                launch_arguments={
                    'scenario' : LaunchConfiguration('scenario'),
                    'obstacles':LaunchConfiguration('obstacles')
                }.items()
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
            output='screen'
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
                    'robot_configs': LaunchConfiguration('robot_configs'),
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
                    'init_pose' : TextSubstitution(text=init_pose),
                    'world': LaunchConfiguration('world'),
                    'use_sim_time': LaunchConfiguration('use_sim_time')
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
            output='screen',
            shell=False
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
        DeclareLaunchArgument('logging', default_value='false', description='Enable/Disable ROS2 Logging' ),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Whether to use Computer Clock or Sim Clock'),
        DeclareLaunchArgument(
            'robot_configs',
            default_value='[{"name": "robot_1", "type": "spirit", "controller": "inverse_dynamics", "init_pose" : "-x 0.0 -y 0.0 -z 4"}]',
            description='A JSON List of robot configurations: MUST specifiy name, type, controller, and spawn pose'
        ),
        DeclareLaunchArgument('scenario', default_value="None", description='Custom Obstacle Scenario to Spawn e.g. Underbrush, Procedural Underbrush)'),
        DeclareLaunchArgument('obstacles', default_value='[]',
            description= 'A JSON List of obstacles to spawn (e.g {"name": "box", "init_pose" : "-x 3.0 -y 0.0 -z 2"})')
    ]

    return LaunchDescription(declared_args + [
        OpaqueFunction(function=launch_ignition_world),
        OpaqueFunction(function=launch_obstacles),
        OpaqueFunction(function=bridge_global_clock),
        OpaqueFunction(function=launch_robot_mapping),
        OpaqueFunction(function=launch_robot_group),
        OpaqueFunction(function=launch_visualization),
        OpaqueFunction(function=launch_plot_juggler)
    ])


# Example Usage, for Running Multiple Robots
# ros2 launch quad_utils quad_gazebo.py robot_configs:='[{"name": "robot_1", "type": "spirit", "controller": "inverse_dynamics",  "init_pose": "-x 0.0 -y 0.0 -z 15"}, {"name": "robot_2", "type": "go2", "controller": "inverse_dynamics",  "init_pose": "-x 2.0 -y 0.0 -z 15"}]'