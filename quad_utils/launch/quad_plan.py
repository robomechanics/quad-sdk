from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition

import json

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
        robot_controller = config["controller_mode"]
        reference = config["reference"]
        twist_input = config["twist_input"]
        # Optional per-robot global_body_planner goal override. Empty string
        # leaves the yaml default in place (preserves single-robot behavior).
        goal_state = config.get("goal_state", "")
        if isinstance(goal_state, (list, tuple)):
            goal_state = json.dumps(list(goal_state))

        planning_launch_file = PathJoinSubstitution([
            FindPackageShare('quad_utils'),
            'launch',
            'planning.py'
        ])

        force_app_launch_file = PathJoinSubstitution([
            FindPackageShare('quad_utils'),
            'launch',
            'force_applicator.py'
        ])
        group = GroupAction([
            PushRosNamespace(robot_ns),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(planning_launch_file),
                launch_arguments={
                    'robot_type': TextSubstitution(text=robot_type),
                    'namespace': TextSubstitution(text=robot_ns),
                    'controller_mode' : TextSubstitution(text=robot_controller),
                    'reference': TextSubstitution(text=reference),
                    'twist_input': TextSubstitution(text=twist_input),
                    'goal_state': TextSubstitution(text=goal_state),
                    'logging' : LaunchConfiguration('logging'),
                    'leaping' : LaunchConfiguration('leaping'),
                    'ac' : LaunchConfiguration('ac'),
                    'cbs_mode' : LaunchConfiguration('cbs_mode'),
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(force_app_launch_file),
                launch_arguments={
                    # Pass common arguments; add/remove based on your force_applicator.py signature
                    'namespace': TextSubstitution(text=robot_ns),
                    'robot_type': TextSubstitution(text=robot_type),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('force_app'))
            ),
        ])
        robot_groups.append(group)

    return robot_groups


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('logging', default_value='false', description='Rosbag Trial Run'),
        DeclareLaunchArgument('leaping', default_value='true', description='Enable Leaping in the Global Planner'),
        DeclareLaunchArgument('ac', default_value='false', description='Enable Adaptive Complexity Planner (Spirit ONLY)'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Simulation Clock or Computer Clock'),
        DeclareLaunchArgument('force_app', default_value='false', description='Launch Force Applicator Alongside Planning'),
        DeclareLaunchArgument('cbs_mode', default_value='false', description='Suppress GBP spin-loop solo planning (used by multi_robot.py for CBS).'),
        DeclareLaunchArgument(
            'robot_configs',
            default_value='[{"name": "robot_1", "type": "go2", "controller_mode" : "inverse_dynamics", "reference": "gbpl", "twist_input": "none"}]',
            description='A JSON List of robot configurations: MUST specifiy name, type, and controller_mode, reference'
        ),
        OpaqueFunction(function=launch_robot_group)
    ])
