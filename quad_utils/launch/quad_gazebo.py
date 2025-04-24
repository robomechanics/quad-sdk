"""
Launch file to start up the robot hardware stack and user interface.
...
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def launch_ignition_world(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)
    pkg_share = FindPackageShare('gazebo_scripts').perform(context)
    world_path = os.path.join(pkg_share, 'worlds', f"{world_name}.sdf")
    model_path = os.path.join(pkg_share, 'models')
    return [
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_path, '--gui', '--verbose', '-r'],
            output='screen',
            additional_env={
                'IGN_GAZEBO_RESOURCE_PATH': model_path  # Point to models
            }
        )
    ]

def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('world', default_value='flat.sdf'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('multiple_robots', default_value='false'),
        DeclareLaunchArgument('paused',default_value='false'),
        DeclareLaunchArgument('controller', default_value = 'inverse_dynamics'),
        DeclareLaunchArgument('live_plot', default_value = 'false'),
        DeclareLaunchArgument('dash', default_value='false'),
        DeclareLaunchArgument('logging',default_value='false' ),
        DeclareLaunchArgument('verbose', default_value='false'),
        # DeclareLaunchArgument('use_sim_time', default_value='true'),
        # DeclareLaunchArgument('use_simulator', default_value='true')
        ]
    world_path = PathJoinSubstitution([
        FindPackageShare('gazebo_scripts'),
        'worlds',
        LaunchConfiguration('world')
    ])

    empty_world_launch = PathJoinSubstitution([
        FindPackageShare('quad_utils'),
        'launch',
        'empty_world.py'
    ])

    include_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(empty_world_launch),
        launch_arguments={
            'world': world_path,
            'gui': LaunchConfiguration('gui'),
            'paused' : LaunchConfiguration('paused'),
            'verbose': LaunchConfiguration('verbose'),

        }.items()
    )

    return LaunchDescription(declared_args + [include_empty_world])