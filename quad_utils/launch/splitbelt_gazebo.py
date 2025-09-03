from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import FindPackageShare

import os

def launch_quad_gazebo(context, *args, **kwargs):
    quad_gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('quad_utils'),
        'launch',
        'quad_gazebo.launch.py'
    ])
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(quad_gazebo_launch_path)
            launch_arguments={
            }
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=TextSubstitution(text=os.path.join(
                FindPackageShare('quad_utils').find('quad_utils'), 'worlds', 'empty.world')),
            description='Path to the world file to load'),
        OpaqueFunction(function=launch_quad_gazebo)
    ])