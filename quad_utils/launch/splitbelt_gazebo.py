from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os

def spawn_obstacle(name: str, init_pose: str, sdf):
    return Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{name}',
        output='screen',
        arguments=[
            '-name', str(name), 
            '-file', sdf,
            '-x', init_pose.split()[1],
            '-y', init_pose.split()[3],
            '-z', init_pose.split()[5],
        ]
    )

def generate_launch_description():

    splitbelt_sdf = DeclareLaunchArgument(
        'splitbelt_sdf',
        default_value=PathJoinSubstitution([FindPackageShare('splitbelt_description'), 'sdf', 'splitbelt_treadmill.sdf']),
        description=''
    )

    # Include quad_gazebo.launch.py
    quad_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('quad_utils'), 'launch', 'quad_gazebo.launch.py'])
        ),
        launch_arguments={}.items()
    )

    obstacle = spawn_obstacle('splitbelt',  "-x 0.0 -y 0.0 -z 0.0", splitbelt_sdf)

    # delayed_spawn = TimerAction(period=2.0, actions=[spawn_splitbelt])

    return LaunchDescription([
        quad_gazebo_launch,
        splitbelt_sdf,
    ])