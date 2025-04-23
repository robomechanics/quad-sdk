from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os


# def launch_ignition_world(context, *args, **kwargs):
#     world_name = LaunchConfiguration('world').perform(context)

#     world_path = os.path.join(
#         FindPackageShare('gazebo_scripts').perform(context),
#         'worlds',
#         f"{world_name}.sdf"
#     )
#     print(world_path)

#     return [
#         ExecuteProcess(
#             cmd=['ign', 'gazebo', world_path, '--gui', '--verbose', '-r'],
#             output='screen'
#         )
#     ]
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
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='flat'),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=launch_ignition_world)
    ])