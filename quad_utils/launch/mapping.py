from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    # Retrieve launch configurations
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    world = LaunchConfiguration('world').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    # Build absolute path to the SDF world file
    gazebo_scripts_pkg = get_package_share_directory('gazebo_scripts')
    world_file = os.path.join(
        gazebo_scripts_pkg,
        'worlds',
        world,
        f"{world}.sdf"
    )

    # Include Ignition Gazebo via ros_ign_gazebo
    ros_ign_pkg = get_package_share_directory('ros_ign_gazebo')
    ign_launch = os.path.join(ros_ign_pkg, 'launch', 'ign_gazebo.launch.py')
    # Prepare ign_args: world file first, then flags
    ign_args_list = [world_file]
    if gui:
        ign_args_list.append('--gui')
    if paused:
        ign_args_list.append('--paused')
    ign_args = ' '.join(ign_args_list)

    include_ign = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ign_launch),
        launch_arguments={
            'ign_args': ign_args,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Include your mapping launch
    quad_utils_pkg = get_package_share_directory('quad_utils')
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                quad_utils_pkg,
                'launch',
                'mapping.launch.py'
            )
        ),
        launch_arguments={
            'input_type': 'mesh',
            'world': world,
            'robot_type': robot_type
        }.items()
    )

    return [include_ign] #, mapping_launch]


def generate_launch_description():
    ld = LaunchDescription()
    # Declare launch arguments
    for name, default, description in [
        ('robot_type', 'spirit', 'Type of robot to launch'),
        ('gui', 'false', 'Enable GUI'),
        ('paused', 'false', 'Start paused'),
        ('world', 'flat', 'Name of the world directory'),
        ('use_sim_time', 'true', 'Use simulation time')
    ]:
        ld.add_action(
            DeclareLaunchArgument(
                name,
                default_value=default,
                description=description
            )
        )
    # Delay inclusion until after args are parsed
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
