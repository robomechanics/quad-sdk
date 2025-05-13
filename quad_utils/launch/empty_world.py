import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def launch_ignition_sim(context, *args, **kwargs):
    # Evaluate launch args
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    verbose = LaunchConfiguration('verbose').perform(context)
    paused = LaunchConfiguration('paused').perform(context)

    processes = []

    if gui == 'true':
        # Use gz_sim.launch.py when GUI is requested
        gz_args = []

        if paused != 'true':
            gz_args.append('-r')

        gz_args.append(world)

        if verbose == 'true':
            gz_args.append('--verbose')

        gz_sim_launch = os.path.join(
            FindPackageShare('ros_gz_sim').perform(context),
            'launch',
            'gz_sim.launch.py'
        )

        processes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={'gz_args': ' '.join(gz_args)}.items()
            )
        )
    else:
        # Run gzserver directly when GUI is false
        gzserver_cmd = ['gzserver']
        if paused != 'true':
            gzserver_cmd.append('-r')
        gzserver_cmd.append(world)
        if verbose == 'true':
            gzserver_cmd.append('--verbose')

        processes.append(
            ExecuteProcess(
                cmd=gzserver_cmd,
                output='screen'
            )
        )

    return processes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', description='Full path to the SDF world file'),
        DeclareLaunchArgument('gui', default_value='true', description='Launch with GUI'),
        DeclareLaunchArgument('verbose', default_value='false', description='Enable verbose Gazebo output'),
        DeclareLaunchArgument('paused', default_value='false', description='Start simulation paused'),
        OpaqueFunction(function=launch_ignition_sim)
    ])
