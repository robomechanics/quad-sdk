from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    robot_type = DeclareLaunchArgument('robot_type')
    mocap = DeclareLaunchArgument('mocap', default_value='false')
    logging = DeclareLaunchArgument('logging', default_value='false')
    controller = DeclareLaunchArgument('controller', default_value='inverse_dynamics')
    is_hardware = DeclareLaunchArgument('is_hardware', default_value='true')

    # Paths to included launch files
    quad_utils_pkg = FindPackageShare('quad_utils')
    load_robot_params_path = PythonLaunchDescriptionSource(
        [quad_utils_pkg, '/launch/load_robot_params.launch.py']
    )
    # mocap_launch_path = PythonLaunchDescriptionSource(
    #     [quad_utils_pkg, '/launch/mocap.py']
    # )
    logging_launch_path = PythonLaunchDescriptionSource(
        [quad_utils_pkg, '/launch/logging.py']
    )

    # Main robot driver node
    robot_driver_node = Node(
        package='robot_driver',
        executable='robot_driver_node',
        name='robot_driver',
        output='screen',
        parameters=[{
            'controller': LaunchConfiguration('controller'),
            'is_hardware': LaunchConfiguration('is_hardware')
        }]
    )

    return LaunchDescription([
        robot_type,
        # mocap,
        logging,
        controller,
        is_hardware,

        # Include robot param loader
        IncludeLaunchDescription(
            launch_description_source=load_robot_params_path,
            launch_arguments={'robot_type': LaunchConfiguration('robot_type')}.items()
        ),

        # Launch robot driver
        robot_driver_node,

        # Optional: mocap
        # GroupAction([
        #     IncludeLaunchDescription(mocap_launch_path)
        # ], condition=IfCondition(LaunchConfiguration('mocap'))),

        # Optional: logging
        GroupAction([
            IncludeLaunchDescription(logging_launch_path)
        ], condition=IfCondition(LaunchConfiguration('logging')))
    ])
