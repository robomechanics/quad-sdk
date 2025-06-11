from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():

    robot_type = DeclareLaunchArgument('robot_type')
    mocap = DeclareLaunchArgument('mocap', default_value='false')
    logging = DeclareLaunchArgument('logging', default_value='false')
    controller = DeclareLaunchArgument('controller', default_value='inverse_dynamics')
    estimator = DeclareLaunchArgument('estimator', default_value="comp_filter")
    is_hardware = DeclareLaunchArgument('is_hardware', default_value='true')
    namespace = DeclareLaunchArgument('namespace', default_value='robot_1')
    robot_description = DeclareLaunchArgument('robot_description')


    # Paths to included launch files
    quad_utils_pkg = FindPackageShare('quad_utils')
    robot_driver_pkg = FindPackageShare('robot_driver')
    # load_robot_params_path = PythonLaunchDescriptionSource(
    #     [quad_utils_pkg, '/launch/load_robot_params.launch.py']
    # )
    # mocap_launch_path = PythonLaunchDescriptionSource(
    #     [quad_utils_pkg, '/launch/mocap.py']
    # )
    logging_launch_path = PythonLaunchDescriptionSource(
        [quad_utils_pkg, '/launch/logging.py']
    )

    robot_driver_param_file = PathJoinSubstitution([robot_driver_pkg, 'config', 'robot_driver.yaml'])
    robot_driver_topics_file = PathJoinSubstitution([robot_driver_pkg, 'config', 'robot_driver_topics.yaml'])
    robot_specific_param_file = PathJoinSubstitution([quad_utils_pkg, 'config', LaunchConfiguration('robot_type')])
    robot_specific_param_file = [robot_specific_param_file, TextSubstitution(text='.yaml')]


    # Main robot driver node
    robot_driver_node = Node(
        package='robot_driver',
        executable='robot_driver_node',
        name='robot_driver',
        output='screen',
        parameters=[
            robot_driver_param_file,
            robot_driver_topics_file,
            robot_specific_param_file,
            {
            'controller': LaunchConfiguration('controller'),
            'is_hardware': LaunchConfiguration('is_hardware'),
            'mocap':LaunchConfiguration('mocap'),
            'namespace': LaunchConfiguration('namespace'),
            'robot_type': LaunchConfiguration('robot_type'),
            'estimator_id' : LaunchConfiguration('estimator'),
            'robot_description': ParameterValue(LaunchConfiguration('robot_description'), value_type=str)
        }]
    )

    return LaunchDescription([
        robot_type,
        mocap,
        logging,
        controller,
        estimator,
        is_hardware,
        namespace,
        robot_description,

        # Include robot param loader
        # IncludeLaunchDescription(
        #     launch_description_source=load_robot_params_path,
        #     launch_arguments={'robot_type': LaunchConfiguration('robot_type')}.items()
        # ),

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
