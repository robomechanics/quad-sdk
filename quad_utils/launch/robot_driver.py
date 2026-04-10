from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, GroupAction,
    OpaqueFunction, SetLaunchConfiguration
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
import xacro

def load_robot_params(context, *args, **kwargs):
    """Parse URDF from robot_type if robot_description was not provided."""
    robot_description = LaunchConfiguration('robot_description').perform(context)

    # If already provided (e.g. from robot_bringup), skip parsing
    if robot_description:
        return []

    robot_type = LaunchConfiguration('robot_type').perform(context)

    desc_pkg_map = {
        'spirit': 'spirit_description',
        'spirit_rotors': 'spirit_description',
        'a1': 'a1_description',
        'go2': 'go2_description',
        'go2w': 'go2w_description',
        'b2': 'b2_description',
        'spot': 'spot_description',
    }
    if robot_type not in desc_pkg_map:
        raise RuntimeError(f"[robot_driver] Unsupported robot type: {robot_type}")

    desc_pkg = desc_pkg_map[robot_type]
    urdf_file = f'{robot_type}.urdf.xacro'

    desc_path = FindPackageShare(desc_pkg).perform(context)
    urdf_path = os.path.join(desc_path, 'models', robot_type, 'urdf', urdf_file)
    urdf = xacro.process_file(urdf_path).toxml()

    return [SetLaunchConfiguration('robot_description', urdf)]


def generate_launch_description():

    robot_type = DeclareLaunchArgument('robot_type', default_value='go2')
    mocap = DeclareLaunchArgument('mocap', default_value='true')
    logging = DeclareLaunchArgument('logging', default_value='false')
    controller = DeclareLaunchArgument('controller', default_value='learned')
    model_path = DeclareLaunchArgument('model_path', default_value='./policies/models/***')
    provider = DeclareLaunchArgument('provider', default_value = "tensorrt")
    estimator = DeclareLaunchArgument('estimator', default_value="comp_filter")
    is_hardware = DeclareLaunchArgument('is_hardware', default_value='true')
    namespace = DeclareLaunchArgument('namespace', default_value='robot_1')
    robot_description = DeclareLaunchArgument('robot_description', default_value='')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value = 'false')


    # Paths to included launch files
    quad_utils_pkg = FindPackageShare('quad_utils')
    robot_driver_pkg = FindPackageShare('robot_driver')
    # load_robot_params_path = PythonLaunchDescriptionSource(
    #     [quad_utils_pkg, '/launch/load_robot_params.launch.py']
    # )
    logging_launch_path = PythonLaunchDescriptionSource(
        [quad_utils_pkg, '/launch/logging.py']
    )
    robot_driver_param_file = PathJoinSubstitution([robot_driver_pkg, 'config', 'robot_driver.yaml'])
    robot_driver_topics_file = PathJoinSubstitution([robot_driver_pkg, 'config', 'robot_driver_topics.yaml'])
    robot_specific_param_file = PathJoinSubstitution([quad_utils_pkg, 'config', LaunchConfiguration('robot_type')])
    robot_specific_param_file = [robot_specific_param_file, TextSubstitution(text='.yaml')]


    # Publish TF from URDF + joint states (needed for TF lookups and RViz)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'robot_description': ParameterValue(LaunchConfiguration('robot_description'), value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[('joint_states', 'state/joints')]
    )

    # Main robot driver node
    robot_driver_node = Node(
        package='robot_driver',
        executable='robot_driver_node',
        name='robot_driver',
        namespace=LaunchConfiguration('namespace'),
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
            'model_path' : LaunchConfiguration('model_path'),
            'provider' : LaunchConfiguration('provider'),
            'robot_description': ParameterValue(LaunchConfiguration('robot_description'), value_type=str),
            'use_sim_time' : LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        robot_type,
        mocap,
        logging,
        controller,
        model_path, 
        provider,
        estimator,
        is_hardware,
        namespace,
        robot_description,
        use_sim_time,

        # Parse URDF from robot_type
        OpaqueFunction(function=load_robot_params),

        # Only launch robot_state_publisher on hardware (robot_bringup handles it in sim)
        GroupAction([robot_state_publisher_node],
                    condition=IfCondition(LaunchConfiguration('is_hardware'))),
        robot_driver_node,

        # Optional: logging
        GroupAction([
            IncludeLaunchDescription(logging_launch_path)
        ], condition=IfCondition(LaunchConfiguration('logging')))
    ])
