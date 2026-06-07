from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, GroupAction,
    OpaqueFunction, SetLaunchConfiguration
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
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
        'a1': 'a1_description',
        'a2': 'a2_description',
        'go1': 'go1_description',
        'go2': 'go2_description',
        'go2w': 'go2w_description',
        'b2': 'b2_description',
        'spot': 'spot_description',
        'vision60'  : 'vision60_description'
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
    mocap = DeclareLaunchArgument('mocap', default_value='false')
    logging = DeclareLaunchArgument('logging', default_value='false')
    controller = DeclareLaunchArgument('controller', default_value='inverse_dynamics')
    model_path = DeclareLaunchArgument('model_path', default_value='./policies/models/***')
    provider = DeclareLaunchArgument('provider', default_value = "tensorrt")
    estimator = DeclareLaunchArgument('estimator', default_value="comp_filter")
    debug_estimator = DeclareLaunchArgument('debug_estimator', default_value="none",
                                            description='Parallel ride-along estimator (publishes to topics.state.estimate for comparison). Set to "none" to disable.')
    is_hardware = DeclareLaunchArgument('is_hardware', default_value='false')
    namespace = DeclareLaunchArgument('namespace', default_value='robot_1')
    robot_description = DeclareLaunchArgument('robot_description', default_value='')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value = 'false')


    # Paths to included launch files
    quad_utils_pkg = FindPackageShare('quad_utils')
    robot_driver_pkg = FindPackageShare('robot_driver')
    logging_launch_path = PythonLaunchDescriptionSource(
        [quad_utils_pkg, '/launch/logging.py']
    )
    robot_driver_param_file = PathJoinSubstitution([robot_driver_pkg, 'config', 'robot_driver.yaml'])
    robot_driver_topics_file = PathJoinSubstitution([robot_driver_pkg, 'config', 'robot_driver_topics.yaml'])
    robot_specific_param_file = PathJoinSubstitution([quad_utils_pkg, 'config', LaunchConfiguration('robot_type')])
    robot_specific_param_file = [robot_specific_param_file, TextSubstitution(text='.yaml')]

    def _make_rsp_node():
        return Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    LaunchConfiguration('robot_description'), value_type=str),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            remappings=[('joint_states', 'state/joints')]
        )

    def _make_robot_driver_node():
        return Node(
            package='robot_driver',
            executable='robot_driver_node',
            name='robot_driver',
            # output='screen', 
            parameters=[
                robot_driver_param_file,
                robot_driver_topics_file,
                robot_specific_param_file,
                {
                    'controller': LaunchConfiguration('controller'),
                    'is_hardware': LaunchConfiguration('is_hardware'),
                    'mocap': LaunchConfiguration('mocap'),
                    'namespace': LaunchConfiguration('namespace'),
                    'robot_type': LaunchConfiguration('robot_type'),
                    'estimator_id': LaunchConfiguration('estimator'),
                    'debug_estimator_id': LaunchConfiguration('debug_estimator'),
                    'model_path': LaunchConfiguration('model_path'),
                    'provider': LaunchConfiguration('provider'),
                    'robot_description': ParameterValue(
                        LaunchConfiguration('robot_description'),
                        value_type=str),
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }
            ]
        )

    # Hardware mode: this launch is invoked directly (no parent
    # PushRosNamespace) so we push the namespace ourselves. Also brings up
    # robot_state_publisher, since hardware bringup does not have a
    # gazebo-side TF chain.
    hardware_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        _make_rsp_node(),
        _make_robot_driver_node(),
    ], condition=IfCondition(LaunchConfiguration('is_hardware')))

    # Sim mode: robot_bringup.py is included from quad_gazebo.py inside a
    # PushRosNamespace(robot_ns), and robot_bringup.py in turn includes
    # this launch. The namespace is therefore already on the stack — DO
    # NOT push it again. robot_state_publisher is owned by robot_bringup
    # in this path.
    sim_group = GroupAction([
        _make_robot_driver_node(),
    ], condition=UnlessCondition(LaunchConfiguration('is_hardware')))

    return LaunchDescription([
        robot_type,
        mocap,
        logging,
        controller,
        model_path,
        provider,
        estimator,
        debug_estimator,
        is_hardware,
        namespace,
        robot_description,
        use_sim_time,

        # Parse URDF from robot_type
        OpaqueFunction(function=load_robot_params),

        hardware_group,
        sim_group,

        # Optional: logging
        GroupAction([
            IncludeLaunchDescription(logging_launch_path)
        ], condition=IfCondition(LaunchConfiguration('logging')))
    ])
