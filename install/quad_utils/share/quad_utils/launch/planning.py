from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, ExecuteProcess, SetLaunchConfiguration
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
import xacro

def load_robot_params(context, *args, **kwargs):
    # Load Robot URDF and Robot Centric Parameters
    robot_type = LaunchConfiguration('robot_type').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    # Find URDF, SDF, and YAML file for the Corresponding Robot
    if robot_type == 'spirit' or robot_type == 'spirit_rotors':
        desc_pkg = 'spirit_description'
        urdf_file = 'spirit.urdf.xacro'
        sdf_file = 'spirit_rotors.sdf.xacro' if robot_type == 'spirit_rotors' else 'spirit.sdf.xacro'
        config_file = 'spirit.yaml'
    elif robot_type == 'a1':
        desc_pkg = 'a1_description'
        urdf_file = 'a1.urdf.xacro'
        sdf_file = 'a1.sdf.xacro'
        config_file = 'a1.yaml'
    elif robot_type == 'go2':
        desc_pkg = 'go2_description'
        urdf_file = 'go2.urdf.xacro'
        sdf_file = 'go2.sdf.xacro'
        config_file = 'go2.yaml'
    elif robot_type == 'go2w':
        desc_pkg = 'go2w_description'
        urdf_file = 'go2w.urdf.xacro'
        sdf_file = 'go2w.sdf.xacro'
        config_file = 'go2w.yaml'
    elif robot_type == 'b2':
        desc_pkg = 'b2_description'
        urdf_file = 'b2.urdf.xacro'
        sdf_file = 'b2.sdf.xacro'
        config_file = 'b2.yaml'
    else:
        raise RuntimeError(f"[robot_bringup] Unsupported robot type: {robot_type}")

    # Merge the Paths
    desc_path = FindPackageShare(desc_pkg).perform(context)
    urdf_path = os.path.join(desc_path, 'models', robot_type, 'urdf', urdf_file)
    sdf_path = os.path.join(desc_path, 'models', robot_type, sdf_file)
    controller_config_path = os.path.join(FindPackageShare('gazebo_scripts').perform(context), 'config', 'quad_control.yaml')
    # Load URDF and SDF from disk, Might be Unnecessary
    urdf = xacro.process_file(urdf_path).toxml()
    sdf = xacro.process_file(sdf_path, mappings={"namespace": namespace, "controller_config_path": controller_config_path}).toxml()

    return [
        SetLaunchConfiguration('robot_urdf', urdf),
        SetLaunchConfiguration('robot_sdf', sdf),
        SetLaunchConfiguration('robot_urdf_path', urdf_path),
        SetLaunchConfiguration('robot_sdf_path', sdf_path)
    ]

def launch_global_planner(context, *args, **kwargs):
    if LaunchConfiguration('reference').perform(context) != 'gbpl':
        return []

    leaping = LaunchConfiguration('leaping').perform(context)
    local_planner_pkg = FindPackageShare('local_planner')
    global_planner_pkg = FindPackageShare('global_body_planner')
    quad_utils_pkg = FindPackageShare('quad_utils')

    local_planner_param_file = PathJoinSubstitution([local_planner_pkg, 'config', 'local_planner.yaml'])
    global_planner_param_file = PathJoinSubstitution([global_planner_pkg, 'config', 'global_body_planner.yaml'])
    global_planner_topics_file = PathJoinSubstitution([global_planner_pkg, 'config', 'global_body_planner_topics.yaml'])
    robot_specific_param_file = os.path.join(quad_utils_pkg.perform(context), 'config', LaunchConfiguration('robot_type').perform(context) + '.yaml')

    return [
        Node(
            package='global_body_planner',
            executable='global_body_planner_node',
            name='global_body_planner',
            # output='screen',
            remappings=[
                ('start_state', 'state/ground_truth'),
                ('goal_state', 'clicked_point')
            ],
            parameters=[local_planner_param_file, 
                        global_planner_topics_file, 
                        global_planner_param_file, 
                        robot_specific_param_file,
                        {'enable_leaping': leaping == 'true',
                         'use_sim_time' : LaunchConfiguration('use_sim_time')}],
        )
    ]


def launch_twist_input_nodes(context, *args, **kwargs):
    if LaunchConfiguration('reference').perform(context) != 'twist':
        return []

    twist_input = LaunchConfiguration('twist_input').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    if twist_input == 'keyboard':
        return [
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                # output='screen',
                prefix='xterm -hold -e',
                parameters=[{'use_sim_time': True}]
                
            )
        ]
    elif twist_input == 'joy':
        # Need to write a custom config file to get this working, and deactivate the safety button
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('teleop_twist_joy'), 'launch', 'teleop-launch.py'
                ])),
                launch_arguments={'joy_config': 'rml-ps3-holonomic', 'use_sim_time' : TextSubstitution(text=use_sim_time)}.items()
            )
        ]
    return []
    
def launch_local_planner(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    ref = LaunchConfiguration('reference').perform(context)
    ac = LaunchConfiguration('ac').perform(context)
    urdf = LaunchConfiguration('robot_urdf').perform(context)
    quad_utils_pkg = FindPackageShare('quad_utils')
    local_planner_pkg = FindPackageShare('local_planner')
    nmpc_controller_pkg = FindPackageShare('nmpc_controller')


    nmpc_controller_param_file = PathJoinSubstitution([nmpc_controller_pkg, 'config', 'nmpc_controller.yaml'])
    local_planner_param_file = PathJoinSubstitution([local_planner_pkg, 'config', 'local_planner.yaml'])
    local_planner_topics_file = PathJoinSubstitution([local_planner_pkg, 'config', 'local_planner_topics.yaml'])
    robot_specific_param_file = os.path.join(quad_utils_pkg.perform(context), 'config', LaunchConfiguration('robot_type').perform(context) + '.yaml')

    if LaunchConfiguration('controller_mode').perform(context) == 'learned':
        return []

    return [
        Node(
            package='local_planner',
            executable='local_planner_node',
            name='local_planner',
            output='screen',
            parameters=[local_planner_param_file,
                nmpc_controller_param_file, 
                local_planner_topics_file,
                robot_specific_param_file, 
                {
                'namespace': namespace,
                'robot_type': robot_type,
                'robot_description': ParameterValue(urdf, value_type=str),
                'local_planner.use_twist_input': ref == 'twist',
                'nmpc_controller.enable_adaptive_complexity': ac == 'true',
                'use_sim_time' : LaunchConfiguration('use_sim_time')
            }]
        )
    ]

def launch_body_force_estimator(context, *args, **kwargs):
    body_force_estimator_pkg = FindPackageShare('body_force_estimator') 
    body_force_estimator_param_file = PathJoinSubstitution([body_force_estimator_pkg, 'config', 'body_force_estimator.yaml'])
    body_force_estimator_topics_file = PathJoinSubstitution([body_force_estimator_pkg, 'config', 'body_force_estimator_topics.yaml'])

    return [
        Node(
            package='body_force_estimator',
            executable='body_force_estimator_node',
            name='body_force_estimator',
            # output='screen',
            parameters=[body_force_estimator_param_file, 
                    body_force_estimator_topics_file,
            {
            'use_sim_time' : LaunchConfiguration('use_sim_time')
            }]
        )
    ]


def launch_plan_publisher(context, *args, **kwargs):
    urdf = LaunchConfiguration('robot_urdf').perform(context)
    quad_utils_package = FindPackageShare('quad_utils')
    plan_publisher_param_file = PathJoinSubstitution([quad_utils_package, 'config', 'trajectory_publisher.yaml'])
    return [
        Node(
            package='quad_utils',
            executable='trajectory_publisher_node',
            name='trajectory_publisher',
            output='screen',
            parameters=[plan_publisher_param_file,
            {
            'robot_description': ParameterValue(urdf, value_type=str),
            'use_sim_time' : LaunchConfiguration('use_sim_time'), 
            'namespace' : LaunchConfiguration('namespace')
            }]
        )
    ]


def launch_logging(context, *args, **kwargs):
    if LaunchConfiguration('logging').perform(context) != 'true':
        return []

    namespace = LaunchConfiguration('namespace').perform(context)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('quad_utils'), 'launch', 'logging.py'
            ])),
            launch_arguments={'namespace': TextSubstitution(text=namespace)}.items()
        )
    ]

def launch_tests(context, *args, **kwargs):
    if LaunchConfiguration('twist_input').perform(context) != 'test':
        return []
    namespace = LaunchConfiguration('namespace').perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('quad_utils'), 'launch', 'testing.py'
            ])),
            launch_arguments={'namespace': TextSubstitution(text=namespace), 'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('reference', default_value='twist'),
        DeclareLaunchArgument('logging', default_value='true'),
        DeclareLaunchArgument('twist_input', default_value='none'),
        DeclareLaunchArgument('controller_mode', default_value='mpc'),
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('robot_type', default_value='spirit'),
        DeclareLaunchArgument('leaping', default_value='true'),
        DeclareLaunchArgument('ac', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=load_robot_params),
        OpaqueFunction(function=launch_logging), 
        OpaqueFunction(function=launch_global_planner),
        OpaqueFunction(function=launch_twist_input_nodes),
        OpaqueFunction(function=launch_local_planner),
        OpaqueFunction(function=launch_body_force_estimator),
        # OpaqueFunction(function=launch_tests)
        # OpaqueFunction(function=launch_plan_publisher),
    ])