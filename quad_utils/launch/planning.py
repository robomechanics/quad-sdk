from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def launch_global_planner(context, *args, **kwargs):
    if LaunchConfiguration('reference').perform(context) != 'gbpl':
        return []

    leaping = LaunchConfiguration('leaping').perform(context)
    return [
        Node(
            package='global_body_planner',
            executable='global_body_planner_node',
            name='global_body_planner',
            output='screen',
            remappings=[
                ('start_state', 'state/ground_truth'),
                ('goal_state', 'clicked_point')
            ],
            parameters=[{'enable_leaping': leaping == 'true'}],
        )
    ]


def launch_twist_input_nodes(context, *args, **kwargs):
    if LaunchConfiguration('reference').perform(context) != 'twist':
        return []

    twist_input = LaunchConfiguration('twist_input').perform(context)

    if twist_input == 'keyboard':
        return [
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard.py',
                name='teleop_twist_keyboard',
                output='screen'
            )
        ]
    elif twist_input == 'joy':
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('teleop_twist_joy'), 'launch', 'teleop.launch.py'
                ])),
                launch_arguments={'joy_config': 'ps3-holonomic'}.items()
            )
        ]
    return []
    
def launch_local_planner(context, *args, **kwargs):
    ref = LaunchConfiguration('reference').perform(context)
    ac = LaunchConfiguration('ac').perform(context)

    return [
        Node(
            package='local_planner',
            executable='local_planner_node',
            name='local_planner',
            output='screen',
            parameters=[{
                'local_planner.use_twist_input': ref == 'twist',
                'nmpc_controller.enable_adaptive_complexity': ac == 'true'
            }]
        )
    ]

def launch_body_force_estimator(context, *args, **kwargs):
    return [
        Node(
            package='body_force_estimator',
            executable='body_force_estimator_node',
            name='body_force_estimator',
            output='screen'
        )
    ]


def launch_plan_publisher(context, *args, **kwargs):
    return [
        Node(
            package='quad_utils',
            executable='trajectory_publisher_node',
            name='plan_publisher',
            output='screen'
        )
    ]


def launch_logging(context, *args, **kwargs):
    if LaunchConfiguration('logging').perform(context) != 'true':
        return []

    namespace = LaunchConfiguration('namespace').perform(context)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('quad_utils'), 'launch', 'logging.launch.py'
            ])),
            launch_arguments={'namespace': TextSubstitution(text=namespace)}.items()
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('reference', default_value='twist'),
        DeclareLaunchArgument('logging', default_value='false'),
        DeclareLaunchArgument('twist_input', default_value='none'),
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('robot_type', default_value='spirit'),
        DeclareLaunchArgument('leaping', default_value='true'),
        DeclareLaunchArgument('ac', default_value='false'),

        # OpaqueFunction(function=launch_global_planner),
        OpaqueFunction(function=launch_twist_input_nodes),
        OpaqueFunction(function=launch_local_planner),
        # OpaqueFunction(function=launch_body_force_estimator),
        # OpaqueFunction(function=launch_plan_publisher),
        # OpaqueFunction(function=launch_logging),
    ])