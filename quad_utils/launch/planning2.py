from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, ExecuteProcess, SetLaunchConfiguration, RegisterEventHandler
from launch.actions import OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit, OnExecutionComplete, OnProcessStart
import os
import xacro
from datetime import datetime

def load_robot_params(context, *args, **kwargs):
    # Load Robot URDF and Robot Centric Parameters
    robot_type = LaunchConfiguration('robot_type').perform(context)
    
    # Find URDF, SDF, and YAML file for the Corresponding Robot
    if robot_type == 'spirit' or robot_type == 'spirit_rotors':
        desc_pkg = 'spirit_description'
        urdf_file = 'spirit.urdf.xacro'
        sdf_file = 'spirit_rotors.sdf' if robot_type == 'spirit_rotors' else 'spirit.sdf'
        config_file = 'spirit.yaml'
    elif robot_type == 'a1':
        desc_pkg = 'a1_description'
        urdf_file = 'a1.urdf.xacro'
        sdf_file = 'a1.sdf'
        config_file = 'a1.yaml'
    elif robot_type == 'go2':
        desc_pkg = 'go2_description'
        urdf_file = 'go2.urdf.xacro'
        sdf_file = 'go2.sdf'
        config_file = 'go2.yaml'
    else:
        raise RuntimeError(f"[robot_bringup] Unsupported robot type: {robot_type}")

    # Merge the Paths
    desc_path = FindPackageShare(desc_pkg).perform(context)
    urdf_path = os.path.join(desc_path, 'models', robot_type, 'urdf', urdf_file)
    sdf_path = os.path.join(desc_path, 'models', robot_type, sdf_file)

    # Load URDF and SDF from disk, Might be Unnecessary
    # with open(os.path.join(desc_path, 'models',robot_type,'urdf', urdf_file), 'r') as f:
    #     urdf = f.read()
    urdf = xacro.process_file(urdf_path).toxml()
    with open(os.path.join(desc_path, 'models',robot_type, sdf_file), 'r') as f:
        sdf = f.read()

    return [
        SetLaunchConfiguration('robot_urdf', urdf),
        SetLaunchConfiguration('robot_sdf', sdf),
        SetLaunchConfiguration('robot_urdf_path', urdf_path),
        SetLaunchConfiguration('robot_sdf_path', sdf_path)
    ]

def launch_bag_recording():
    namespace = LaunchConfiguration('namespace')
    timestamp = TextSubstitution(text=datetime.now().strftime('%Y%m%d_%H%M'))
    bag_name = TextSubstitution(text='quad_log')

    full_name = PythonExpression([
        namespace, " + '_' + ", bag_name, " + '_' + ", timestamp
    ])

    topic_prefix = PythonExpression(["'/' + ", namespace])
    topic_list = [
        PythonExpression([topic_prefix, " + '/state'"]),
        PythonExpression([topic_prefix, " + '/state/imu'"]),
        PythonExpression([topic_prefix, " + '/state/trajectory'"]),
        PythonExpression([topic_prefix, " + '/state/ground_truth'"]),
        PythonExpression([topic_prefix, " + '/state/estimate'"]),
        PythonExpression([topic_prefix, " + '/state/grfs'"]),
        PythonExpression([topic_prefix, " + '/mocap_node/quad/pose'"]),
        PythonExpression([topic_prefix, " + '/global_plan'"]),
        PythonExpression([topic_prefix, " + '/local_plan'"]),
        PythonExpression([topic_prefix, " + '/control/grfs'"]),
        PythonExpression([topic_prefix, " + '/control/joint_command'"]),
        PythonExpression([topic_prefix, " + '/control/mode'"]),
        PythonExpression([topic_prefix, " + '/foot_plan_continuous'"]),
        PythonExpression([topic_prefix, " + '/foot_plan_discrete'"]),
        PythonExpression([topic_prefix, " + '/body_force/joint_torques'"]),
        PythonExpression([topic_prefix, " + '/body_force/toe_forces'"]),
        TextSubstitution(text='/terrain_map')
    ]

    bag_dir = PathJoinSubstitution([
        FindPackageShare('quad_logger'), 'bags'
    ])

    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', PathJoinSubstitution([bag_dir, full_name]), '--include-hidden-topics', *topic_list],
            shell=False
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', PathJoinSubstitution([bag_dir, 'archive', full_name]), '--include-hidden-topics', *topic_list],
            shell=False
        )
    ]
# def generate_launch_description():
#     # Launch Configs
#     ref = LaunchConfiguration('reference')
#     logging = LaunchConfiguration('logging')
#     twist_input = LaunchConfiguration('twist_input')
#     namespace = LaunchConfiguration('namespace')
#     robot_type = LaunchConfiguration('robot_type')
#     leaping = LaunchConfiguration('leaping')
#     ac = LaunchConfiguration('ac')
#     use_sim_time = LaunchConfiguration('use_sim_time')

#     # Step 1: Load robot params (Opaque)
#     load_params = OpaqueFunction(function=load_robot_params)

#     log_args = LogInfo(
#         msg=[
#             '[quad_plan] reference=',
#             LaunchConfiguration('reference'),
#             ', twist_input=',
#             LaunchConfiguration('twist_input'),
#             ', logging=',
#             LaunchConfiguration('logging'),
#             ', robot_type=',
#             LaunchConfiguration('robot_type'),
#         ]
#     )

#     # Step 2: Launch logging node
#     namespace_value = namespace.perform(LaunchContext())

#     logger_1, logger_2 = launch_bag_recording(namespace_value, 'quad_log')
#     logging_group = GroupAction([logger_1, logger_2])

#     # Step 3: Launch twist input
#     # keyboard_condition = 
#     twist_condition = IfCondition(PythonExpression(["'", LaunchConfiguration('reference'), "' == 'twist' and '", LaunchConfiguration('twist_input'), "' == 'keyboard'"]))
#     # joy_condition = 
#     twist_node = Node(
#         condition=twist_condition,
#         package='teleop_twist_keyboard',
#         executable='teleop_twist_keyboard',
#         name='teleop_twist_keyboard',
#         output='screen',
#         prefix='xterm -hold -e',
#         parameters=[{'use_sim_time': True}]
#     )

#     # Step 4: Launch local planner
#     local_planner_node = Node(
#         package='local_planner',
#         executable='local_planner_node',
#         name='local_planner',
#         output='screen',
#         parameters=[
#             PathJoinSubstitution([FindPackageShare('local_planner'), 'config', 'local_planner.yaml']),
#             PathJoinSubstitution([FindPackageShare('nmpc_controller'), 'config', 'nmpc_controller.yaml']),
#             PathJoinSubstitution([FindPackageShare('local_planner'), 'config', 'local_planner_topics.yaml']),
#             PathJoinSubstitution([FindPackageShare('quad_utils'), 'config', PythonExpression([ "'", LaunchConfiguration('robot_type'), "' + '.yaml'"])]),
#             {
#                 'namespace': namespace,
#                 'robot_type': robot_type,
#                 'robot_description': ParameterValue(LaunchConfiguration('robot_urdf'), value_type=str),
#                 'local_planner.use_twist_input': PythonExpression(["'", LaunchConfiguration('reference'), "' == 'twist'"]),
#                 'nmpc_controller.enable_adaptive_complexity': PythonExpression(["'", LaunchConfiguration('ac'), "' == 'true'"]),
#                 'use_sim_time': LaunchConfiguration('use_sim_time')
#             }
#         ]
#     )

#     # Event handlers for sequencing
#     event_handlers = [
#         RegisterEventHandler(
#             OnExecutionComplete(
#                 target_action=load_params,
#                 on_completion=[logging_group]
#             )
#         ),
#         RegisterEventHandler(
#             OnProcessStart(
#                 target_action=logging_group,
#                 on_start=[twist_node]
#             )
#         ),
#         RegisterEventHandler(
#             OnProcessStart(
#                 target_action=twist_node,
#                 on_start=[local_planner_node],
#             )
#         )
#     ]

#     return LaunchDescription([
#         DeclareLaunchArgument('reference', default_value='twist'),
#         DeclareLaunchArgument('logging', default_value='true'),
#         DeclareLaunchArgument('twist_input', default_value='keyboard'),
#         DeclareLaunchArgument('namespace', default_value='robot_1'),
#         DeclareLaunchArgument('robot_type', default_value='spirit'),
#         DeclareLaunchArgument('leaping', default_value='true'),
#         DeclareLaunchArgument('ac', default_value='false'),
#         DeclareLaunchArgument('use_sim_time', default_value='true'),
#         load_params,
#         log_args,
#         *event_handlers
#     ])
def generate_launch_description():
    ld = [
        DeclareLaunchArgument('reference', default_value='twist'),
        DeclareLaunchArgument('logging', default_value='true'),
        DeclareLaunchArgument('twist_input', default_value='keyboard'),
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('robot_type', default_value='spirit'),
        DeclareLaunchArgument('leaping', default_value='true'),
        DeclareLaunchArgument('ac', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true')
    ]

    ref = LaunchConfiguration('reference')
    logging = LaunchConfiguration('logging')
    twist_input = LaunchConfiguration('twist_input')
    namespace = LaunchConfiguration('namespace')
    robot_type = LaunchConfiguration('robot_type')
    leaping = LaunchConfiguration('leaping')
    ac = LaunchConfiguration('ac')
    use_sim_time = LaunchConfiguration('use_sim_time')

    load_params = OpaqueFunction(function=load_robot_params)

    log_args = LogInfo(
        msg=[
            '[quad_plan] reference=', ref,
            ', twist_input=', twist_input,
            ', logging=', logging,
            ', robot_type=', robot_type,
        ]
    )

    logger_1, logger_2 = launch_bag_recording()
    logging_group = GroupAction([logger_1, logger_2])

    twist_condition = IfCondition(PythonExpression(["'", ref, "' == 'twist' and '", twist_input, "' == 'keyboard'"]))

    twist_node = Node(
        condition=twist_condition,
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -hold -e',
        parameters=[{'use_sim_time': True}]
    )

    local_planner_node = Node(
        package='local_planner',
        executable='local_planner_node',
        name='local_planner',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('local_planner'), 'config', 'local_planner.yaml']),
            PathJoinSubstitution([FindPackageShare('nmpc_controller'), 'config', 'nmpc_controller.yaml']),
            PathJoinSubstitution([FindPackageShare('local_planner'), 'config', 'local_planner_topics.yaml']),
            PathJoinSubstitution([FindPackageShare('quad_utils'), 'config', PythonExpression(["'", robot_type, "' + '.yaml'"])]),
            {
                'namespace': namespace,
                'robot_type': robot_type,
                'robot_description': ParameterValue(LaunchConfiguration('robot_urdf'), value_type=str),
                'local_planner.use_twist_input': PythonExpression(["'", ref, "' == 'twist'"]),
                'nmpc_controller.enable_adaptive_complexity': PythonExpression(["'", ac, "' == 'true'"]),
                'use_sim_time': use_sim_time
            }
        ]
    )

    event_handlers = [
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=load_params,
                on_completion=[logger_1, logger_2]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=logger_1,
                on_start=[twist_node]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=twist_node,
                on_start=[local_planner_node]
            )
        )
    ]

    ld += [
        load_params,
        log_args,
        *event_handlers
    ]

    return LaunchDescription(ld)
