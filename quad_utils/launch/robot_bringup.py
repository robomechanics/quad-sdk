from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
import os

def load_robot_params(context, *args, **kwargs):
    # Load Robot URDF and Robot Centric Parameters
    robot_type = LaunchConfiguration('robot_type').perform(context)
    
    # Find URDF, SDF, and YAML file for the Corresponding Robot
    if robot_type == 'spirit' or robot_type == 'spirit_rotors':
        desc_pkg = 'spirit_description'
        urdf_file = 'spirit.urdf'
        sdf_file = 'spirit_rotors.sdf' if robot_type == 'spirit_rotors' else 'spirit.sdf'
        config_file = 'spirit.yaml'
    elif robot_type == 'a1':
        desc_pkg = 'a1_description'
        urdf_file = 'a1.urdf'
        sdf_file = 'a1.sdf'
        config_file = 'a1.yaml'
    elif robot_type == 'go2':
        desc_pkg = 'go2_description'
        urdf_file = 'go2.urdf'
        sdf_file = 'go2.sdf'
        config_file = 'go2.yaml'
    else:
        raise RuntimeError(f"[robot_bringup] Unsupported robot type: {robot_type}")

    # Merge the Paths
    desc_path = FindPackageShare(desc_pkg).perform(context)
    urdf_path = os.path.join(desc_path, 'urdf', urdf_file)
    sdf_path = os.path.join(desc_path, 'models','spirit', sdf_file)

    # Load URDF and SDF from disk, Might be Unnecessary
    with open(os.path.join(desc_path, 'models','spirit','urdf', urdf_file), 'r') as f:
        urdf = f.read()
    with open(os.path.join(desc_path, 'models','spirit', sdf_file), 'r') as f:
        sdf = f.read()

    context.robot_urdf = urdf
    context.robot_sdf = sdf
    context.robot_sdf_path = sdf_path
    context.robot_urdf_path = urdf_path

def spawn_sdf_model(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    init_pose = LaunchConfiguration('init_pose').perform(context)
    sdf = context.robot_sdf
    sdf_path = context.robot_sdf_path

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', namespace,
            '-file', sdf_path,
            '-x', init_pose.split()[1],
            '-y', init_pose.split()[3],
            '-z', init_pose.split()[5],
            '-allow_renaming', 'true'
        ],
        additional_env={  
            'IGN_GAZEBO_RESOURCE_PATH': (EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH'))}
    )
    return [spawn_node] 

def ign_ros_bridge(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    quad_utils_path = FindPackageShare('quad_utils').perform(context)

    ign_ros_bridge_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(quad_utils_path, 'launch', 'ign_quad_bridge.py')
            ),
            launch_arguments={
                'namespace': namespace,
            }.items()
        )
    return [ign_ros_bridge_node]

def launch_robot_driver(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    urdf = context.robot_urdf
    sdf = context.robot_sdf
    quad_utils_path = FindPackageShare('quad_utils').perform(context)

    robot_driver_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(quad_utils_path, 'launch', 'robot_driver.py')
            ),
            launch_arguments={
                'robot_type': robot_type,
                'controller': controller,
                'mocap': 'false',
                'is_hardware': 'false',
                'namespace': namespace,
                'robot_description': urdf
            }.items()
        )
    return [robot_driver_node]

def launch_controller_manager(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    urdf = context.robot_urdf
    urdf_path = context.robot_urdf_path
    gazebo_scripts_path = FindPackageShare('gazebo_scripts').perform(context)

    robot_state_urdf_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': urdf
        }],
    )
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[
            os.path.join(gazebo_scripts_path, 'config', 'quad_control.yaml')
        ],
        output='screen'
    )
    controller_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_urdf_node,
            on_start=[controller_manager_node]
        )
    )

    return [robot_state_urdf_node, controller_start_handler]

def spawn_controller_broadcasters(context, *args, **kwards):
    namespace = LaunchConfiguration('namespace').perform(context)
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_state_broadcaster',
            '--controller-manager', f'/{namespace}/controller_manager'
        ],
    )

    spawn_joint_controller = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_controller',
            '--controller-manager', f'/{namespace}/controller_manager'
        ],
        output='screen'
    )

    # Optional delay to give controller_manager time to start
    return[ 
        TimerAction(
            period=3.0,
            actions=[
                spawn_joint_state_broadcaster,
                spawn_joint_controller
            ]
        )
    ]


def launch_contact_state_publisher(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    quad_utils_path = FindPackageShare('quad_utils').perform(context)
    config_file = os.path.join(quad_utils_path, 'config', 'topics_robot.yaml')

    return [
        Node(
            package='gazebo_scripts',
            executable='contact_state_publisher_node',
            # name='contact_state_publisher_node',
            # namespace=namespace,
            output='screen',
            parameters=[config_file,
                        {'namespace': namespace, 
                         'world': world_name}]
        )
    ]

def launch_visualization_plugins(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration

    # Get arguments from the context
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    urdf = context.robot_urdf
    sdf = context.robot_sdf

    quad_utils_path = FindPackageShare('quad_utils').perform(context)

    # Launch the visualization launch file
    visualization_plugins_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(quad_utils_path, 'launch', 'visualization_plugins.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'robot_type': robot_type,
            'controller': controller,
            'robot_description': urdf,
        }.items()
    )

    return [visualization_plugins_launch]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value = 'flat.sdf', description = 'Loaded World SDF File'),
        DeclareLaunchArgument('robot_type', default_value = 'spirit', description='Robot type'),
        DeclareLaunchArgument('namespace', default_value = 'robot_1', description='Robot namespace'),
        DeclareLaunchArgument('controller', default_value = 'inverse_kinematics', description='Controller type'),
        DeclareLaunchArgument('init_pose', default_value = '-x 0.0 -y 0.0 -z 0.5', description= "Initial Robot Position"),
        OpaqueFunction(function=load_robot_params),
        OpaqueFunction(function=spawn_sdf_model), 
        # OpaqueFunction(function=ign_ros_bridge),
        # OpaqueFunction(function=launch_robot_driver),
        OpaqueFunction(function=launch_controller_manager),
        # OpaqueFunction(function=spawn_controller_broadcasters),
        OpaqueFunction(function=launch_contact_state_publisher),
        OpaqueFunction(function= launch_visualization_plugins)
    ])


##Load in Parameters as Needed
    # Parameters to load
    # Find Path to Quad-Utils, Gazebo Scripts
    # quad_utils_path = FindPackageShare('quad_utils').perform(context)
    # gazebo_scripts_path = FindPackageShare('quad_utils').perform(context)
    # param_files = [os.path.join(quad_utils_path, 'config', 'topics_robot.yaml'),
    #                os.path.join(quad_utils_path, 'config, topics_global.yaml'),
    #                os.path.join(quad_utils_path, 'config', config_file),
    #                os.path.join(gazebo_scripts_path), 'config', 'quad_control.yaml']