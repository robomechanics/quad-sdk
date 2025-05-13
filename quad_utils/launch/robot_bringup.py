from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
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
    sdf_path = os.path.join(desc_path, 'models','spirit', sdf_file)

    # Load URDF and SDF from disk
    with open(os.path.join(desc_path, 'models','spirit','urdf', urdf_file), 'r') as f:
        urdf = f.read()
    with open(os.path.join(desc_path, 'models','spirit', sdf_file), 'r') as f:
        sdf = f.read()

    context.robot_urdf = urdf
    context.robot_sdf = sdf
    context.robot_sdf_path = sdf_path

def spawn_sdf_model(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    init_pose = LaunchConfiguration('init_pose').perform(context)
    sdf = context.robot_sdf
    sdf_path = context.robot_sdf_path

    ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    print(f"[DEBUG] IGN_GAZEBO_RESOURCE_PATH: {ign_path}")

    # spawn_node = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     output='screen',
    #     arguments=[
    #         '-name', namespace,
    #         '-file', sdf_path,
    #         '-x', init_pose.split()[1],
    #         '-y', init_pose.split()[3],
    #         '-z', init_pose.split()[5],
    #         '-allow_renaming', 'true'
    #     ],
    #     additional_env={  # 👈 THIS FIXES IT
    #         'IGN_GAZEBO_RESOURCE_PATH': ign_path
    #     }
    # )
    # return [spawn_node] 
    return [
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', namespace,
                '-file', sdf_path,
                '-x', init_pose.split()[1],
                '-y', init_pose.split()[3],
                '-z', init_pose.split()[5],
                '-allow_renaming', 'true'
            ],
            output='screen',
            additional_env={'IGN_GAZEBO_RESOURCE_PATH': (EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH'))}
        )
    ]

def launch_robot_driver(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    urdf = context.robot_urdf
    sdf = context.robot_sdf
    quad_utils_path = FindPackageShare('quad_utils').perform(context)
    gazebo_scripts_path = FindPackageShare('quad_utils').perform(context)


    quad_utils_path = FindPackageShare('quad_utils').perform(context)

    robot_driver_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(quad_utils_path, 'launch', 'robot_driver.launch.py')
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

# def launch_controller_plugins(context, *args, **kwargs):
    # namespace = LaunchConfiguration('namespace').perform(context)

    # return [
    #     ExecuteProcess(
    #         cmd=[
    #             'ros2', 'run', 'controller_manager', 'spawner',
    #             'joint_controller', 'joint_state_controller',
    #             '--controller-manager', f'/{namespace}/controller_manager'
    #         ],
    #         output='screen'
    #     )
    # ]
#     return [controller_plugin_node]

# def launch_contact_state_publisher(context, *args, **kwargs):
    # namespace = LaunchConfiguration('namespace').perform(context)
    # robot_type = LaunchConfiguration('robot_type').perform(context)
    # gazebo_scripts_path = FindPackageShare('gazebo_scripts').perform(context)
    # config_file = os.path.join(gazebo_scripts_path, 'config', f'{robot_type}.yaml')

    # return [
    #     Node(
    #         package='gazebo_scripts',
    #         executable='contact_state_publisher_node',
    #         name=f'{namespace}_contact_publisher',
    #         namespace=namespace,
    #         output='screen',
    #         parameters=[config_file]
    #     )
    # ]
#     return [contact_state_publisher_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value = 'spirit', description='Robot type'),
        DeclareLaunchArgument('namespace', default_value = 'robot_1', description='Robot namespace'),
        DeclareLaunchArgument('controller', default_value = 'inverse_kinematics', description='Controller type'),
        DeclareLaunchArgument('init_pose', default_value = '-x 0.0 -y 0.0 -z 0.5', description= "Initial Robot Position"),
        OpaqueFunction(function=load_robot_params),
        OpaqueFunction(function=spawn_sdf_model), 
        # OpaqueFunction(function=launch_robot_driver),
        # OpaqueFunction(function=launch_controller_plugins),
        # OpaqueFunction(function=launch_contact_state_publisher)
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