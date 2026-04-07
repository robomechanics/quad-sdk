from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription,
    ExecuteProcess, TimerAction, RegisterEventHandler, SetEnvironmentVariable,
    SetLaunchConfiguration
)
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

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
        xml_file = None
        config_file = 'spirit.yaml'

    elif robot_type == 'a1':
        desc_pkg = 'a1_description'
        urdf_file = 'a1.urdf.xacro'
        sdf_file = 'a1.sdf.xacro'
        xml_file = None
        config_file = 'a1.yaml'

    elif robot_type == 'go2':
        desc_pkg = 'go2_description'
        urdf_file = 'go2.urdf.xacro'
        sdf_file = 'go2.sdf.xacro'
        xml_file = 'go2.xml'
        config_file = 'go2.yaml'

    elif robot_type == 'go2w':
        desc_pkg = 'go2w_description'
        urdf_file = 'go2w.urdf.xacro'
        sdf_file = 'go2w.sdf.xacro'
        xml_file = 'go2w.xml'
        config_file = 'go2w.yaml'

    elif robot_type == 'b2':
        desc_pkg = 'b2_description'
        urdf_file = 'b2.urdf.xacro'
        sdf_file = 'b2.sdf.xacro'
        xml_file = 'b2.xml'
        config_file = 'b2.yaml'

    elif robot_type == 'spot':
        desc_pkg = 'spot_description'
        urdf_file = 'spot.urdf.xacro'
        sdf_file = 'spot.sdf.xacro'
        xml_file = None
        config_file = 'spot.yaml'
    else:
        raise RuntimeError(f"[robot_bringup] Unsupported robot type: {robot_type}")

    # Merge the Paths
    desc_path = FindPackageShare(desc_pkg).perform(context)
    urdf_path = os.path.join(desc_path, 'models', robot_type, 'urdf', urdf_file)
    sdf_path = os.path.join(desc_path, 'models', robot_type, sdf_file)
    xml_path = os.path.join(desc_path, 'models', robot_type, f'{robot_type}_mjc', xml_file) 

    controller_config_path = os.path.join(FindPackageShare('gazebo_scripts').perform(context), 'config', 'quad_control.yaml')
    robot_config_path = os.path.join(FindPackageShare('quad_utils').perform(context), 'config', config_file)

    # Load URDF and SDF from disk
    urdf = xacro.process_file(urdf_path).toxml()
    sdf = xacro.process_file(sdf_path, mappings={"namespace": namespace, "controller_config_path": controller_config_path, "robot_config_path": robot_config_path}).toxml()
    xml = xacro.process_file(xml_path, mappings={"namespace": namespace, "controller_config_path": controller_config_path, "robot_config_path": robot_config_path}).toxml()

    return [
        SetLaunchConfiguration('robot_urdf', urdf),
        SetLaunchConfiguration('robot_sdf', sdf),
        SetLaunchConfiguration('robot_xml', xml),
        SetLaunchConfiguration('robot_urdf_path', urdf_path),
        SetLaunchConfiguration('robot_sdf_path', sdf_path)
    ]

def launch_robot_urdf_node(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    urdf = LaunchConfiguration('robot_urdf').perform(context)

    set_qos_env = SetEnvironmentVariable(
        name='RMW_QOS_PROFILE_SENSOR_DATA',
        value='rmw_qos_profile_default'
    )

    robot_state_urdf_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf}],
        # output='screen'
    )
    return [set_qos_env, robot_state_urdf_node]

def spawn_sdf_model(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    init_pose = LaunchConfiguration('init_pose').perform(context)
    sdf = LaunchConfiguration('robot_sdf').perform(context)
    sdf_path = LaunchConfiguration('robot_sdf_path').perform(context)

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        # output='screen',
        arguments=[
            '-name', namespace,
            '-string', sdf,
            '-x', init_pose.split()[1],
            '-y', init_pose.split()[3],
            '-z', init_pose.split()[5],
            '-allow_renaming', 'true',
            # '--ros-args', '--log-level', 'debug'
        ],
        additional_env={  
            'GZ_SIM_RESOURCE_PATH': (EnvironmentVariable('GZ_SIM_RESOURCE_PATH')),
            'GZ_SIM_SYSTEM_PLUGIN_PATH': (EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH')),
            'GZ_SIM_VERBOSE': '1'}
            
    )
    return [spawn_node] 

def spawn_controller_broadcasters(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    gazebo_scripts_path = FindPackageShare('gazebo_scripts').perform(context)
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_state_broadcaster',
            '--controller-manager', f'/{namespace}/controller_manager',
        ],
        # output='screen'
    ) 

    spawn_joint_controller = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_controller',
            '--controller-manager', f'/{namespace}/controller_manager'
        ],
        # output='screen'
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

def launch_robot_driver(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    urdf = LaunchConfiguration('robot_urdf').perform(context)
    sdf = LaunchConfiguration('robot_sdf').perform(context)
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
                'robot_description': urdf, 
            }.items()
        )
    return [robot_driver_node]

def access_terrain_map(context, *args, **kwargs):
    return [
        Node(
            package='topic_tools', executable='relay',
            name='terrain_map_relay',
            arguments=['/mapping/terrain_map', 'terrain_map'],
            parameters=[{'use_sim_time': True}],
        )
    ]

def harmonic_ros_bridge(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)

    toe_args = []
    toe_remaps = []

    for i in range(4):
        toe_args.append(
            f'/world/default/model/{namespace}/link/toe{i}/sensor/toe{i}_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
        )
        toe_remaps.append(
            (f'/world/default/model/{namespace}/link/toe{i}/sensor/toe{i}_contact/contact',
             f'/{namespace}/gazebo/toe{i}_contact_states')
        )

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=toe_args,
            remappings=toe_remaps,
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ]

def launch_contact_state_publisher(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    world = LaunchConfiguration('world').perform(context)

    config_file = os.path.join(
        FindPackageShare('quad_utils').perform(context),
        'config', 'topics_robot.yaml'
    )

    return [
        Node(
            package='gazebo_scripts',
            executable='contact_state_publisher_node',
            parameters=[
                config_file,
                {
                    'namespace': namespace,
                    'world': world,
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }
            ]
        )
    ]

def launch_simulator_specific_nodes(context, *args, **kwargs):
    simulator = LaunchConfiguration('simulator').perform(context)
 
    if simulator == 'gazebo':
        return [
            *spawn_sdf_model(context),
            *harmonic_ros_bridge(context),
            *spawn_controller_broadcasters(context),
            *launch_contact_state_publisher(context),
        ]
    elif simulator == 'mujoco':
        # Robot is defined in the world XML — no spawn step needed.
        # Add MuJoCo-specific bridges here as they are implemented.
        return [
            *spawn_controller_broadcasters(context),
        ]
 
    return []

def launch_visualization_plugins(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    quad_utils_path = FindPackageShare('quad_utils').perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(quad_utils_path, 'launch', 'visualization_plugins.py')),
            launch_arguments={
                'namespace':        namespace,
                'robot_type':       LaunchConfiguration('robot_type').perform(context),
                'controller':       LaunchConfiguration('controller').perform(context),
                'robot_description':LaunchConfiguration('robot_urdf').perform(context),
                'robot_urdf_path':  LaunchConfiguration('robot_urdf_path').perform(context),
                'use_sim_time':     LaunchConfiguration('use_sim_time').perform(context),
            }.items()
        )
    ]
 
def launch_simulator_specific_nodes(context, *args, **kwargs):
    simulator = LaunchConfiguration('simulator').perform(context)
 
    if simulator == 'gazebo':
        return [
            *spawn_sdf_model(context),
            *harmonic_ros_bridge(context),
            *spawn_controller_broadcasters(context),
            *launch_contact_state_publisher(context),
        ]
    elif simulator == 'mujoco':
        return [
            *spawn_controller_broadcasters(context),
        ]
 
    return []

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulator', default_value='gazebo'),
        DeclareLaunchArgument('world', default_value='flat.sdf'),
        DeclareLaunchArgument('robot_type', default_value='spirit'),
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('controller', default_value='inverse_kinematics'),
        DeclareLaunchArgument('init_pose', default_value='-x 2.0 -y 0.0 -z 15'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        OpaqueFunction(function=load_robot_params),
        OpaqueFunction(function=launch_robot_urdf_node),
        # OpaqueFunction(function=spawn_sdf_model),
        # OpaqueFunction(function=harmonic_ros_bridge),
        # OpaqueFunction(function=spawn_controller_broadcasters),
        OpaqueFunction(function=access_terrain_map),
        
        OpaqueFunction(function=launch_robot_driver),
        OpaqueFunction(function=launch_simulator_specific_nodes),
        # OpaqueFunction(function=launch_contact_state_publisher),
    ])