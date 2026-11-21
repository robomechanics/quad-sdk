from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.actions import SetEnvironmentVariable, GroupAction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare
from launch.event_handlers import OnProcessStart, OnProcessExit
from functools import partial
import os
import xacro
import yaml

def cpg_go2_contact_model(sdf, disable_springs=False, disable_damping=False, actuator_profile="legacy"):
    """Remove duplicate fixed-lumped foot spheres; retain instrumented toes.

    Only used by the CPG Go2 simulation. Inertia, friction and joint dynamics
    remain unchanged by default. Explicit diagnostic flags disable one passive
    term at a time; they never change the source SDF.
    """
    import xml.etree.ElementTree as ET
    if actuator_profile not in ('legacy','go2hv_mujoco_v1'):
        raise ValueError('Unknown CPG actuator profile')
    root = ET.fromstring(sdf)
    for leg, name in enumerate(('FL', 'RL', 'FR', 'RR')):
        lower = root.find(f".//link[@name='lower{leg}']")
        toe = root.find(f".//link[@name='toe{leg}']/collision[@name='toe{leg}_collision']")
        duplicate = lower.find(f"collision[@name='lower{leg}_fixed_joint_lump__{name}_foot_collision_3']")
        if toe is None or duplicate is None:
            raise RuntimeError('Go2 collision model changed; review CPG contact mapping')
        if abs(float(toe.findtext('geometry/sphere/radius'))-float(duplicate.findtext('geometry/sphere/radius'))) > 1e-8:
            raise RuntimeError('Go2 foot spheres differ; cannot deduplicate automatically')
        lower.remove(duplicate)
    for joint in root.findall('.//joint'):
        if joint.get('type') == 'revolute':
            dynamics = joint.find('axis/dynamics')
            if actuator_profile == 'go2hv_mujoco_v1':
                for name,value in [('spring_stiffness','0'),('damping','0.1'),('friction','0.2')]:
                    element=dynamics.find(name)
                    if element is None:element=ET.SubElement(dynamics,name)
                    element.text=value
            if disable_springs:
                dynamics.find('spring_stiffness').text = '0'
            if disable_damping:
                dynamics.find('damping').text = '0'
    return ET.tostring(root, encoding='unicode')


def load_robot_params(context, *args, **kwargs):
    # Load Robot URDF and Robot Centric Parameters
    robot_type = LaunchConfiguration('robot_type').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    # Find URDF, SDF, and YAML file for the Corresponding Robot
    if robot_type == 'spirit':
        desc_pkg = 'spirit_description'
        urdf_file = 'spirit.urdf.xacro'
        sdf_file = 'spirit.sdf.xacro'
        config_file = 'spirit.yaml'
    elif robot_type == 'a1':
        desc_pkg = 'a1_description'
        urdf_file = 'a1.urdf.xacro'
        sdf_file = 'a1.sdf.xacro'
        config_file = 'a1.yaml'
    elif robot_type == 'go1':
        desc_pkg = 'go1_description'
        urdf_file = 'go1.urdf.xacro'
        sdf_file = 'go1.sdf.xacro'
        config_file = 'go1.yaml'
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
    elif robot_type == 'spot':
        desc_pkg = 'spot_description'
        urdf_file = 'spot.urdf.xacro'
        sdf_file = 'spot.sdf.xacro'
        config_file = 'spot.yaml'
    elif robot_type == 'vision60':
        desc_pkg = 'vision60_description'
        urdf_file = 'vision60.urdf.xacro'
        sdf_file = 'vision60.sdf.xacro'
        config_file = 'vision60.yaml'
    else:
        raise RuntimeError(f"[robot_bringup] Unsupported robot type: {robot_type}")

    # Merge the Paths
    desc_path = FindPackageShare(desc_pkg).perform(context)
    urdf_path = os.path.join(desc_path, 'models', robot_type, 'urdf', urdf_file)
    sdf_path = os.path.join(desc_path, 'models', robot_type, sdf_file)

    controller_config_path = os.path.join(FindPackageShare('quad_utils').perform(context), 'config', config_file)

    profile=LaunchConfiguration('cpg_actuator_profile').perform(context)
    if profile != 'legacy':
        if profile != 'go2hv_mujoco_v1' or robot_type != 'go2' or LaunchConfiguration('controller').perform(context) not in ('cpg', 'conga_driver'):
            raise RuntimeError('Actuator profile is only supported by Go2 CPG simulation')
        import tempfile
        with open(controller_config_path) as handle:config=yaml.safe_load(handle)
        config.setdefault('/**/joint_controller',{}).setdefault('ros__parameters',{}).setdefault('motor_limits',{})['go2hv_envelope']=True
        with tempfile.NamedTemporaryFile(mode='w',prefix='cpg_go2hv_',suffix='.yaml',delete=False) as handle:
            yaml.safe_dump(config,handle);controller_config_path=handle.name
    # Load URDF and SDF from disk
    urdf = xacro.process_file(urdf_path).toxml()
    sdf = xacro.process_file(
        sdf_path,
        mappings={"namespace": namespace, "controller_config_path": controller_config_path},
    ).toxml()

    if robot_type == 'go2' and LaunchConfiguration('controller').perform(context) in ('cpg', 'conga_driver'):
        sdf = cpg_go2_contact_model(sdf,
            LaunchConfiguration('cpg_disable_springs').perform(context).lower() == 'true',
            LaunchConfiguration('cpg_disable_damping').perform(context).lower() == 'true', profile)
        dump = LaunchConfiguration('cpg_model_dump').perform(context)
        if dump:
            from pathlib import Path
            Path(dump).write_text(sdf)

    return [
        SetLaunchConfiguration('robot_urdf', urdf),
        SetLaunchConfiguration('robot_sdf', sdf),
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

def _parse_init_pose(init_pose):
    """Parse a "-x X -y Y -z Z [-Y YAW] [-R ROLL] [-P PITCH]" init_pose
    string into a flag->value dict. Position flags are required; rotation
    flags are optional (default 0). The CBS hex-swap demo uses -Y to
    spawn each robot pre-rotated toward its goal so NMPC isn't asked to
    track 180° heading reversals."""
    tokens = init_pose.split()
    out = {}
    i = 0
    while i + 1 < len(tokens):
        flag = tokens[i]
        if flag in ('-x', '-y', '-z', '-R', '-P', '-Y'):
            out[flag] = tokens[i + 1]
            i += 2
        else:
            i += 1
    return out


def spawn_sdf_model(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    init_pose = LaunchConfiguration('init_pose').perform(context)
    sdf = LaunchConfiguration('robot_sdf').perform(context)
    sdf_path = LaunchConfiguration('robot_sdf_path').perform(context)

    pose = _parse_init_pose(init_pose)
    args = [
        '-name', namespace,
        '-string', sdf,
        '-x', pose.get('-x', '0.0'),
        '-y', pose.get('-y', '0.0'),
        '-z', pose.get('-z', '0.5'),
        '-allow_renaming', 'true',
    ]
    # Only forward rotation flags when the user actually supplied one;
    # gz create defaults missing flags to 0 anyway, but keeping args
    # tight makes the spawn command easier to read in the launch log.
    for flag in ('-R', '-P', '-Y'):
        if flag in pose:
            args.extend([flag, pose[flag]])

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        # output='screen',
        arguments=args,
        additional_env={
            'GZ_SIM_RESOURCE_PATH': (EnvironmentVariable('GZ_SIM_RESOURCE_PATH')),
            'GZ_SIM_SYSTEM_PLUGIN_PATH': (EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH')),
            'GZ_SIM_VERBOSE': '1'}

    )
    return [spawn_node]

def spawn_controller_broadcasters(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    # Track the spawner directly so launch can stop it and release its lock.
    spawner = ExecutableInPackage(package='controller_manager', executable='spawner')
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            spawner,
            'joint_state_broadcaster',
            '--controller-manager', f'/{namespace}/controller_manager',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '180',
        ],
        # output='screen'
    )

    spawn_joint_controller = ExecuteProcess(
        cmd=[
            spawner,
            'joint_controller',
            '--controller-manager', f'/{namespace}/controller_manager',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '180',
        ],
        # output='screen'
    )

    # joint_controller fires only after joint_state_broadcaster exits.
    # Per-robot serialisation; across robots the six joint_state_broadcaster
    # spawners still go in parallel but at least each robot's own pair is
    # ordered.
    chain_jc_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_joint_controller],
        )
    )

    return [
        TimerAction(
            period=0.5,
            actions=[
                chain_jc_after_jsb,
                spawn_joint_state_broadcaster,
            ]
        )
    ]



def launch_robot_driver(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    estimator = LaunchConfiguration('estimator').perform(context)
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
                'model_path': LaunchConfiguration('model_path'),
                'estimator': estimator,
                'mocap': 'false',
                'is_hardware': 'false',
                'namespace': namespace,
                'robot_description': urdf,
            }.items()
        )
    return [robot_driver_node]

def access_terrain_map(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)

    return [
        Node(
            package='topic_tools',
            executable='relay',
            name='terrain_map_relay',
            arguments=['/mapping/terrain_map', 'terrain_map'],  # relative → becomes /robot_X/terrain_map
            # output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ]

def spawn_sdf_model_with_driver(context, *arg, **kwargs):
    [spawn_node] = spawn_sdf_model(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    urdf = LaunchConfiguration('robot_urdf').perform(context)
    mocap = LaunchConfiguration('mocap').perform(context)
    is_hardware = LaunchConfiguration('is_hardware').perform(context)

    def on_exit_rd(inner_context, *args, **kwargs):
        return launch_robot_driver(namespace, robot_type, controller, urdf, inner_context)
    
    return [
        spawn_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_node,
                on_exit=[OpaqueFunction(function=on_exit_rd)]
            )
        )
    ]

def harmonic_ros_bridge(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    quad_utils_path = FindPackageShare('quad_utils').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    robot_config_path = os.path.join(quad_utils_path, 'config', f'{robot_type}.yaml')

    with open(robot_config_path, 'r') as f:
        robot_config = yaml.safe_load(f)

    root_params = robot_config.get('/**', {}).get('ros__parameters', {})
    toe_names = [
        root_params.get(f'leg_{toe_id}', {}).get('frames', {}).get('toe', f'toe{toe_id}')
        for toe_id in range(4)
    ]

    toe_args, toe_remaps = [],[]
    for toe_name in toe_names:
        toe_args.append(f'/world/default/model/{namespace}/link/{toe_name}/sensor/{toe_name}_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts')
        toe_remaps.append((f'/world/default/model/{namespace}/link/{toe_name}/sensor/{toe_name}_contact/contact', f'/{namespace}/gazebo/{toe_name}_contact_states'))

    contact_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='contact_state_bridge',
        # namespace=namespace,
        # output='screen',
        arguments=toe_args,
        remappings=toe_remaps,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        
    )

    imu_bridge = Node(
                package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        # namespace=namespace,
        # output='screen',
        arguments=[
            f'/world/default/model/{namespace}/model/imu/link/link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        remappings=[
            (f'/world/default/model/{namespace}/model/imu/link/link/sensor/imu_sensor/imu', f'/{namespace}/imu')
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    if LaunchConfiguration('controller').perform(context) in ('cpg', 'conga_driver'):
        return [imu_bridge]
    return [contact_state_bridge, imu_bridge]

def launch_contact_state_publisher(context, *args, **kwargs):
    if LaunchConfiguration('controller').perform(context) in ('cpg', 'conga_driver'):
        return []
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    quad_utils_path = FindPackageShare('quad_utils').perform(context)
    topics_config_file = os.path.join(quad_utils_path, 'config', 'topics_robot.yaml')
    robot_config_file = os.path.join(quad_utils_path, 'config', f'{robot_type}.yaml')

    return [
        Node(
            package='gazebo_scripts',
            executable='contact_state_publisher_node',
            # name='contact_state_publisher_node',
            # namespace=namespace,
            # output='screen',
            parameters=[topics_config_file,
                        robot_config_file,
                        {'namespace': namespace, 
                         'world': world_name, 
                         'use_sim_time' : LaunchConfiguration('use_sim_time')}]
        )
    ]

def launch_visualization_plugins(context, *args, **kwargs):
    # Get arguments from the context
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    urdf = LaunchConfiguration('robot_urdf').perform(context)
    urdf_path = LaunchConfiguration('robot_urdf_path').perform(context)
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
            'robot_urdf_path': urdf_path,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    return [visualization_plugins_launch]

def launch_pinocchio_test_node(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    urdf = LaunchConfiguration('robot_urdf').perform(context)
    sdf = LaunchConfiguration('robot_sdf').perform(context)
    quad_utils_path = FindPackageShare('quad_utils').perform(context)

    quad_pinocchio_node = Node(
         package='quad_pinocchio',
         executable='test_quad_kd_node',
         name='quad_pinocchio',
         # output='screen',  # only needed when debugging the kinematics test node
         parameters=[{
              'namespace': namespace,
              'robot_description' : urdf,
         }
         ]
    )
    return [quad_pinocchio_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value = 'flat.sdf', description = 'Loaded World SDF File'),
        DeclareLaunchArgument('robot_type', default_value = 'spirit', description='Robot type'),
        DeclareLaunchArgument('namespace', default_value = 'robot_1', description='Robot namespace'),
        DeclareLaunchArgument('controller', default_value = 'inverse_kinematics', description='Controller type'),
        DeclareLaunchArgument('cpg_disable_springs', default_value='false'),
        DeclareLaunchArgument('cpg_disable_damping', default_value='false'),
        DeclareLaunchArgument('cpg_model_dump', default_value=''),
        DeclareLaunchArgument('cpg_actuator_profile', default_value='legacy'),
        DeclareLaunchArgument('model_path', default_value='', description='ONNX policy path; empty keeps YAML setting'),
        DeclareLaunchArgument('estimator', default_value = 'ekf_filter', description='State estimator type (comp_filter or ekf_filter)'),
        DeclareLaunchArgument('init_pose', default_value = '-x 2.0 -y 0.0 -z 15', description= "Initial Robot Position"),
        DeclareLaunchArgument('is_hardware', default_value = 'false', description="Simulation or Hardware"),
        DeclareLaunchArgument('mocap', default_value = 'false', description='Launch the Motion Capture Node'),
        DeclareLaunchArgument('use_sim_time', default_value = 'true', description='Use Simulation Clock or Computer Clock'),
        OpaqueFunction(function=load_robot_params),
        OpaqueFunction(function=launch_robot_urdf_node),
        OpaqueFunction(function=spawn_sdf_model),
        OpaqueFunction(function=harmonic_ros_bridge),
        OpaqueFunction(function=access_terrain_map),
        OpaqueFunction(function=spawn_controller_broadcasters),
        OpaqueFunction(function=launch_robot_driver),
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
    #                os.path.join(quad_utils_path), 'config', config_file]
