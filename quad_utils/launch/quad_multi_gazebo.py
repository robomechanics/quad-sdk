from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

import os
import json

def launch_ignition_world(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    verbose = LaunchConfiguration('verbose').perform(context).lower() == 'true'
    paused = LaunchConfiguration('paused').perform(context).lower() == 'true'

    pkg_share = FindPackageShare('quad_sim_scripts').perform(context)
    world_path = os.path.join(pkg_share, 'worlds', f"{world_name}")

    gz_args = [world_path]
    if not paused:
        gz_args.append('-r')
    if not gui:
        gz_args.append('-s')
    if verbose:
        gz_args.extend(['-v', '4'])

    gz_sim_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    ])

    return [
        GroupAction([
            PushRosNamespace('remote'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={
                    'gz_args': ' '.join(gz_args),
                    'on_exit_shutdown': 'true',
                }.items()
            )
        ])
    ]

def launch_obstacles(context, *args, **kwargs):
    obstacle_launch_path = PathJoinSubstitution([
        FindPackageShare('quad_utils'),
        'launch',
        'spawn_obstacles.py'
        ])
    return[
        GroupAction([
            PushRosNamespace('remote'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(obstacle_launch_path),
                launch_arguments={
                    'scenario' : LaunchConfiguration('scenario'),
                    'obstacles':LaunchConfiguration('obstacles')
                }.items()
            )
        ])
    ]

def bridge_global_clock(context, *args, **kwargs):
    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            namespace='',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            # output='screen'
        )
    ]

def launch_robot_mapping(context, *args, **kwargs):
    mapping_launch_path = PathJoinSubstitution([
        FindPackageShare('quad_utils'),
        'launch',
        'mapping.py'
    ])
    return [
        GroupAction([
            PushRosNamespace('mapping'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch_path),
                launch_arguments={
                    'input_type': 'mesh',
                    'world': LaunchConfiguration('world'),
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items()
            )
        ])
    ]

def launch_robot_group(context, *args, **kwargs):
    robot_configs_raw = LaunchConfiguration('robot_configs').perform(context)
    try:
        robot_configs = json.loads(robot_configs_raw)
    except json.JSONDecodeError as e:
        raise RuntimeError(f"Invalid JSON in 'robot_configs': {e}")

    robot_groups = []

    for config in robot_configs:
        robot_ns = config["name"]
        robot_type = config["type"]
        controller = config["controller"]
        init_pose = config["init_pose"]

        robot_launch_file = PathJoinSubstitution([
            FindPackageShare('quad_utils'), # May Need Changing, Maybe a Launch File that Contains Quad_Spawn
            'launch',
            'robot_bringup.py'
        ])

        group = GroupAction([
            PushRosNamespace(robot_ns),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch_file),
                launch_arguments={
                    'robot_type': TextSubstitution(text=robot_type),
                    'namespace': TextSubstitution(text=robot_ns),
                    'controller': TextSubstitution(text=controller),
                    'init_pose' : TextSubstitution(text=init_pose),
                    'world': LaunchConfiguration('world'),
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items()
            )
        ])
        robot_groups.append(group)

    return robot_groups

def launch_visualization(context, *args, **kwargs):
    visualization_launch_path = PathJoinSubstitution([
        FindPackageShare('quad_utils'),
        'launch',
        'quad_visualization.py'
    ])
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visualization_launch_path),
            launch_arguments={
                'live_plot' : LaunchConfiguration('live_plot'),
                'dash' : LaunchConfiguration('dash'),
                'rviz' : LaunchConfiguration('rviz'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items(),
        )
    ]


def launch_plot_juggler(context, *args, **kwargs):
    live_plot = LaunchConfiguration('live_plot').perform(context).lower() == 'true'

    if not live_plot:
        return []

    return [
        ExecuteProcess(
            cmd=['plotjuggler'],
            # output='screen',
            shell=False
        )
    ]


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('world', default_value='big_flat.sdf', description='SDF world file name to load into simulation'),
        DeclareLaunchArgument('gui', default_value='false', description='Whether to launch the Gazebo GUI. Defaults off — RViz is the primary viewer for the multi-robot demo, and the Gazebo GUI is heavyweight to render six robots at once. Override with gui:=true to bring it back.'),
        DeclareLaunchArgument('paused', default_value='false', description='Whether to start the simulation in a paused state'),
        DeclareLaunchArgument('verbose', default_value='false', description='Launch the simulator in verbose mode'),
        DeclareLaunchArgument('live_plot', default_value='false', description='Launch Plot Juggler'),
        DeclareLaunchArgument('dash', default_value='false', description='Launch RQT Dashboard'),
        DeclareLaunchArgument('logging', default_value='false', description='Enable/Disable ROS2 Logging' ),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Whether to use Computer Clock or Sim Clock'),
        DeclareLaunchArgument(
            'robot_configs',
            # Eight-robot regular-octagon layout on big_flat at radius
            # 8 m (bumped from the 6-robot hex's 7 m to give comparable
            # lateral clearance with two extra bodies in the rotation;
            # big_flat is 22 m × 22 m so ±8 m fits cleanly).
            # multi_robot.py's default goal_states pair these up as
            # diametrically-opposite swaps (vertex i goes to vertex
            # i+4 mod 8), so all eight straight-line paths converge on
            # the origin and CBS sees twenty-eight pairwise OBB
            # conflicts to resolve.
            #
            # Each robot's -Y (yaw) is set to atan2(goal_y - start_y,
            # goal_x - start_x) so it spawns ALREADY POINTED at its
            # goal. Without this, the default Gazebo spawn faces +X and
            # robots whose goal is behind them need an in-place 180°
            # turn, which the GBP-L planner cannot represent (no
            # in-place rotation primitive — yaw is derived from
            # velocity direction). The result is a smoothly-curving
            # reference yaw the NMPC cannot track, runaway joint
            # efforts, fall. Pre-rotating eliminates the U-turn so
            # each robot just walks forward to the opposite vertex.
            #
            # z=3.0 holds the body well above the slab so feet are
            # clear of the ground while spawn_lock pins the chassis.
            # Joints articulate freely (no foot-friction fight) while
            # controllers come up and fold the legs into sit pose.
            # The 8 s hold expires AFTER the legs are folded, so
            # release drops the robot onto its already-sit-folded
            # legs rather than onto an unstable extended-leg stance.
            default_value=(
                '[{"name": "robot_1", "type": "go2", "controller": "inverse_dynamics", "init_pose": "-x  8.00 -y  0.00 -z 3.0 -Y  3.14159"},'
                ' {"name": "robot_2", "type": "go2", "controller": "inverse_dynamics", "init_pose": "-x  5.66 -y  5.66 -z 3.0 -Y -2.35619"},'
                ' {"name": "robot_3", "type": "go2", "controller": "inverse_dynamics", "init_pose": "-x  0.00 -y  8.00 -z 3.0 -Y -1.57080"},'
                ' {"name": "robot_4", "type": "go2", "controller": "inverse_dynamics", "init_pose": "-x -5.66 -y  5.66 -z 3.0 -Y -0.78540"},'
                ' {"name": "robot_5", "type": "go2", "controller": "inverse_dynamics", "init_pose": "-x -8.00 -y  0.00 -z 3.0 -Y  0.00000"},'
                ' {"name": "robot_6", "type": "go2", "controller": "inverse_dynamics", "init_pose": "-x -5.66 -y -5.66 -z 3.0 -Y  0.78540"},'
                ' {"name": "robot_7", "type": "go2", "controller": "inverse_dynamics", "init_pose": "-x  0.00 -y -8.00 -z 3.0 -Y  1.57080"},'
                ' {"name": "robot_8", "type": "go2", "controller": "inverse_dynamics", "init_pose": "-x  5.66 -y -5.66 -z 3.0 -Y  2.35619"}]'
            ),
            description='A JSON List of robot configurations: MUST specify name, type, controller, and spawn pose. init_pose accepts -x/-y/-z and optionally -R/-P/-Y for orientation.'
        ),
        DeclareLaunchArgument('scenario', default_value="None", description='Custom Obstacle Scenario to Spawn e.g. Underbrush, Procedural Underbrush)'),
        DeclareLaunchArgument('obstacles', default_value='[]',
            description= 'A JSON List of obstacles to spawn (e.g {"name": "box", "init_pose" : "-x 3.0 -y 0.0 -z 2"})')
    ]

    return LaunchDescription(declared_args + [
        OpaqueFunction(function=launch_ignition_world),
        OpaqueFunction(function=launch_obstacles),
        OpaqueFunction(function=bridge_global_clock),
        OpaqueFunction(function=launch_robot_mapping),
        OpaqueFunction(function=launch_robot_group),
        OpaqueFunction(function=launch_visualization),
        OpaqueFunction(function=launch_plot_juggler)
    ])


# Example Usage:
#   N-robot CBS swap demo (default 8-robot octagon on big_flat):
#     ros2 launch quad_utils quad_multi.py
#   In a second terminal, run CBS planning:
#     ros2 launch quad_utils multi_robot.py
# For single-robot development, use quad_gazebo.py instead.
