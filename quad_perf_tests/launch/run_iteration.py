"""
Headless test-iteration launch file.

Orchestrates one full simulation iteration without any GUI:
  1. Gazebo (headless) + robot spawn (sitting pose via SDF initial_position)
  2. Wait for robot to be upright with active controllers
  3. Stand command
  4. Planning stack (global planner, local planner / NMPC, body force estimator)
  5. Time synchronization node + bag recording
  6. Iteration monitor (auto-terminates on goal, planner failure, or collision)

Designed to be driven repeatedly by run_iterations.py for batch testing.

Usage:
  ros2 launch quad_perf_tests run_iteration.py
  ros2 launch quad_perf_tests run_iteration.py robot_type:=spirit world:=rough.sdf
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess,
    TimerAction, OpaqueFunction, GroupAction, Shutdown,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

import json
import os


def launch_setup(context, *args, **kwargs):
    """Set up the full test-iteration pipeline with event-driven sequencing."""
    robot_type = LaunchConfiguration('robot_type').perform(context)
    world = LaunchConfiguration('world').perform(context)
    plan_delay = float(LaunchConfiguration('plan_delay').perform(context))
    output_dir = LaunchConfiguration('output_dir').perform(context)

    robot_configs_sim = json.dumps([{
        "name": "robot_1",
        "type": robot_type,
        "controller": "inverse_dynamics",
        "init_pose": "-x 0.0 -y 0.0 -z 5",
    }])

    robot_configs_plan = json.dumps([{
        "name": "robot_1",
        "type": robot_type,
        "controller_mode": "inverse_dynamics",
        "reference": "gbpl",
        "twist_input": "none",
    }])

    # --- 1. Launch Gazebo (running, robot spawns in sitting pose) -----------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('quad_utils'), 'launch', 'quad_gazebo.py',
        ])),
        launch_arguments={
            'world': world,
            'gui': 'false',
            'verbose': 'false',
            'live_plot': 'false',
            'dash': 'false',
            'rviz': 'true',
            'logging': 'false',
            'paused': 'false',
            'use_sim_time': 'true',
            'robot_configs': robot_configs_sim,
        }.items(),
    )

    # --- 2. Wait for robot to be upright with active controllers ------------------
    wait_for_robot = Node(
        package='quad_perf_tests',
        executable='wait_for_robot_node',
        name='wait_for_robot',
        parameters=[{
            'namespace': 'robot_1',
            'required_msgs': 500,
            'max_tilt': 0.5,
            'use_sim_time': True,
        }],
    )

    # --- 3. On robot ready: send stand command ------------------------------------
    stand_command = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/robot_1/control/mode',
            'std_msgs/msg/UInt8',
            '{data: 1}',
        ],
        shell=False,
    )

    on_robot_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_robot,
            on_exit=[stand_command],
        )
    )

    # --- 4. On stand sent: launch planners + sync + bags + monitor ----------------
    plan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('quad_utils'), 'launch', 'quad_plan.py',
        ])),
        launch_arguments={
            'logging': 'false',
            'leaping': 'true',
            'ac': 'false',
            'use_sim_time': 'true',
            'force_app': 'false',
            'robot_configs': robot_configs_plan,
        }.items(),
    )

    time_sync = Node(
        package='quad_perf_tests',
        executable='time_sync_node',
        name='time_sync',
        parameters=[{
            'namespace': 'robot_1',
            'slop': 0.02,
            'queue_size': 30,
            'use_sim_time': True,
        }],
    )

    bag_recording = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('quad_perf_tests'), 'launch', 'iteration_logging.py',
        ])),
        launch_arguments={
            'namespace': 'robot_1',
            'robot_type': robot_type,
            'output_dir': output_dir,
        }.items(),
    )

    episode_monitor = Node(
        package='quad_perf_tests',
        executable='episode_monitor_node',
        name='episode_monitor',
        parameters=[{
            'namespace': 'robot_1',
            'settle_time': 2.0,
            'use_sim_time': True,
        }],
        on_exit=[Shutdown()],
    )

    on_stand_sent = RegisterEventHandler(
        OnProcessExit(
            target_action=stand_command,
            on_exit=[
                TimerAction(
                    period=plan_delay,
                    actions=[
                        plan_launch,
                        time_sync,
                        bag_recording,
                        episode_monitor,
                    ],
                ),
            ],
        )
    )

    return [
        gazebo_launch,
        wait_for_robot,
        on_robot_ready,
        on_stand_sent,
    ]


def generate_launch_description():
    default_output = os.path.join(os.getcwd(), 'iteration_bags')

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='go2',
                              description='Robot type (go2, spirit, a1, etc.)'),
        DeclareLaunchArgument('world', default_value='flat.sdf',
                              description='Gazebo world file'),
        DeclareLaunchArgument('plan_delay', default_value='5.0',
                              description='Seconds after stand before launching planners'),
        DeclareLaunchArgument('output_dir', default_value=default_output,
                              description='Directory for output bag files'),
        OpaqueFunction(function=launch_setup),
    ])
