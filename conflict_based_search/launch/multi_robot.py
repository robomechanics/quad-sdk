"""Launch a multi-robot Quad-SDK scenario with conflict-based search.

This is the ROS2 port of multi-robot.launch from the ROS1 devel_cbs branch.

Implementation: this launch is a thin wrapper around
`quad_utils/launch/quad_plan.py`. quad_plan owns the canonical "spawn N
robots' planning stacks" logic; multi_robot.py adds the central
`conflict_based_search` node on top, plus a small amount of CBS-specific
JSON validation/munging.

Differences vs. quad_plan.py:

  * Each robot's per-robot config requires a `goal_state` field
    (a `[x, y]` list). Without distinct goals there is nothing for CBS
    to resolve. quad_plan.py treats goal_state as optional (yaml fallback);
    this launch enforces it.

  * The `reference` field is forced to `gbpl` for every robot. The CBS
    node calls `/<robot>/plan_with_constraints`, which only exists when
    that robot's `global_body_planner_node` is running. Twist-controlled
    robots have no global planner and therefore cannot participate.

Expected end-to-end workflow:
    1. ros2 launch quad_utils quad_gazebo.py robot_configs:='[...]'
    2. ros2 topic pub --once /<ns>/control/mode std_msgs/UInt8 "data: 1"
       (once per robot, to stand it)
    3. ros2 launch conflict_based_search multi_robot.py robot_configs:='[...]'
"""

import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _validate_and_munge(context):
    """Parse `robot_configs`, enforce CBS requirements (goal_state per
    robot, reference = 'gbpl'), and return the rewritten JSON string.
    Stored back into a launch configuration so quad_plan.py picks up the
    munged version."""
    raw = LaunchConfiguration('robot_configs').perform(context)
    try:
        configs = json.loads(raw)
    except json.JSONDecodeError as e:
        raise RuntimeError(f"Invalid JSON in 'robot_configs': {e}")

    for cfg in configs:
        for required in ('name', 'type', 'controller_mode', 'goal_state'):
            if required not in cfg:
                raise RuntimeError(
                    f"robot_configs entry {cfg!r} is missing required field "
                    f"'{required}'. Each robot must have name, type, "
                    f"controller_mode, and goal_state at minimum.")
        if 'reference' in cfg and cfg['reference'] != 'gbpl':
            print(f"[multi_robot] WARNING: robot {cfg['name']!r} has "
                  f"reference={cfg['reference']!r}; CBS only works when "
                  f"every robot uses 'gbpl'. Forcing to 'gbpl'.")
        cfg['reference'] = 'gbpl'
        cfg.setdefault('twist_input', 'none')

    return [SetLaunchConfiguration('robot_configs', json.dumps(configs))]


def launch_quad_plan(context, *args, **kwargs):
    """Delegate per-robot planning bring-up to quad_plan.py."""
    quad_plan_launch = PathJoinSubstitution([
        FindPackageShare('quad_utils'), 'launch', 'quad_plan.py'
    ])
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(quad_plan_launch),
            launch_arguments={
                'robot_configs': LaunchConfiguration('robot_configs'),
                'logging': LaunchConfiguration('logging'),
                'leaping': LaunchConfiguration('leaping'),
                'ac': LaunchConfiguration('ac'),
                # cbs_mode hard-on for the CBS pipeline so each per-robot
                # GBP suppresses its spin-loop solo planning from boot.
                # Without this, the first GBPs to finish waitForData()
                # publish solo plans before CBS finishes its search; the
                # local_planner subscribes to the solo plan, then the
                # later CBS plan arrival corrupts internal indexing
                # state and SIGSEGVs.
                'cbs_mode': 'true',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'force_app': LaunchConfiguration('force_app'),
            }.items(),
        )
    ]


def launch_cbs(context, *args, **kwargs):
    cbs_pkg = FindPackageShare('conflict_based_search')
    cbs_param_file = PathJoinSubstitution([cbs_pkg, 'config', 'conflict_based_search.yaml'])

    # Re-parse the (already-validated) config for the robot_names list.
    configs = json.loads(LaunchConfiguration('robot_configs').perform(context))
    robot_names = [c['name'] for c in configs]

    return [
        Node(
            package='conflict_based_search',
            executable='conflict_based_search_node',
            name='conflict_based_search',
            output='screen',
            parameters=[
                cbs_param_file,
                {
                    'robot_names': robot_names,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        )
    ]


def generate_launch_description():
    # Args mirror quad_plan.py exactly so callers can swap one launch
    # for the other by changing only the filename + adding goal_state to
    # each robot's config entry.
    return LaunchDescription([
        DeclareLaunchArgument('logging', default_value='false', description='Rosbag Trial Run'),
        # Default leaping off for CBS: the global GBP leap envelope
        # (t_s_min/max, dz0_min/max in global_body_planner.yaml) is tuned
        # for Spirit, so on go2/a1/spot the leap primitive is dynamically
        # awkward. Override with leaping:=true on the command line if you
        # are running a Spirit-only scenario.
        DeclareLaunchArgument('leaping', default_value='true', description='Enable Leaping in the Global Planner (off by default for CBS — leap dynamics are Spirit-tuned)'),
        DeclareLaunchArgument('ac', default_value='false', description='Enable Adaptive Complexity Planner (Spirit ONLY)'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Simulation Clock or Computer Clock'),
        DeclareLaunchArgument('force_app', default_value='false', description='Launch Force Applicator Alongside Planning'),
        DeclareLaunchArgument(
            'robot_configs',
            # Eight-robot octagon-swap demo on big_flat (radius 8 m).
            # Pair the (start, goal) tuples with quad_gazebo.py's
            # matching octagon-vertex init_poses (i -> i+4 mod 8) so
            # every robot's straight-line path crosses every other
            # robot's near the origin. Twenty-eight pairwise OBB
            # conflicts to resolve.
            #
            #   robot_1: ( 8.00,  0.00) -> (-8.00,  0.00)
            #   robot_2: ( 5.66,  5.66) -> (-5.66, -5.66)
            #   robot_3: ( 0.00,  8.00) -> ( 0.00, -8.00)
            #   robot_4: (-5.66,  5.66) -> ( 5.66, -5.66)
            #   robot_5: (-8.00,  0.00) -> ( 8.00,  0.00)
            #   robot_6: (-5.66, -5.66) -> ( 5.66,  5.66)
            #   robot_7: ( 0.00, -8.00) -> ( 0.00,  8.00)
            #   robot_8: ( 5.66, -5.66) -> (-5.66,  5.66)
            default_value=(
                '[{"name": "robot_1", "type": "go2", "controller_mode": "inverse_dynamics",'
                ' "twist_input": "none", "goal_state": [-8.00,  0.00]},'
                ' {"name": "robot_2", "type": "go2", "controller_mode": "inverse_dynamics",'
                ' "twist_input": "none", "goal_state": [-5.66, -5.66]},'
                ' {"name": "robot_3", "type": "go2", "controller_mode": "inverse_dynamics",'
                ' "twist_input": "none", "goal_state": [ 0.00, -8.00]},'
                ' {"name": "robot_4", "type": "go2", "controller_mode": "inverse_dynamics",'
                ' "twist_input": "none", "goal_state": [ 5.66, -5.66]},'
                ' {"name": "robot_5", "type": "go2", "controller_mode": "inverse_dynamics",'
                ' "twist_input": "none", "goal_state": [ 8.00,  0.00]},'
                ' {"name": "robot_6", "type": "go2", "controller_mode": "inverse_dynamics",'
                ' "twist_input": "none", "goal_state": [ 5.66,  5.66]},'
                ' {"name": "robot_7", "type": "go2", "controller_mode": "inverse_dynamics",'
                ' "twist_input": "none", "goal_state": [ 0.00,  8.00]},'
                ' {"name": "robot_8", "type": "go2", "controller_mode": "inverse_dynamics",'
                ' "twist_input": "none", "goal_state": [-5.66,  5.66]}]'
            ),
            description=(
                'JSON list of robot configurations. Each entry must specify '
                'name, type, controller_mode, and goal_state ([x, y]). '
                'twist_input and reference are optional (reference is forced '
                'to "gbpl" since CBS requires the global body planner). The '
                'default is an eight-robot octagon-swap scenario on big_flat.'),
        ),
        # Validation + munging runs first so launch_quad_plan and launch_cbs
        # see the canonical robot_configs string.
        OpaqueFunction(function=_validate_and_munge),
        OpaqueFunction(function=launch_quad_plan),
        OpaqueFunction(function=launch_cbs),
    ])
