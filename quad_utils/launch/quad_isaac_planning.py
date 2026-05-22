r"""Isaac-side ROS planning stack: global body planner + local planner + BFE.

Launch this LAST -- after the Isaac bridge (sim) and the bringup launch
(robot_driver + terrain + rsp). This is a thin wrapper over the standard
quad-sdk planning.py that pre-sets `use_sim_time:=true` and routes the
canonical Isaac defaults so the planner stack picks up the bridge's
/clock and the bringup's filtered terrain map.

What this brings up (via planning.py under <namespace>):
    - global_body_planner_node
    - local_planner_node
    - body_force_estimator_node

Typical launch order:
    Terminal 1 -- Isaac sim + bridge
        bash <path>/run_isaac_bridge.sh --robot go2 --scene flat
    Terminal 2 -- bringup (terrain + rsp + driver)
        ros2 launch quad_utils quad_isaac_bringup.py \
            namespace:=robot_1 robot_type:=go2 controller:=inverse_dynamics
    Terminal 3 -- this launch (planning)
        ros2 launch quad_utils quad_isaac_planning.py \
            namespace:=robot_1 robot_type:=go2

The world arg matters for resolving the .sdf the planner reads for map
extents; it does NOT cause any Gazebo to spawn. Defaults to flat.sdf to
match `run_isaac_bridge.sh --scene flat`. For an underbrush scene the
planner doesn't currently have a dedicated terrain definition -- the
filtered map fed in by the bringup is the source of truth.

Supported robot_type values (must match an entry in
isaac_plugins/scripts/isaac_bridge.py ROBOT_REGISTRY): spirit, go2.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the Isaac-side planning launch: gbpl + local_planner + BFE."""
    args = [
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('robot_type', default_value='spirit'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world', default_value='flat.sdf',
            description=(
                'World name used by planning.py to resolve the terrain '
                'extents. Does not spawn any simulator.'
            ),
        ),
        # planning.py gates global_body_planner_node behind
        # `reference==gbpl` (default is 'twist' for manual driving).
        # The Isaac demos lean autonomous, so default to gbpl here. Set
        # reference:=twist to revert to keyboard/joy-driven cmd_vel.
        DeclareLaunchArgument(
            'reference', default_value='gbpl',
            description=(
                'Reference source for the local planner. '
                'gbpl = run global_body_planner; '
                'twist = drive from /<ns>/cmd_vel (skip gbpl).'
            ),
        ),
    ]

    # planning.py spawns its nodes (global_body_planner_node,
    # local_planner_node, body_force_estimator_node) with RELATIVE
    # remappings like ('start_state', 'state/ground_truth'). Those
    # resolve to /<ns>/state/ground_truth only if a parent launch
    # pushed the namespace -- same pattern as robot_driver.py's
    # sim_group. In the gazebo flow it's wrapped inside
    # robot_bringup.py's PushRosNamespace; here we wrap it ourselves.
    planning_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('quad_utils'),
                    'launch', 'planning.py',
                ])
            ),
            launch_arguments={
                'robot_type': LaunchConfiguration('robot_type'),
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'world': LaunchConfiguration('world'),
                'reference': LaunchConfiguration('reference'),
            }.items(),
        ),
    ])

    return LaunchDescription(args + [planning_group])
