"""Forward Quad-SDK teleop to the Go2's stock (Unitree sport mode) controller.

Runs on the robot, in the Quad-SDK container after init_robot.sh (domain 42 +
ROBOT_MCU_IFACE). go2_sport_bridge_node subscribes /<namespace>/cmd_vel and
/<namespace>/control/mode and calls SportClient::Move / RecoveryStand /
StandDown / Damp. The cmd_vel low-pass and scale are read from
robot_driver.yaml so the stock controller gets the same filtered command a
learned policy would; the clip box defaults to the policies' trained bounds.

Requires sport mode to be active: do not start robot_driver (it releases
sport mode), and reboot the Go2 if it already ran.

  ros2 launch quad_utils go2_sport_bridge.py [namespace:=robot_1] [select_mode:=]
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_bridge(context, *args, **kwargs):
    robot_driver_share = FindPackageShare('robot_driver').perform(context)
    with open(os.path.join(robot_driver_share, 'config', 'robot_driver.yaml')) as f:
        rd = yaml.safe_load(f)['/**/robot_driver']['ros__parameters']['robot_driver']

    return [
        Node(
            package='robot_driver',
            executable='go2_sport_bridge_node',
            name='go2_sport_bridge',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'cmd_vel_filter_const': float(rd['cmd_vel_filter_const']),
                'cmd_vel_scale': float(rd['cmd_vel_scale']),
                'select_mode': LaunchConfiguration('select_mode'),
            }],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('select_mode', default_value='',
                              description='Motion service to select if none is active; '
                                          'empty = refuse to start'),
        OpaqueFunction(function=launch_bridge),
    ])
