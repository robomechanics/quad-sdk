"""PS3 joystick teleop for stock-controller (Unitree sport mode) trials.

Starts exactly the joy_node + teleop_twist_joy pair that planning.py starts for
twist_input:=joy (rml-ps3-holonomic config, same deadzone, autorepeat and
scaling), in the same namespace, so it publishes the same
/<namespace>/cmd_vel and /<namespace>/control/mode as in a learned-policy run.
go2_sport_bridge.py on the robot forwards them to the sport controller.

  ros2 launch quad_utils go2_sport_joy.py [namespace:=robot_1]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('joy_config', default_value='rml-ps3-holonomic'),
        DeclareLaunchArgument('joy_dev', default_value='0'),
        GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('teleop_twist_joy'), 'launch', 'teleop-launch.py'
                ])),
                launch_arguments={
                    'joy_config': LaunchConfiguration('joy_config'),
                    'joy_dev': LaunchConfiguration('joy_dev'),
                }.items()
            ),
        ]),
    ])
