from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_ns =  LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    quad_perf_test_pkg = FindPackageShare('quad_perf_tests')
    cmd_vel_param_file = PathJoinSubstitution([quad_perf_test_pkg, 'config', 'cmd_vel_publisher.yaml'])
    cmd_vel_topics_file = PathJoinSubstitution([quad_perf_test_pkg, 'config', 'cmd_vel_publisher_topics.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='robot_1',
            description='Topic namespace to use'
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use /clock from simulation'),
         Node(
            package='quad_perf_tests',
            executable='cmd_vel_publisher_node',
            name='cmd_vel_publisher',
            namespace=robot_ns,
            # output='screen',
            parameters=[
                cmd_vel_param_file,
                cmd_vel_topics_file,
                {
                'namespace' : robot_ns,
                'use_sim_time' : ParameterValue(use_sim_time, value_type=bool),
            }]
        )
    ])