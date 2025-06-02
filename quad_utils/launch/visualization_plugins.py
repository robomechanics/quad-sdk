from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
import os


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('robot_type', default_value='spirit'),
        DeclareLaunchArgument('robot_description', default_value=''),
    ]

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    urdf = LaunchConfiguration('robot_description')
    quad_utils_share = FindPackageShare('quad_utils')
    rviz_yaml_file = ParameterFile(PathJoinSubstitution([quad_utils_share, 'config', 'rviz_visualization.yaml']))

    # Group 1: trajectory state publisher
    trajectory_state_publisher = GroupAction([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='trajectory',
            parameters=[
                {'robot_description': ParameterValue(urdf, value_type=str),
                 'frame_prefix': [namespace, '_trajectory','/']}
            ],
            remappings=[
                ('/joint_states', '/visualization/joint_states')
            ],
            # output='screen'
        )
    ])

    # Group 2: ground truth state publisher
    ground_truth_state_publisher = GroupAction([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='ground_truth',
            parameters=[
                {'robot_description': ParameterValue(urdf, value_type=str),
                 'frame_prefix': [namespace, '_ground_truth', '/']}
            ],
            remappings=[
                ('/joint_states', '/visualization/joint_states')
            ],
            # output='screen'
        )
    ])

    # RViz interface node
    rviz_interface = Node(
        package='quad_utils',
        executable='rviz_interface_node',
        name='rviz_interface',
        parameters=[
        rviz_yaml_file,
        {'tf_prefix': namespace}
    ],
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        trajectory_state_publisher,
        ground_truth_state_publisher,
        rviz_interface
    ])
