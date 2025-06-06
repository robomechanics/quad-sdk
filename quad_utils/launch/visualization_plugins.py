from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
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
        DeclareLaunchArgument('robot_urdf_path', description='Path to Robot URDF Xacro File')
    ]

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    urdf = LaunchConfiguration('robot_description')
    urdf_path = LaunchConfiguration('robot_urdf_path')
    quad_utils_share = FindPackageShare('quad_utils')
    rviz_yaml_file = ParameterFile(PathJoinSubstitution([quad_utils_share, 'config', 'rviz_visualization.yaml']))
    robot_type = LaunchConfiguration('robot_type')


    trajectory_state_urdf = Command([
        'xacro ', urdf_path,
        # ' prefix:=', namespace, '_trajectory_',
        ' robot_type:=', robot_type
    ])
    ground_truth_urdf = Command([
        'xacro ', urdf_path,
        # ' prefix:=', namespace, '_ground_truth_',
        ' robot_type:=', robot_type
    ])

    # Group 1: trajectory state publisher
    trajectory_state_publisher = GroupAction([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='trajectory',
            parameters=[
                {'robot_description': ParameterValue(trajectory_state_urdf, value_type=str),
                 'frame_prefix': [namespace, '_trajectory','/']
                 }
            ],
            remappings=[
                ('joint_states', 'visualization/joint_states')
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
                {'robot_description': ParameterValue(ground_truth_urdf, value_type=str),
                 'frame_prefix': [namespace, '_ground_truth', '/']
                 }
            ],
            remappings=[
                ('joint_states', 'visualization/joint_states')
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
