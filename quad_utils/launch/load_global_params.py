import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LoadComposableNodes
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node

def generate_launch_description():
    # Define arguments
    robot_type_arg = DeclareLaunchArgument(
        'robot_type', default_value='spirit',
        description='Type of robot (spirit, spirit_rotors, or a1)')
    
    load_robot_params_arg = DeclareLaunchArgument(
        'load_robot_params', default_value='false',
        description='Whether to load robot parameters')

    # Load YAML parameter files
    config_dir = get_package_share_directory('quad_utils')
    yaml_files = [
        ('global_body_planner', 'global_body_planner.yaml'),
        ('local_planner', 'local_planner.yaml'),
        ('nmpc_controller', 'nmpc_controller.yaml'),
        ('robot_driver', 'robot_driver.yaml'),
        ('body_force_estimator', 'body_force_estimator.yaml'),
        ('quad_utils', 'config/terrain_map_publisher.yaml'),
        ('quad_utils', 'config/grid_map_visualization.yaml'),
        ('quad_utils', 'config/remote_heartbeat.yaml'),
        ('quad_utils', 'config/teleop_twist_keyboard.yaml'),
        ('quad_utils', 'config/rviz_interface.yaml'),
        ('quad_utils', 'config/trajectory_publisher.yaml'),
        ('quad_utils', 'config/topics_global.yaml')
    ]

    param_load_actions = [
        Node(
            package='rclcpp',
            executable='parameter_loader',
            arguments=[os.path.join(get_package_share_directory(pkg), file)],
            output='screen'
        ) for pkg, file in yaml_files
    ]

    # Load robot description based on robot_type argument
    spirit_group = GroupAction([
        Node(
            package='rclcpp',
            executable='parameter_loader',
            arguments=[os.path.join(get_package_share_directory('spirit_description'), 'sdf_mesh/spirit.sdf')],
            name='robot_description_sdf'
        ),
        Node(
            package='rclcpp',
            executable='parameter_loader',
            arguments=[os.path.join(get_package_share_directory('spirit_description'), 'urdf/spirit.urdf')],
            name='robot_description'
        )
    ])

    a1_group = GroupAction([
        Node(
            package='rclcpp',
            executable='parameter_loader',
            arguments=[os.path.join(get_package_share_directory('a1_description'), 'sdf_mesh/a1.sdf')],
            name='robot_description_sdf'
        ),
        Node(
            package='rclcpp',
            executable='parameter_loader',
            arguments=[os.path.join(get_package_share_directory('a1_description'), 'urdf/a1.urdf')],
            name='robot_description'
        )
    ])

    # Conditionally include robot parameter loading launch file
    load_robot_params_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('quad_utils'), 'launch/load_robot_params.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('load_robot_params'))
    )

    return LaunchDescription([
        robot_type_arg,
        load_robot_params_arg,
        *param_load_actions,
        GroupAction([
            IfCondition(LaunchConfiguration('robot_type') == 'spirit'),
            spirit_group,
        ]),
        GroupAction([
            IfCondition(LaunchConfiguration('robot_type') == 'a1'),
            a1_group,
        ]),
        load_robot_params_include,
    ])
