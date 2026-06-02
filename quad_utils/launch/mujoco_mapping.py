"""
mujoco_mapping.py launches nodes for generating, filtering, and visualizing terrain maps from 
Mujoco MJCF worlds in Ros2. 
"""
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Declare all launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'input_type', 
            default_value='grid',
            description='Input used to generate terrain data'
        ),
        DeclareLaunchArgument('frame_id_mjcf_loaded', default_value='map'),
        DeclareLaunchArgument('grid_map_layer_name', default_value='z'),
        DeclareLaunchArgument('grid_map_resolution', default_value='0.05'),
        DeclareLaunchArgument('latch_grid_map_pub', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('world', default_value='step_20cm.sdf'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    mjcf_to_grid_group = GroupAction(
        actions=[
            Node(
                package='quad_utils',
                executable='mjcf_to_grid_map_node',
                name='mjcf_to_grid_map_node',
                # output='screen',
                parameters=[{
                    'frame_id_mjcf_loaded': LaunchConfiguration('frame_id_mjcf_loaded'),
                    'grid_map_resolution': LaunchConfiguration('grid_map_resolution'),
                    'layer_name': LaunchConfiguration('grid_map_layer_name'),
                    'latch_grid_map_pub': LaunchConfiguration('latch_grid_map_pub'),
                    'verbose': LaunchConfiguration('verbose'),
                    'world': LaunchConfiguration('world'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }]
            )
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('input_type'), "' == 'mjcf'"])
        )
    )



    # Launches grid map visualizer. Stays the same, ensure use_sim_time matches mujoco clock source
    grid_map_visualization = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        # output='screen',
        # parameters=[ PathJoinSubstitution([
        #         FindPackageShare('quad_utils'),
        #         'config',
        #         'demo.yaml'
        #     ]),

        # ]
    )

    # Launch the grid map filters demo node. Check if the input and output topics match for mujoco
    grid_map_filter_node = Node(
        package='quad_utils',
        executable='grid_map_filters_demo',
        name='grid_map_filters',
        # output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('quad_utils'),
                'config',
                'filter_chain.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            # {'input_topic': '/mapping/terrain_map_raw'},
            # {'output_topic': '/mapping/terrain_map'},
        ],
        arguments=[],
        remappings=[],
    )

    # Verify frame names, mujoco ros2 bridge may publish under different conventions than gazebo.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{ 'use_sim_time': LaunchConfiguration('use_sim_time') }],
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        # output='screen',
        emulate_tty=True,
    )

    return LaunchDescription(
        declared_arguments + [
            # terrain_map_group,
            mjcf_to_grid_group,
            grid_map_visualization,
            grid_map_filter_node,
            static_tf
        ]
    )
