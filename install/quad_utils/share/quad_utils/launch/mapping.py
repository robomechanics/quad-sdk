from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare all launch arguments
    declared_arguments = [
        DeclareLaunchArgument('input_type', default_value='grid',
            description='Input used to generate terrain data'),
        DeclareLaunchArgument('frame_id_mesh_loaded', default_value='map'),
        DeclareLaunchArgument('grid_map_layer_name', default_value='z'),
        DeclareLaunchArgument('grid_map_resolution', default_value='0.05'),
        DeclareLaunchArgument('latch_grid_map_pub', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('world', default_value='step_20cm.sdf'),
        DeclareLaunchArgument('use_sim_time', default_value='true')
    ]

    # Node for terrain_map_publisher if input_type == "grid"
    # Launch the node to generate simple terrain from csv or compute in node

    # terrain_map_group = GroupAction(
    #     actions=[
    #         Node(
    #             package='quad_utils',
    #             executable='terrain_map_publisher_node',
    #             name='terrain_map_publisher',
    #             output='screen'
    #         )
    #     ],
    #     condition=IfCondition(PythonExpression(["'", LaunchConfiguration('input_type'), "' == 'grid'"]))
    # )

    # Node for mesh_to_grid_map_node if input_type == "mesh"
    # Launch the node to generate a mesh
    mesh_to_grid_group = GroupAction(
        actions=[
            Node(
                package='quad_utils',
                executable='mesh_to_grid_map_node',
                name='mesh_to_grid_map_node',
                # output='screen',
                parameters=[{
                    'frame_id_mesh_loaded': LaunchConfiguration('frame_id_mesh_loaded'),
                    'grid_map_resolution': LaunchConfiguration('grid_map_resolution'),
                    'layer_name': LaunchConfiguration('grid_map_layer_name'),
                    'latch_grid_map_pub': LaunchConfiguration('latch_grid_map_pub'),
                    'verbose': LaunchConfiguration('verbose'),
                    'world': LaunchConfiguration('world'),
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }]
            )
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('input_type'), "' == 'mesh'"]))
    )

    # Launch the grid map visualizer
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

    # Launch the grid map filters demo node
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
            mesh_to_grid_group,
            grid_map_visualization,
            grid_map_filter_node,
            static_tf
        ]
    )
