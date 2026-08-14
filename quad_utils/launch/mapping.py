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
        # 5 mm, not 1 cm: a 10 cm beam is only 10 cells wide at 1 cm, so the
        # beam edge falling mid-cell aliases the traversable corridor by a full
        # cell and it visibly wanders along the beam. At 5 mm that ambiguity
        # halves. The erosion radius in filter_chain.yaml is specified in
        # metres, so the physical 1 cm erosion is unchanged by this.
        DeclareLaunchArgument('grid_map_resolution', default_value='0.005'),
        DeclareLaunchArgument('latch_grid_map_pub', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('world', default_value='step_20cm.sdf'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('track_frame', default_value='',
            description='TF frame the terrain mesh follows (e.g. a mocap rigid '
                        'body). Empty places the mesh statically at mesh_pose.'),
        DeclareLaunchArgument('track_rate', default_value='5.0',
            description='Hz at which the tracked mesh pose is polled'),
        DeclareLaunchArgument('mesh_pose', default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
            description='Static mesh placement [x y z roll pitch yaw], used when not tracking'),
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
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'track_frame': LaunchConfiguration('track_frame'),
                    'track_rate': LaunchConfiguration('track_rate'),
                    'mesh_pose': LaunchConfiguration('mesh_pose'),
                }]
            )
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('input_type'), "' == 'mesh'"]))
    )

    # Beam centreline / edge / corridor markers for RViz. The Gazebo SDF
    # stripe is invisible to RViz (RViz never loads the world SDF), so the
    # same geometry is republished as markers in the map frame.
    beam_centerline_marker = Node(
        package='quad_utils',
        executable='beam_centerline_marker.py',
        name='beam_centerline_marker',
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id_mesh_loaded'),
            # Same placement the terrain mesh gets, so the line lands on the
            # beam rather than at the map origin.
            'mesh_pose': LaunchConfiguration('mesh_pose'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
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
            beam_centerline_marker,
            grid_map_visualization,
            grid_map_filter_node,
            static_tf
        ]
    )
