"""Bring up a LiDAR point cloud source under <namespace>/perception/.

Two sources are selectable via the 'lidar' argument:

  livox   (default) - Livox Mid-360 over ethernet, driven by livox_ros_driver2.
  unitree           - the Go2's built-in L1, bridged off the Unitree SDK2 DDS
                      domain by quad_perception's unitree_pointcloud_bridge.

Both publish the same relative topic ('output_topic'), so downstream consumers
do not care which one is running.

Note: the livox option needs livox_ros_driver2, which ships as a submodule under
external/ and is prepared by external/setup_deps.sh (it writes the ROS 2
package.xml and the colcon.pkg carrying its required cmake args). It then builds
with the rest of the workspace. Use lidar:=unitree if the submodule is not
checked out.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar = DeclareLaunchArgument(
        'lidar', default_value='livox', choices=['livox', 'unitree'],
        description='Which LiDAR to bring up: livox (Mid-360) or unitree (built-in L1).')
    namespace = DeclareLaunchArgument(
        'namespace', default_value='robot_1',
        description='Robot namespace (source publishes under <ns>/perception/)')
    output_topic = DeclareLaunchArgument(
        'output_topic', default_value='cloud_deskewed',
        description=('Relative topic the selected LiDAR publishes on. Resolves '
                     'under <namespace>/perception/ due to the pushed namespace.'))
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    # --- Unitree L1 (built-in) -----------------------------------------------
    network_interface = DeclareLaunchArgument(
        'network_interface', default_value='',
        description=('Network interface for the Unitree DDS MCU domain. Empty '
                     'falls back to the ROBOT_MCU_IFACE env var, then eth0.'))
    input_dds_topic = DeclareLaunchArgument(
        'input_dds_topic', default_value='rt/utlidar/cloud_deskewed',
        description='DDS topic to subscribe from on the Unitree SDK2 channel.')
    frame_id = DeclareLaunchArgument(
        'frame_id', default_value='',
        description=('Override the frame_id stamped on outgoing messages. '
                     'Empty passes the DDS frame through (odom for deskewed).'))

    unitree_bridge = Node(
        package='quad_perception',
        executable='unitree_pointcloud_bridge_node',
        name='unitree_pointcloud_bridge',
        output='screen',
        parameters=[{
            'network_interface': LaunchConfiguration('network_interface'),
            'input_dds_topic': LaunchConfiguration('input_dds_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # --- Livox Mid-360 -------------------------------------------------------
    livox_config = DeclareLaunchArgument(
        'livox_config', default_value=PathJoinSubstitution([
            FindPackageShare('quad_utils'), 'config', 'mid360_config.json']),
        description=('Livox user config JSON. The host_net_info addresses in it '
                     'must match this machine, and lidar_configs the sensor.'))
    livox_frame_id = DeclareLaunchArgument(
        'livox_frame_id', default_value='livox_frame',
        description='frame_id stamped on Mid-360 clouds.')
    livox_publish_freq = DeclareLaunchArgument(
        'livox_publish_freq', default_value='10.0',
        description='Cloud publish rate in Hz (5.0, 10.0, 20.0, 50.0, ...).')

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            # 0 -> sensor_msgs/PointCloud2, so downstream matches the L1 bridge.
            'xfer_format': 0,
            # 0 -> one topic for all lidars rather than one topic per device.
            'multi_topic': 0,
            'data_src': 0,          # 0 -> live sensor (not a replayed lvx file)
            'output_data_type': 0,
            'publish_freq': LaunchConfiguration('livox_publish_freq'),
            'frame_id': LaunchConfiguration('livox_frame_id'),
            'user_config_path': LaunchConfiguration('livox_config'),
            'cmdline_input_bd_code': 'livox0000000001',
            'lvx_file_path': '',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        # The driver publishes 'livox/lidar'; land it on the shared output topic.
        remappings=[('livox/lidar', LaunchConfiguration('output_topic'))],
    )

    # --- Point cloud -> grid_map ---------------------------------------------
    gridmap = DeclareLaunchArgument(
        'gridmap', default_value='true',
        description='Turn the cloud into a grid_map elevation layer.')
    mapping_backend = DeclareLaunchArgument(
        'mapping_backend', default_value='octomap', choices=['octomap', 'custom'],
        description=(
            'octomap (default) - octomap_server does probabilistic log-odds '
            'voxel fusion and sensor-origin ray-traced free-space clearing, so '
            'noise self-corrects instead of accumulating as permanent spikes, '
            'and real 3D structure keeps obstacles (e.g. a table) from being '
            'read as a terrain step; quad_perception/octomap_to_gridmap then '
            'projects it down to the same grid_map contract below. '
            'custom - the original quad_perception/pointcloud_to_gridmap '
            'node (PCL ground segmentation, naive max-height rasterizing); '
            'kept as a lighter-weight fallback with no TF requirement.'))
    gridmap_topic = DeclareLaunchArgument(
        'gridmap_topic', default_value='/mapping/terrain_map_raw',
        description=('Where the elevation grid_map is published. This is the '
                     'input_topic the terrain filter chain in '
                     'config/filter_chain.yaml subscribes to.'))
    gridmap_resolution = DeclareLaunchArgument(
        'gridmap_resolution', default_value='0.05',
        description='Grid cell size in metres.')
    gridmap_length = DeclareLaunchArgument(
        'gridmap_length', default_value='10.0',
        description='Side length of the (square) grid in metres. custom backend only.')
    map_frame = DeclareLaunchArgument(
        'map_frame', default_value='',
        description=('Fixed frame to accumulate the grid in. custom backend only: '
                     'empty builds it in the cloud frame, which needs no TF tree; '
                     'set this once a transform to a fixed frame is being published. '
                     'octomap backend always needs a real fixed frame, see '
                     'octomap_frame_id.'))
    octomap_frame_id = DeclareLaunchArgument(
        'octomap_frame_id', default_value='odom',
        description=('Fixed frame octomap_server accumulates in. Needs a real TF '
                     'tree from the cloud frame to this one (unlike the custom '
                     'backend, octomap_server cannot run frameless).'))
    base_frame_id = DeclareLaunchArgument(
        'base_frame_id', default_value='body',
        description='Robot base frame, used by octomap_server for ground filtering.')
    ground_min_z = DeclareLaunchArgument(
        'ground_min_z', default_value='-0.5',
        description=('Lower bound (m, in octomap_frame_id) of the band '
                     'octomap_to_gridmap reads as terrain height.'))
    ground_max_z = DeclareLaunchArgument(
        'ground_max_z', default_value='0.5',
        description=('Upper bound (m, in octomap_frame_id) of the band '
                     'octomap_to_gridmap reads as terrain height; obstacles above '
                     'this (tables, shelves) are excluded from the height layer.'))
    visualize = DeclareLaunchArgument(
        'visualize', default_value='false',
        description=('Run grid_map_visualization, which republishes the grid as '
                     'a PointCloud2 and OccupancyGrid for plain RViz.'))

    custom_gridmap_node = Node(
        package='quad_perception',
        executable='pointcloud_to_gridmap_node',
        name='pointcloud_to_gridmap',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('output_topic'),
            'output_topic': LaunchConfiguration('gridmap_topic'),
            'resolution': LaunchConfiguration('gridmap_resolution'),
            'length_x': LaunchConfiguration('gridmap_length'),
            'length_y': LaunchConfiguration('gridmap_length'),
            'map_frame': LaunchConfiguration('map_frame'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('gridmap'), "' == 'true' and '",
             LaunchConfiguration('mapping_backend'), "' == 'custom'"])),
    )

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': LaunchConfiguration('octomap_frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'resolution': LaunchConfiguration('gridmap_resolution'),
            # Ground stays IN the octree; octomap_to_gridmap's ground band
            # decides what counts as terrain at projection time instead.
            'filter_ground_plane': False,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[('cloud_in', LaunchConfiguration('output_topic'))],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('gridmap'), "' == 'true' and '",
             LaunchConfiguration('mapping_backend'), "' == 'octomap'"])),
    )

    octomap_gridmap_node = Node(
        package='quad_perception',
        executable='octomap_to_gridmap_node',
        name='octomap_to_gridmap',
        output='screen',
        parameters=[{
            # Relative: octomap_server_node also publishes 'octomap_full' as a
            # relative name, so both need the same pushed namespace to agree
            # on where it actually lands. gridmap_topic stays absolute, same
            # as custom_gridmap_node's output.
            'input_topic': 'octomap_full',
            'output_topic': LaunchConfiguration('gridmap_topic'),
            'ground_min_z': LaunchConfiguration('ground_min_z'),
            'ground_max_z': LaunchConfiguration('ground_max_z'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('gridmap'), "' == 'true' and '",
             LaunchConfiguration('mapping_backend'), "' == 'octomap'"])),
    )

    # --- LiDAR mounting transform --------------------------------------------
    # body -> livox_frame. quad-sdk's estimator supplies map -> body, so with
    # this published the grid can accumulate in a fixed frame (map_frame:=map).
    #
    # Defaults are the standard Go2 EDU bracket: LiDAR centred on the mounting
    # track, upright and facing forward, so only the forward and vertical
    # offsets are non-zero. They are the midpoints of the quoted mount ranges
    # (x 0.15-0.19, z 0.11-0.13), measured from the geometric body centre to the
    # optical centre. Measure your own build if footstep placement matters --
    # the +/-20mm on x lands directly in the terrain the footstep planner reads.
    publish_lidar_tf = DeclareLaunchArgument(
        'publish_lidar_tf', default_value='true',
        description='Publish the static parent -> LiDAR frame transform.')
    lidar_parent_frame = DeclareLaunchArgument(
        'lidar_parent_frame', default_value='body',
        description='Parent frame the LiDAR is rigidly mounted to.')
    lidar_x = DeclareLaunchArgument(
        'lidar_x', default_value='0.17',
        description='Forward offset, body centre to LiDAR optical centre (m).')
    lidar_y = DeclareLaunchArgument(
        'lidar_y', default_value='0.0',
        description='Lateral offset; zero on the centred EDU mounting track.')
    lidar_z = DeclareLaunchArgument(
        'lidar_z', default_value='0.12',
        description='Vertical offset above the body centre (m).')
    lidar_roll = DeclareLaunchArgument('lidar_roll', default_value='0.0')
    lidar_pitch = DeclareLaunchArgument('lidar_pitch', default_value='0.0')
    lidar_yaw = DeclareLaunchArgument('lidar_yaw', default_value='0.0')

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_static_tf',
        arguments=[
            '--x', LaunchConfiguration('lidar_x'),
            '--y', LaunchConfiguration('lidar_y'),
            '--z', LaunchConfiguration('lidar_z'),
            '--roll', LaunchConfiguration('lidar_roll'),
            '--pitch', LaunchConfiguration('lidar_pitch'),
            '--yaw', LaunchConfiguration('lidar_yaw'),
            '--frame-id', LaunchConfiguration('lidar_parent_frame'),
            '--child-frame-id', LaunchConfiguration('livox_frame_id'),
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('publish_lidar_tf')),
    )

    gridmap_visualization = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('quad_utils'), 'config',
                'lidar_gridmap_visualization.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(LaunchConfiguration('visualize')),
    )

    return LaunchDescription([
        lidar,
        namespace,
        output_topic,
        use_sim_time,
        gridmap,
        mapping_backend,
        gridmap_topic,
        gridmap_resolution,
        gridmap_length,
        map_frame,
        octomap_frame_id,
        base_frame_id,
        ground_min_z,
        ground_max_z,
        visualize,
        publish_lidar_tf,
        lidar_parent_frame,
        lidar_x,
        lidar_y,
        lidar_z,
        lidar_roll,
        lidar_pitch,
        lidar_yaw,
        network_interface,
        input_dds_topic,
        frame_id,
        livox_config,
        livox_frame_id,
        livox_publish_freq,
        GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),
            PushRosNamespace('perception'),
            GroupAction(
                [livox_driver],
                condition=IfCondition(PythonExpression(
                    ["'", LaunchConfiguration('lidar'), "' == 'livox'"])),
            ),
            GroupAction(
                [unitree_bridge],
                condition=IfCondition(PythonExpression(
                    ["'", LaunchConfiguration('lidar'), "' == 'unitree'"])),
            ),
            # Inside the namespace so it picks up the relative cloud topic; its
            # own output topic is absolute and so escapes the namespace.
            custom_gridmap_node,
            octomap_server_node,
            octomap_gridmap_node,
            lidar_tf,
        ]),
        # Left unnamespaced: the yaml addresses the node as
        # /grid_map_visualization, and pushing a namespace would orphan it.
        gridmap_visualization,
    ])
