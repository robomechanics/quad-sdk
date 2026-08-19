import os
import struct

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _read_ply_vertices(path):
    """Vertices of a binary little-endian PLY -- the format the beam meshes ship in."""
    with open(path, 'rb') as f:
        data = f.read()

    for terminator in (b'end_header\r\n', b'end_header\n'):
        if terminator in data:
            offset = data.index(terminator) + len(terminator)
            break
    else:
        raise ValueError('no PLY header terminator')

    header = data[:offset].replace(b'\r\n', b'\n')
    if b'binary_little_endian' not in header:
        raise ValueError('not a binary little-endian PLY')
    n_vertices = int([l for l in header.split(b'\n')
                      if l.startswith(b'element vertex')][0].split()[-1])

    return [struct.unpack_from('<3f', data, offset + i * 12) for i in range(n_vertices)]


def _beam_geometry(world, resolution, erosion_radius):
    """Marker geometry for the loaded terrain mesh, or None if it is not a beam.

    Read from the same .ply mesh_to_grid_map_node loads, so the markers cannot
    drift from the terrain they annotate -- which is what happened when they
    were hardcoded to beam_world_10cm and the 15 cm beam went in.

    Returns (x_min, x_max, waist_half_width, corridor_half_width).
    """
    base = world[:-4] if world.endswith('.sdf') else world
    ply = os.path.join(FindPackageShare('quad_sim_scripts').find('quad_sim_scripts'),
                       'models', base, 'meshes', base + '.ply')
    vertices = _read_ply_vertices(ply)

    xs = [v[0] for v in vertices]
    half_widths = sorted({round(abs(v[1]), 6) for v in vertices if abs(v[1]) > 1e-6})

    # A beam world is a dogbone: wide platforms either end, a narrow waist in
    # the middle, cut by CAD so only a handful of distinct |y| values exist.
    # Bail out on anything else (flat, step_*, the dense rough_* heightfields)
    # rather than drawing a centreline across terrain that has no beam.
    if len(half_widths) < 2 or len(half_widths) > 8:
        return None
    waist, platform = half_widths[0], half_widths[-1]
    if waist > 0.5 * platform:
        return None

    # Erosion removes whole cells and the cell centres do not line up with the
    # beam edge, so the corridor is NOT waist - erosion_radius. See the long
    # note in filter_chain.yaml: a 0.6096 m wide mesh at a 5 mm pitch is an EVEN
    # cell count, which puts centres half a pitch off the centreline. Measured
    # against the published map for the 10/12/15/17 cm beams: this reproduces
    # 0.0325 / 0.0425 / 0.0575 / 0.0675 exactly.
    n_eroded = int(erosion_radius / resolution)  # radius convention: N*pitch + 1 mm
    n_cells = round(2.0 * platform / resolution)
    phase = resolution / 2.0 if n_cells % 2 == 0 else 0.0
    outermost_centre_on_beam = phase + int((waist - phase) / resolution) * resolution
    corridor = outermost_centre_on_beam - n_eroded * resolution

    return min(xs), max(xs), waist, corridor


def _erosion_radius():
    """Erosion radius from filter_chain.yaml, so retuning it moves the marker."""
    cfg = os.path.join(FindPackageShare('quad_utils').find('quad_utils'),
                       'config', 'filter_chain.yaml')
    with open(cfg) as f:
        params = yaml.safe_load(f)
    filters = params['mapping']['grid_map_filters']['ros__parameters']['filters']
    for f_cfg in filters.values():
        if f_cfg.get('name') == 'traversability_erode_boundary':
            return float(f_cfg['params']['radius'])
    raise RuntimeError('traversability_erode_boundary not found in filter_chain.yaml')


def launch_beam_marker(context, *args, **kwargs):
    """Beam centreline / edge / corridor markers for RViz.

    The Gazebo SDF stripe is invisible to RViz (RViz never loads the world
    SDF), so the same geometry is republished as markers in the map frame.
    """
    if LaunchConfiguration('beam_markers').perform(context).lower() == 'false':
        return []

    world = LaunchConfiguration('world').perform(context)
    resolution = float(LaunchConfiguration('grid_map_resolution').perform(context))
    try:
        geometry = _beam_geometry(world, resolution, _erosion_radius())
    except (OSError, ValueError, IndexError) as exc:
        print(f"[mapping.py] beam markers disabled: cannot read mesh for "
              f"'{world}' ({exc})")
        return []
    if geometry is None:
        print(f"[mapping.py] beam markers disabled: '{world}' has no beam waist")
        return []

    x_min, x_max, waist, corridor = geometry
    print(f"[mapping.py] beam markers for '{world}': waist +/-{waist:.4f} m, "
          f"foothold corridor +/-{corridor:.4f} m")

    return [
        Node(
            package='quad_utils',
            executable='beam_centerline_marker.py',
            name='beam_centerline_marker',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id_mesh_loaded'),
                # Same placement the terrain mesh gets, so the line lands on
                # the beam rather than at the map origin.
                'mesh_pose': LaunchConfiguration('mesh_pose'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'x_min': x_min,
                'x_max': x_max,
                'edge_half_width': waist,
                'corridor_half_width': corridor,
            }],
        )
    ]


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
        DeclareLaunchArgument('beam_markers', default_value='auto',
            description='RViz beam centreline/edge/corridor markers. "auto" '
                        'draws them only for worlds whose mesh has a beam '
                        'waist; "false" never does.'),
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
            OpaqueFunction(function=launch_beam_marker),
            grid_map_visualization,
            grid_map_filter_node,
            static_tf
        ]
    )
