from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import json
import os
import xacro


def spawn_obstacle(name: str, init_pose: str, sdf, context):
    sdf_path = sdf.perform(context) if hasattr(sdf, "perform") else str(sdf)
    is_xacro = sdf_path.endswith(".xacro")
    create_args = [
        '-name', str(name),
        '-x', init_pose.split()[1],
        '-y', init_pose.split()[3],
        '-z', init_pose.split()[5],
    ]

    if is_xacro:
        # Expand the Xacro and inject model_name:=<name>, then feed to -string
        xacro_expanded = xacro.process_file(sdf_path, mappings={"model_name": name}).toxml()
        create_args += ['-string', xacro_expanded]
    else:
        # Use a normal SDF/URDF file path
        create_args += ['-file', sdf_path]

    return Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{name}',
        # output='screen',
        arguments=create_args,
    )

def parse_obstacles(context):
    # Select cord model via env var UNDERBRUSH_CORD_MODEL:
    #   'ros2' (default) — original thin whip cord (radius 5mm, mass 10g, damping 0.005)
    #   'isaac_matched'  — Isaac-vine-matched cord (radius 20mm, mass 60g,
    #                       damping 4.0, spring 25) so the v81 policy sees
    #                       contact patterns it was actually trained on.
    _cord_variant = os.environ.get('UNDERBRUSH_CORD_MODEL', 'ros2')
    _cord_file = 'compliant_cord_isaac_matched.sdf.xacro' if _cord_variant == 'isaac_matched' else 'compliant_cord_ros2.sdf.xacro'
    print(f"[spawn_obstacles] cord variant: {_cord_variant} ({_cord_file})")
    compliant_cord_sdf = PathJoinSubstitution([FindPackageShare('underbrush_description'), 'models', 'underbrush_description', _cord_file])
    # underbrush2 scenario pins the Isaac-matched cord regardless of env var
    # so a scenario:=underbrush2 launch is always the in-distribution vine
    # geometry for the v81 policy (radius 20mm, mass 60g, spring 25, damping 4).
    compliant_cord_isaac_sdf = PathJoinSubstitution([FindPackageShare('underbrush_description'), 'models', 'underbrush_description', 'compliant_cord_isaac_matched.sdf.xacro'])
    compliant_beam_sdf = PathJoinSubstitution([FindPackageShare('underbrush_description'), 'models', 'underbrush_description','compliant_beam_horizontal.sdf.xacro'])
    box = PathJoinSubstitution([FindPackageShare('objects_description'), 'models', 'box','sdf', 'box.sdf'])

    scenario_config = LaunchConfiguration('scenario').perform(context)
    obstacles_config_raw = LaunchConfiguration('obstacles').perform(context)

    nodes = []

    # Add Scenario Configurations to Launch Order
    if scenario_config == 'underbrush':
        print("Handling Underbrush Scenario (z-heights TEMPORARILY clamped to Isaac range [0.10, 0.18] for in-distribution v81 eval)")
        # TEMPORARY clamp — orig z values: 0.20, 0.12, 0.20, 0.15
        # Two of four were at 0.20 (above Isaac's height_range max of 0.18),
        # putting the v81 policy out of distribution. Lowered to 0.18 to
        # bring the interaction plane into Isaac training. Restore originals
        # if you want the pre-v81 obstacle layout back.
        nodes.extend([
            spawn_obstacle('underbrush',  "-x 0.60 -y -0.50 -z 0.18", compliant_cord_sdf, context),  # was 0.20
            spawn_obstacle('underbrush1', "-x 1.03 -y -0.50 -z 0.12", compliant_cord_sdf, context),
            spawn_obstacle('underbrush2', "-x 1.03 -y -0.50 -z 0.18", compliant_cord_sdf, context),  # was 0.20
            spawn_obstacle('underbrush3', "-x 1.36 -y -0.50 -z 0.15", compliant_cord_sdf, context),
        ])
    elif scenario_config == 'underbrush2':
        # Same spawn positions/orientations as the 'underbrush' scenario, but
        # each cord is the Isaac-vine-matched model (chunky 20mm radius, 60g
        # links, stiff spring, 0.9m anchor-to-anchor length). Z-heights
        # clamped to Isaac's height_range=(0.10, 0.18) — the original
        # underbrush z=0.20 spawns were ABOVE Isaac's max training height and
        # would put the v81 policy out of distribution. Use this scenario to
        # check whether the v81 transfer failure is purely a vine-parameter
        # mismatch.
        print("Handling Underbrush2 Scenario (Isaac-matched cords, z clamped to [0.10, 0.18])")
        nodes.extend([
            spawn_obstacle('underbrush',  "-x 0.60 -y -0.50 -z 0.18", compliant_cord_isaac_sdf, context),
            spawn_obstacle('underbrush1', "-x 1.03 -y -0.50 -z 0.12", compliant_cord_isaac_sdf, context),
            spawn_obstacle('underbrush2', "-x 1.03 -y -0.50 -z 0.18", compliant_cord_isaac_sdf, context),
            spawn_obstacle('underbrush3', "-x 1.36 -y -0.50 -z 0.15", compliant_cord_isaac_sdf, context),
        ])
    elif scenario_config == 'underbrush3':
        # Universal-joint variant (one 2-axis joint per segment, no carrier links).
        # vine_v90_single.sdf.xacro (3 co-located revolutes + 13 near-massless
        # carriers) blows up ODE on robot contact; set UNDERBRUSH_VINE_MODEL=single
        # to spawn it anyway for comparison.
        _vine_file = ('vine_v90_single.sdf.xacro' if os.environ.get('UNDERBRUSH_VINE_MODEL') == 'single'
                      else 'vine_v90_universal.sdf.xacro')
        vine_v90 = PathJoinSubstitution([FindPackageShare('underbrush_description'),
            'models', 'underbrush_description', _vine_file])
        print(f"Underbrush3: four v90 comparison vines ({_vine_file})")
        # This model is centered laterally, unlike the original cord model.
        # Retain its y=0 placement while using the established four x/z pairs.
        nodes.extend([
            spawn_obstacle('underbrush3_vine_0', "-x 0.60 -y 0.0 -z 0.18", vine_v90, context),
            spawn_obstacle('underbrush3_vine_1', "-x 1.03 -y 0.0 -z 0.12", vine_v90, context),
            spawn_obstacle('underbrush3_vine_2', "-x 1.03 -y 0.0 -z 0.18", vine_v90, context),
            spawn_obstacle('underbrush3_vine_3', "-x 1.36 -y 0.0 -z 0.15", vine_v90, context),
        ])

    elif scenario_config == 'hardware_v90_2205':
        vine_v90 = PathJoinSubstitution([FindPackageShare('underbrush_description'),
            'models', 'underbrush_description', 'vine_v90_single.sdf.xacro'])
        # Measured hardware spacing/heights. First-vine distance is provisionally
        # 0.60 m; material/tension remains the existing comparison fixture.
        for i, (x, z) in enumerate(((0.600, 0.127), (1.032, 0.229),
                                     (1.032, 0.127), (1.362, 0.152))):
            nodes.append(spawn_obstacle(f'hardware_v90_2205_vine_{i}',
                         f'-x {x} -y 0.0 -z {z}', vine_v90, context))

    # Add Custom Scenario Configurations Here

    # Add Obstacle Configurations to Launch Order
    try:
        obstacle_configs = json.loads(obstacles_config_raw)
    except json.JSONDecodeError as e:
        raise RuntimeError(f"Invalid JSON in 'obstacle_configs': {e}")
    
    if not isinstance(obstacle_configs, list):
        raise RuntimeError("'obstacles' must be a JSON list.")
    else:
        for i, config in enumerate(obstacle_configs):
            obstacle_name = config["name"]
            obstacle_pose = config["init_pose"]
            try:
                obstacle_sdf = PathJoinSubstitution([FindPackageShare('objects_description'), 'models', obstacle_name, f'{obstacle_name}.sdf'])
            except:
                raise RuntimeError(f'Obstacle SDF File Not Found')
            node_name = obstacle_name + "_" + str(i)
            nodes.append(spawn_obstacle(node_name, obstacle_pose, obstacle_sdf, context))
    return nodes


def generate_launch_description():
    # Defaults for SDFs (override via launch args if needed)

    scenario = DeclareLaunchArgument('scenario', default_value="None", description='Custom Obstacle Scenario to Spawn (e.g. "underbrush" or "None")')
    obstacles = DeclareLaunchArgument('obstacles', default_value='[]',
        description= 'A JSON List of obstalces.Each item:{"name": "<type>", "init_pose" : " -x ... -y ... -z ..."}')

    return LaunchDescription([scenario, obstacles, OpaqueFunction(function=parse_obstacles)])