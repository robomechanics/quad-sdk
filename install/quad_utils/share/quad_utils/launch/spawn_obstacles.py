from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import json
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
    compliant_cord_sdf = PathJoinSubstitution([FindPackageShare('underbrush_description'), 'models', 'underbrush_description', 'compliant_cord_ros2.sdf.xacro'])
    compliant_beam_sdf = PathJoinSubstitution([FindPackageShare('underbrush_description'), 'models', 'underbrush_description','compliant_beam_horizontal.sdf.xacro'])
    box = PathJoinSubstitution([FindPackageShare('objects_description'), 'models', 'box','sdf', 'box.sdf'])

    scenario_config = LaunchConfiguration('scenario').perform(context)
    obstacles_config_raw = LaunchConfiguration('obstacles').perform(context)

    nodes = []
    
    # Add Scenario Configurations to Launch Order
    if scenario_config == 'underbrush':
        print("Handling Underbrush Scenario")
        nodes.extend([
            spawn_obstacle('underbrush',  "-x 0.60 -y -0.50 -z 0.20", compliant_cord_sdf, context),
            spawn_obstacle('underbrush1', "-x 1.03 -y -0.50 -z 0.12", compliant_cord_sdf, context),
            spawn_obstacle('underbrush2', "-x 1.03 -y -0.50 -z 0.23", compliant_cord_sdf, context),
            spawn_obstacle('underbrush3', "-x 1.36 -y -0.50 -z 0.15", compliant_cord_sdf, context),
        ])
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