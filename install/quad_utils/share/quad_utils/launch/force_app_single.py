from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_force_harmonic_bridge(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    gz_wrench_persistent  = f"/world/default/wrench/persistent"
    gz_wrench_clear       = f"/world/default/wrench/clear"
    ros_wrench_persistent = f"/{namespace}/wrench/persistent"
    ros_wrench_clear      = f"/{namespace}/wrench/clear"

    wrench_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wrench_bridge',
        # output='screen',
        arguments=[
            f"{gz_wrench_persistent}@ros_gz_interfaces/msg/EntityWrench@gz.msgs.EntityWrench",
            f"{gz_wrench_clear}@ros_gz_interfaces/msg/Entity@gz.msgs.Entity",
        ],
        remappings=[
            (gz_wrench_persistent, ros_wrench_persistent),
            (gz_wrench_clear,      ros_wrench_clear),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    return [wrench_bridge]
        
def launch_force_applicator(context, *args, **kwargs):
    force_applicator_pkg = FindPackageShare('force_applicator') 
    force_applicator_param_file = PathJoinSubstitution([force_applicator_pkg, 'config', 'force_applicator.yaml'])
    force_applicator_topics_file = PathJoinSubstitution([force_applicator_pkg, 'config', 'force_applicator_topics.yaml'])
    return [Node(
                package="force_applicator",
                executable="force_applicator_node",
                name="force_applicator",
                namespace=LaunchConfiguration('namespace'),
                # output="screen",
                parameters=[force_applicator_param_file,
                            force_applicator_topics_file, 
                    {'use_sim_time' : LaunchConfiguration('use_sim_time'),
                     'world' : LaunchConfiguration('world'),
                     'robot_type' : LaunchConfiguration('robot_type'),
                     'robot_ns' : LaunchConfiguration('namespace'),
                     'mode': LaunchConfiguration('mode'),
                     'force_mode' : LaunchConfiguration('force_mode'),
                     'link' : LaunchConfiguration('link')
                }]
            )
      ]
        
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="flat.sdf", description = "SDF world file to load into simulation"),
        DeclareLaunchArgument('robot_type', default_value = 'go2', description='Robot type'),
        DeclareLaunchArgument('namespace', default_value = 'robot_1', description='Robot namespace'),
        DeclareLaunchArgument("mode", default_value = 'periodic', 
            description = "Method for applying forces: 'single' to apply once, periodic' to apply at set time intervals, or 'distance' to apply based on distance traveled." ),
        DeclareLaunchArgument("force_mode", default_value = "random", 
            description = "Source for force magnitudes: 'yaml' to load predefined wrenches from a YAML file, or 'random' to sample from a distribution."),
        DeclareLaunchArgument("link", default_value = "robot_1::body",
            description = "Name of the robot link on which to apply the force."),
        DeclareLaunchArgument("use_sim_time", default_value = "True"),
        # OpaqueFunction(function=launch_force_harmonic_bridge),
        OpaqueFunction(function=launch_force_applicator),
    ])