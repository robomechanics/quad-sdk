from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    namespace = DeclareLaunchArgument(
        'namespace', default_value='robot_1',
        description='Robot namespace (bridge publishes under <ns>/perception/)')
    network_interface = DeclareLaunchArgument(
        'network_interface', default_value='',
        description=('Network interface for the Unitree DDS MCU domain. Empty '
                     'falls back to the ROBOT_MCU_IFACE env var, then eth0.'))
    input_dds_topic = DeclareLaunchArgument(
        'input_dds_topic', default_value='rt/utlidar/cloud_deskewed',
        description='DDS topic to subscribe from on the Unitree SDK2 channel.')
    output_topic = DeclareLaunchArgument(
        'output_topic', default_value='cloud_deskewed',
        description=('Relative topic to publish on. Resolves under '
                     '<namespace>/perception/ due to the pushed namespace.'))
    frame_id = DeclareLaunchArgument(
        'frame_id', default_value='',
        description=('Override the frame_id stamped on outgoing messages. '
                     'Empty passes the DDS frame through (odom for deskewed).'))
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    bridge_node = Node(
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

    return LaunchDescription([
        namespace,
        network_interface,
        input_dds_topic,
        output_topic,
        frame_id,
        use_sim_time,
        GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),
            PushRosNamespace('perception'),
            bridge_node,
        ]),
    ])
