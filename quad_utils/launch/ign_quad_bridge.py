from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context)

    bridge_nodes = []
    toe_ids = [0, 1, 2, 3]

    for i in toe_ids:
        contact_ign = f"/world/default/model/{ns}/link/toe{i}/sensor/toe{i}_contact/contact"
        ft_ign = f"/world/default/model/{ns}/joint/jtoe{i}/sensor/toe{i}_grf/forcetorque"

        bridge_nodes.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"{ns}_contact_bridge_toe{i}",
                arguments=[f"{contact_ign}@gazebo_msgs/msg/ContactsState[ignition.msgs.Contacts"],
            )
        )

        bridge_nodes.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"{ns}_ft_bridge_toe{i}",
                arguments=[f"{ft_ign}@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench"],
            )
        )

    return bridge_nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        OpaqueFunction(function=launch_setup)
    ])
