from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare('mocap_optitrack'),
        'config',
        'mocap.yaml'
    ])

    return LaunchDescription([
        Node(
            package='mocap_optitrack',
            executable='mocap_node',
            name='mocap_node',
            output='screen',
            parameters=[config_path],
            # 'respawn' and 'required' are no longer standard args in ROS 2 launch
            # Use lifecycle or launch supervision for respawn-like behavior if needed
        )
    ])
