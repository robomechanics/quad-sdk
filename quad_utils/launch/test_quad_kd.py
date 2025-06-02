import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package paths
    quad_utils_path = get_package_share_directory('quad_utils')
    desc_path = get_package_share_directory('spirit_description')

    # Load the URDF
    urdf_path = os.path.join(desc_path, 'models', 'spirit', 'urdf', 'spirit.urdf')
    with open(urdf_path, 'r') as f:
        urdf = f.read()

    # Parameter YAML files
    # param_files = [
    #     os.path.join(quad_utils_path, 'config', 'topics_robot.yaml'),
    #     os.path.join(quad_utils_path, 'config', 'topics_global.yaml'),
    #     # os.path.join(quad_utils_path, 'config', 'quad_kd_params.yaml'),  # optional
    # ]

    # Launch the test node as a regular node
    return LaunchDescription([
        Node(
            package='quad_utils',
            executable='test_quad_kd',  # make sure this matches your target name
            name='quad_kd_test_node',
            parameters=[{'robot_description': urdf}],
            output='screen'
        )
    ])