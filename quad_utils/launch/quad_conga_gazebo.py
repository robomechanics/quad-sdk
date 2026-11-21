"""Run the complete CPG controller through Quad-SDK in Gazebo (no hardware)."""
import json
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def start(context):
    root=Path(LaunchConfiguration('conga_root').perform(context)).expanduser().resolve()
    selection=LaunchConfiguration('selection').perform(context)
    if not selection:
        selection=str(root/'assets/selection.json')
        if not Path(selection).is_file():
            selection=str(root/'exports/go2_cpg_hardware_prep_20261102/assets/selection.json')
    if not Path(selection).is_file(): raise RuntimeError('Missing CPG selection: '+selection)
    script=os.path.join(get_package_prefix('quad_utils'),'lib','quad_utils','cpg_gazebo.py')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('quad_utils'),'launch','quad_gazebo.py')),
            launch_arguments={'robot_configs':json.dumps([{'name':'robot_1','type':'go2','controller':'conga_driver','init_pose':'-x 0 -y 0 -z 0.5'}]),
                              'cpg_disable_springs':LaunchConfiguration('cpg_disable_springs'),
                              'cpg_disable_damping':LaunchConfiguration('cpg_disable_damping'),
                              'cpg_model_dump':LaunchConfiguration('cpg_model_dump'),
                              'cpg_actuator_profile':LaunchConfiguration('cpg_actuator_profile'),
                              'gui':LaunchConfiguration('gui'), 'rviz':LaunchConfiguration('rviz'), 'use_sim_time':'true'}.items()),
        ExecuteProcess(cmd=['/usr/bin/python3','-u',script,'--conga-root',str(root),
                            '--selection',selection,'--policy-python',LaunchConfiguration('policy_python'),
                            '--trace',LaunchConfiguration('trace'),'--ros-args','-p','use_sim_time:=true'],output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('conga_root',default_value=str(Path(get_package_share_directory('robot_driver'))/'conga'),description='Packaged frozen runtime root'),
        DeclareLaunchArgument('policy_python',default_value=os.environ.get('CONGA_POLICY_PYTHON', '/opt/conga-venv/bin/python' if Path('/opt/conga-venv/bin/python').exists() else str(Path.home()/'venvs/go2-cpg/bin/python'))),
        DeclareLaunchArgument('cpg_disable_springs',default_value='false',description='Diagnostic: zero passive joint springs'),
        DeclareLaunchArgument('cpg_disable_damping',default_value='false',description='Diagnostic: zero passive joint damping'),
        DeclareLaunchArgument('cpg_model_dump',default_value='',description='Optional generated SDF audit path'),
        DeclareLaunchArgument('cpg_actuator_profile',default_value='go2hv_mujoco_v1'),
        DeclareLaunchArgument('selection',default_value='',description='Defaults to frozen contact-free actor'),
        DeclareLaunchArgument('trace',default_value='/tmp/cpg_gazebo_trace.jsonl'),
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        OpaqueFunction(function=start)])
