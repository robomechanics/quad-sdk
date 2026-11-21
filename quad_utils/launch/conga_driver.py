"""Go2 contact-free Conga hardware launch; starts in SAFETY, never arms itself."""
import os
from pathlib import Path
import json
import select
import socket
import subprocess
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    if value('activate_hardware').lower() != 'true':
        raise RuntimeError('Use activate_hardware:=true to start the Unitree hardware interface')
    iface = value('mcu_iface').strip()
    if not iface or iface == 'lo':
        raise RuntimeError('Set mcu_iface:=<robot-facing interface> or ROBOT_MCU_IFACE')
    try:
        socket.if_nametoindex(iface)
    except OSError as error:
        raise RuntimeError(f'Unitree network interface {iface!r} is unavailable') from error
    root = Path(value('conga_root')).expanduser().resolve()
    selection = value('selection') or str(root/'assets/selection.json')
    if not value('selection') and not Path(selection).is_file():
        selection = str(root/'exports/go2_cpg_hardware_prep_20261102/assets/selection.json')
    scripts = Path(get_package_prefix('quad_utils'))/'lib/quad_utils'
    # Load/verify the actual contact-free worker before the driver can release sport mode.
    worker = subprocess.Popen([value('policy_python'), str(scripts/'conga_worker.py'),
        '--conga-root', str(root), '--selection', selection], stdin=subprocess.PIPE,
        stdout=subprocess.PIPE, text=True)
    try:
        if not select.select([worker.stdout], [], [], 20)[0] or not json.loads(worker.stdout.readline()).get('ready'):
            raise RuntimeError('Contact-free policy preflight failed')
    finally:
        worker.stdin.close()
        try: worker.wait(timeout=3)
        except subprocess.TimeoutExpired: worker.kill(); worker.wait()
    namespace = value('namespace')
    return [
        SetEnvironmentVariable('ROBOT_MCU_IFACE', iface),
        ExecuteProcess(cmd=['/usr/bin/python3', '-u', str(scripts/'cpg_gazebo.py'),
            '--hardware', '--conga-root', str(root), '--selection', selection,
            '--policy-python', value('policy_python'), '--robot-ns', '/'+namespace,
            '--trace', value('trace'), '--ros-args', '-p', 'use_sim_time:=false'], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('quad_utils'))/'launch/robot_driver.py')),
            launch_arguments={'controller':'conga_driver', 'robot_type':'go2', 'namespace':namespace,
                'is_hardware':'true', 'mocap':'false', 'use_sim_time':'false', 'conga_hardware_enabled':'true'}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('conga_root', default_value=str(Path(get_package_share_directory('robot_driver'))/'conga')),
        DeclareLaunchArgument('policy_python', default_value=os.environ.get('CONGA_POLICY_PYTHON', '/opt/conga-venv/bin/python' if Path('/opt/conga-venv/bin/python').exists() else str(Path.home()/'venvs/go2-cpg/bin/python'))),
        DeclareLaunchArgument('selection', default_value=''),
        DeclareLaunchArgument('activate_hardware', default_value='false'),
        DeclareLaunchArgument('mcu_iface', default_value=os.environ.get('ROBOT_MCU_IFACE', ''),
                              description='Linux network interface on the Go2 MCU subnet'),
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('trace', default_value='/tmp/conga_hardware_trace.jsonl'),
        OpaqueFunction(function=start)])
