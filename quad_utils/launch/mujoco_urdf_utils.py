"""
MuJoCo-specific URDF construction.

MuJoCo requires specific elements in the URDF that are not needed for Gazebo, 
but are needed for MuJoCo. To run the same robot in both simulators without 
maintaining two separate URDFs, this file injects MuJoCo specific elements at 
launch. These include: 
  * a `<mujoco>` block with the asset compiler and the `mujoco_ros2_control`
    plugin reference;
  * a `<ros2_control name="MujocoSystem">` block whose hardware plugin is
    `mujoco_ros2_control/MujocoSystemInterface`;
  * one `<transmission>` per controllable joint that maps the
    ros2_control joint name to a MuJoCo actuator/joint name.

The existing `GazeboSimSystem` `<ros2_control>` block is stripped during
injection so the two hardware interfaces don't collide.

Per-robot data (joint maps, floating-base joint name) lives in
`mujoco_profiles.py`; everything in this file is robot-agnostic.
"""

import os
import xml.etree.ElementTree as ET
import xacro
from mujoco_profiles import get_profile

def build_mujoco_urdf(robot_type, urdf_path, desc_path, world_path,
                      namespace, plugin_params_path):
    """Process the standard Gazebo URDF xacro and inject MuJoCo-specific
    elements, returning the final URDF XML as a string.

    `desc_path` is the share directory of `<robot>_description`; the MuJoCo
    asset directory is derived from it (`<desc_path>/models/<type>/<type>_mjc/assets/`).
    """
    profile = get_profile(robot_type)
    base_urdf = xacro.process_file(urdf_path).toxml()
    meshdir = os.path.join(desc_path, 'models', robot_type,
                           f'{robot_type}_mjc', 'assets') + '/'
    return _inject_mujoco_elements(
        base_urdf, profile, world_path, namespace,
        meshdir, plugin_params_path,
    )

def _inject_mujoco_elements(urdf_xml, profile, world_path, namespace,
                            meshdir, plugin_params_path):
    root = ET.fromstring(urdf_xml)

    for ros2_ctrl in list(root.findall('ros2_control')):
        root.remove(ros2_ctrl)

    mujoco_el = ET.SubElement(root, 'mujoco')
    ET.SubElement(mujoco_el, 'compiler', {
        'meshdir': meshdir,
        'strippath': 'true',
    })
    plugin_el = ET.SubElement(mujoco_el, 'plugin', {
        'name': 'mujoco_ros2_control',
        'filename': 'libmujoco_ros2_control.so',
    })
    ET.SubElement(plugin_el, 'parameters').text = plugin_params_path

    rc = ET.SubElement(root, 'ros2_control',
                       {'name': 'MujocoSystem', 'type': 'system'})
    hardware = ET.SubElement(rc, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'mujoco_ros2_control/MujocoSystemInterface'
    ET.SubElement(hardware, 'param', {'name': 'mujoco_model'}).text = world_path
    ET.SubElement(hardware, 'param', {'name': 'odom_free_joint_name'}).text = profile['odom_free_joint_name']
    ET.SubElement(hardware, 'param', {'name': 'odom_topic'}).text = f'/{namespace}/odom'

    if profile.get('initial_keyframe'):
        ET.SubElement(hardware, 'param', {'name': 'initial_keyframe'}).text = profile['initial_keyframe']

    for idx, mjc_joint_name in profile['joint_map']:
        j = ET.SubElement(rc, 'joint', {'name': idx})
        ET.SubElement(j, 'command_interface', {'name': 'effort'})
        ET.SubElement(j, 'state_interface', {'name': 'velocity'})
        ET.SubElement(j, 'state_interface', {'name': 'position'})
        trans = ET.SubElement(rc, 'transmission', {'name': f'trans_{idx}'})
        ET.SubElement(trans, 'plugin').text = 'transmission_interface/SimpleTransmission'
        ET.SubElement(trans, 'actuator', {'name': mjc_joint_name, 'role': 'actuator'})
        ET.SubElement(trans, 'joint', {'name': idx, 'role': 'joint'})

    return ET.tostring(root, encoding='unicode')