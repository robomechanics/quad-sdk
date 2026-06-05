"""
Per-robot MuJoCo injection profiles.

Each profile contains the robot-specific information for MuJoCo: the 
floating-base joint name MuJoCo uses for odometry and mapping from ros2_control 
joint names to MuJoCo actuator/joint names.

To support a new robot, add an entry to `PROFILES` keyed by `robot_type`.
The generic injection logic in `mujoco_urdf_utils.py` consumes these
profiles unchanged.
"""

_GO2_PROFILE = {
    'odom_free_joint_name': 'floating_base',
    'initial_keyframe': 'home',
    'joint_map': [
        ('0',  'FL_thigh_joint'),
        ('1',  'FL_calf_joint'),
        ('2',  'RL_thigh_joint'),
        ('3',  'RL_calf_joint'),
        ('4',  'FR_thigh_joint'),
        ('5',  'FR_calf_joint'),
        ('6',  'RR_thigh_joint'),
        ('7',  'RR_calf_joint'),
        ('8',  'FL_hip_joint'),
        ('9',  'RL_hip_joint'),
        ('10', 'FR_hip_joint'),
        ('11', 'RR_hip_joint'),
    ],
}

_A1_PROFILE = {
    'odom_free_joint_name': 'floating_base',
    'initial_keyframe': 'home',
    'joint_map': [
        ('0',  'FL_thigh_joint'),
        ('1',  'FL_calf_joint'),
        ('2',  'RL_thigh_joint'),
        ('3',  'RL_calf_joint'),
        ('4',  'FR_thigh_joint'),
        ('5',  'FR_calf_joint'),
        ('6',  'RR_thigh_joint'),
        ('7',  'RR_calf_joint'),
        ('8',  'FL_hip_joint'),
        ('9',  'RL_hip_joint'),
        ('10', 'FR_hip_joint'),
        ('11', 'RR_hip_joint'),
    ],
}

_A2_PROFILE = {
    'odom_free_joint_name': 'floating_base_joint',
    'initial_keyframe': 'home',
    'joint_map': [
        ('FL_thigh_joint', 'FL_thigh_joint'),
        ('FL_calf_joint',  'FL_calf_joint'),
        ('RL_thigh_joint', 'RL_thigh_joint'),
        ('RL_calf_joint',  'RL_calf_joint'),
        ('FR_thigh_joint', 'FR_thigh_joint'),
        ('FR_calf_joint',  'FR_calf_joint'),
        ('RR_thigh_joint', 'RR_thigh_joint'),
        ('RR_calf_joint',  'RR_calf_joint'),
        ('FL_hip_joint',   'FL_hip_joint'),
        ('RL_hip_joint',   'RL_hip_joint'),
        ('FR_hip_joint',   'FR_hip_joint'),
        ('RR_hip_joint',   'RR_hip_joint'),
    ],
}

_GO1_PROFILE = {
    'odom_free_joint_name': 'floating_base',
    'initial_keyframe': 'home',
    'joint_map': [
        ('FL_thigh_joint',   'FL_thigh_joint'),
        ('FL_calf_joint', 'FL_calf_joint'),
        ('RL_thigh_joint',  'RL_thigh_joint'),
        ('RL_calf_joint',  'RL_calf_joint'),
        ('FR_thigh_joint',  'FR_thigh_joint'),
        ('FR_calf_joint',  'FR_calf_joint'),
        ('RR_thigh_joint',  'RR_thigh_joint'),
        ('RR_calf_joint',  'RR_calf_joint'),
        ('FL_hip_joint',  'FL_hip_joint'),
        ('RL_hip_joint',  'RL_hip_joint'),
        ('FR_hip_joint',  'FR_hip_joint'),
        ('RR_hip_joint',  'RR_hip_joint'),

    ],
}

_SPOT_PROFILE = {
    'odom_free_joint_name': 'floating_base',
    'initial_keyframe': 'home',
    'joint_map': [
        ('0',  'fl_hy'),
        ('1',  'fl_kn'),
        ('2',  'hl_hy'),
        ('3',  'hl_kn'),
        ('4',  'fr_hy'),
        ('5',  'fr_kn'),
        ('6',  'hr_hy'),
        ('7',  'hr_kn'),
        ('8',  'fl_hx'),
        ('9',  'hl_hx'),
        ('10', 'fr_hx'),
        ('11', 'hr_hx'),
    ],
}

_B2_PROFILE = {
    'odom_free_joint_name': 'floating_base_joint',
    'initial_keyframe': 'home',
    'joint_map': [
        ('0',  'FL_thigh_joint'),
        ('1',  'FL_calf_joint'),
        ('2',  'RL_thigh_joint'),
        ('3',  'RL_calf_joint'),
        ('4',  'FR_thigh_joint'),
        ('5',  'FR_calf_joint'),
        ('6',  'RR_thigh_joint'),
        ('7',  'RR_calf_joint'),
        ('8',  'FL_hip_joint'),
        ('9',  'RL_hip_joint'),
        ('10', 'FR_hip_joint'),
        ('11', 'RR_hip_joint'),
    ],
}

_SPIRIT_PROFILE = {
    'odom_free_joint_name': 'floating_base',
    'initial_keyframe': 'home',
    'joint_map': [
        ('0',  '0'),
        ('1',  '1'),
        ('2',  '2'),
        ('3',  '3'),
        ('4',  '4'),
        ('5',  '5'),
        ('6',  '6'),
        ('7',  '7'),
        ('8',  '8'),
        ('9',  '9'),
        ('10', '10'),
        ('11', '11'),
    ],
}

PROFILES = {
    'go2': _GO2_PROFILE,
    'go1': _GO1_PROFILE,
    'a1':  _A1_PROFILE,
    'a2':  _A2_PROFILE,
    'b2':  _B2_PROFILE,
    'spot': _SPOT_PROFILE,
    'spirit': _SPIRIT_PROFILE,
}

def has_profile(robot_type):
    """Whether on-the-fly MuJoCo URDF generation is supported for this robot."""
    return robot_type in PROFILES

def get_profile(robot_type):
    """Return the profile for `robot_type` or raise with a fix-it hint."""
    try:
        return PROFILES[robot_type]
    except KeyError:
        raise RuntimeError(
            f"No MuJoCo injection profile for robot_type {robot_type!r}; "
            "add one to PROFILES in mujoco_profiles.py."
        )