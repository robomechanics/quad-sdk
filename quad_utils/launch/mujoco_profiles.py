"""Per-robot MuJoCo injection profiles.

Each profile captures the robot-specific bits that the Gazebo URDF doesn't
carry: the floating-base joint name MuJoCo uses for odometry, and the
mapping from ros2_control joint names to MuJoCo actuator/joint names. These
mirror the rows that lived in the (now retired) `<robot>_mujoco.urdf.xacro`
files as `<xacro:mujoco_joint>` entries.


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


# a1's URDF (and ros2_control block) uses the same numeric joint names as go2,
# and its mjcf uses the same FL_/FR_/RL_/RR_ actuator names — so the mapping
# is identical to go2.
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


# go1's URDF and mjcf both use the same FL_/FR_/RL_/RR_ joint names, so the
# ros2_control -> MuJoCo map is identity.
_GO1_PROFILE = {
    'odom_free_joint_name': 'floating_base',
    'initial_keyframe': 'home',
    'joint_map': [
        ('FL_hip_joint',   'FL_hip_joint'),
        ('FL_thigh_joint', 'FL_thigh_joint'),
        ('FL_calf_joint',  'FL_calf_joint'),
        ('RL_hip_joint',   'RL_hip_joint'),
        ('RL_thigh_joint', 'RL_thigh_joint'),
        ('RL_calf_joint',  'RL_calf_joint'),
        ('FR_hip_joint',   'FR_hip_joint'),
        ('FR_thigh_joint', 'FR_thigh_joint'),
        ('FR_calf_joint',  'FR_calf_joint'),
        ('RR_hip_joint',   'RR_hip_joint'),
        ('RR_thigh_joint', 'RR_thigh_joint'),
        ('RR_calf_joint',  'RR_calf_joint'),
    ],
}


# spot.xml uses Boston-Dynamics' short actuator names: f/h = front/hind,
# l/r = left/right; hx = hip abduction, hy = hip flexion (thigh),
# kn = knee (calf). The ros2_control side uses the numeric scheme shared
# with go2/a1 (8/0/1 = FL hip/thigh/calf, 9/2/3 = RL, 10/4/5 = FR,
# 11/6/7 = RR).
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


# b2's mjcf uses short FL_/FR_/RL_/RR_ actuator names (no `_joint` suffix)
# and a `floating_base_joint` freejoint (not `floating_base`). Numeric
# ros2_control scheme matches go2/a1/spot.
_B2_PROFILE = {
    'odom_free_joint_name': 'floating_base_joint',
    'initial_keyframe': 'home',
    'joint_map': [
        ('0',  'FL_thigh'),
        ('1',  'FL_calf'),
        ('2',  'RL_thigh'),
        ('3',  'RL_calf'),
        ('4',  'FR_thigh'),
        ('5',  'FR_calf'),
        ('6',  'RR_thigh'),
        ('7',  'RR_calf'),
        ('8',  'FL_hip'),
        ('9',  'RL_hip'),
        ('10', 'FR_hip'),
        ('11', 'RR_hip'),
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
