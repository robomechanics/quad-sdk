"""Per-robot MuJoCo data not derivable from the standard YAML config.

The YAML files in `quad_utils/config/<robot>.yaml` are the single source of
truth for the leg/joint topology: each `leg_X.joints.{abad,hip,knee}.name`
gives the ros2_control joint name (matching the URDF). What lives here is
only the MuJoCo-specific delta:

  * `odom_free_joint_name` — the floating-base freejoint name in the MJCF
    that mujoco_ros2_control reads odometry from.
  * `mjc_joint_names` — for robots whose MJCF asset uses different joint
    names than the URDF/YAML side, the per-leg/per-role MJCF name. When
    this is omitted, we use an identity map (MJC name == YAML name), which
    is the case for spirit and go1.

To support a new robot:

  1. Add a YAML config in `quad_utils/config/<robot>.yaml` with the leg
     topology (this is required for the rest of the SDK anyway).
  2. Add an entry to `PROFILES` below — `mjc_joint_names` only when the
     MJCF asset uses different names than the URDF.
"""

import os

import yaml


LEGS = ('leg_0', 'leg_1', 'leg_2', 'leg_3')
JOINT_ROLES = ('abad', 'hip', 'knee')


# go2 / a1: URDF uses numeric joint names ("0"..."11"); MJCF uses the
# Unitree convention `<FL|FR|RL|RR>_<hip|thigh|calf>_joint`.
_UNITREE_NUMERIC_TO_MJC = {
    'leg_0': {'abad': 'FL_hip_joint', 'hip': 'FL_thigh_joint', 'knee': 'FL_calf_joint'},
    'leg_1': {'abad': 'RL_hip_joint', 'hip': 'RL_thigh_joint', 'knee': 'RL_calf_joint'},
    'leg_2': {'abad': 'FR_hip_joint', 'hip': 'FR_thigh_joint', 'knee': 'FR_calf_joint'},
    'leg_3': {'abad': 'RR_hip_joint', 'hip': 'RR_thigh_joint', 'knee': 'RR_calf_joint'},
}

# Spot: MJCF uses Boston Dynamics' compact naming. `h*` is the rear (hind)
# pair; `_hx` is hip-x (abduction), `_hy` is hip-y (front/back hip), `_kn`
# is knee.
_SPOT_MJC_JOINT_NAMES = {
    'leg_0': {'abad': 'fl_hx', 'hip': 'fl_hy', 'knee': 'fl_kn'},
    'leg_1': {'abad': 'hl_hx', 'hip': 'hl_hy', 'knee': 'hl_kn'},
    'leg_2': {'abad': 'fr_hx', 'hip': 'fr_hy', 'knee': 'fr_kn'},
    'leg_3': {'abad': 'hr_hx', 'hip': 'hr_hy', 'knee': 'hr_kn'},
}


PROFILES = {
    # YAML joint names == MJCF joint names → no `mjc_joint_names` override.
    'spirit': {'odom_free_joint_name': 'floating_base'},
    'go1':    {'odom_free_joint_name': 'floating_base'},

    # YAML uses "0".."11" but MJCF uses Unitree FR/FL/RR/RL_*_joint names.
    'go2': {'odom_free_joint_name': 'floating_base',
            'mjc_joint_names': _UNITREE_NUMERIC_TO_MJC},
    'a1':  {'odom_free_joint_name': 'floating_base',
            'mjc_joint_names': _UNITREE_NUMERIC_TO_MJC},

    'spot': {'odom_free_joint_name': 'freejoint',
             'mjc_joint_names': _SPOT_MJC_JOINT_NAMES},
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


def _load_robot_yaml(robot_yaml_path):
    if not os.path.isfile(robot_yaml_path):
        raise RuntimeError(
            f"Robot YAML not found: {robot_yaml_path}. "
            "Each robot needs a config in quad_utils/config/<robot>.yaml."
        )
    with open(robot_yaml_path) as f:
        cfg = yaml.safe_load(f)
    return cfg.get('/**', {}).get('ros__parameters', {})


def build_joint_map(robot_type, robot_yaml_path):
    """Return [(ros2_control_name, mjc_joint_name), ...] for `robot_type`.

    `robot_yaml_path` is the path to the per-robot YAML (the same file that
    feeds ros2_control / robot_driver). The ros2_control names come from
    `leg_X.joints.{abad,hip,knee}.name`; the MJC names come from the
    per-robot profile (or default to the ros2_control name when the
    profile omits `mjc_joint_names`).
    """
    profile = get_profile(robot_type)
    overrides = profile.get('mjc_joint_names', {})
    root = _load_robot_yaml(robot_yaml_path)

    joint_map = []
    for leg in LEGS:
        leg_cfg = root.get(leg, {}).get('joints', {})
        leg_overrides = overrides.get(leg, {})
        for role in JOINT_ROLES:
            joint = leg_cfg.get(role)
            if not joint or 'name' not in joint:
                raise RuntimeError(
                    f"{robot_yaml_path} is missing {leg}.joints.{role}.name "
                    f"(needed to build the MuJoCo joint map for {robot_type!r})."
                )
            ros_name = joint['name']
            mjc_name = leg_overrides.get(role, ros_name)
            joint_map.append((ros_name, mjc_name))
    return joint_map
