"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Franka Operational Space Control
----------------
Operational Space Control of Franka robot to demonstrate Jacobian and Mass Matrix Tensor APIs
"""

from isaacgym import gymapi
from isaacgym import gymutil
from isaacgym import gymtorch
from isaacgym.torch_utils import *

import math
import numpy as np
import torch


def orientation_error(desired, current):
    cc = quat_conjugate(current)
    q_r = quat_mul(desired, cc)
    return q_r[:, 0:3] * torch.sign(q_r[:, 3]).unsqueeze(-1)


# Parse arguments
args = gymutil.parse_arguments(description="Franka Tensor OSC Example",
                               custom_parameters=[
                                   {"name": "--num_envs", "type": int, "default": 256, "help": "Number of environments to create"},
                                   {"name": "--pos_control", "type": gymutil.parse_bool, "const": True, "default": True, "help": "Trace circular path in XZ plane"},
                                   {"name": "--orn_control", "type": gymutil.parse_bool, "const": True, "default": False, "help": "Send random orientation commands"}])

# Initialize gym
gym = gymapi.acquire_gym()

# configure sim
sim_params = gymapi.SimParams()
sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)
sim_params.dt = 1.0 / 60.0
sim_params.substeps = 2
if args.physics_engine == gymapi.SIM_PHYSX:
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 4
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.num_threads = args.num_threads
    sim_params.physx.use_gpu = args.use_gpu
else:
    raise Exception("This example can only be used with PhysX")

sim_params.use_gpu_pipeline = args.use_gpu_pipeline

sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)

if sim is None:
    raise Exception("Failed to create sim")

# Create viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    raise Exception("Failed to create viewer")

# Add ground plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)
gym.add_ground(sim, plane_params)

# Load franka asset
asset_root = "../../assets"
franka_asset_file = "urdf/franka_description/robots/franka_panda.urdf"

asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True
asset_options.flip_visual_attachments = True
asset_options.armature = 0.01
asset_options.disable_gravity = True

print("Loading asset '%s' from '%s'" % (franka_asset_file, asset_root))
franka_asset = gym.load_asset(
    sim, asset_root, franka_asset_file, asset_options)

# get joint limits and ranges for Franka
franka_dof_props = gym.get_asset_dof_properties(franka_asset)
franka_lower_limits = franka_dof_props['lower']
franka_upper_limits = franka_dof_props['upper']
franka_ranges = franka_upper_limits - franka_lower_limits
franka_mids = 0.5 * (franka_upper_limits + franka_lower_limits)
franka_num_dofs = len(franka_dof_props)

# set default DOF states
default_dof_state = np.zeros(franka_num_dofs, gymapi.DofState.dtype)
default_dof_state["pos"][:7] = franka_mids[:7]

# set DOF control properties (except grippers)
franka_dof_props["driveMode"][:7].fill(gymapi.DOF_MODE_EFFORT)
franka_dof_props["stiffness"][:7].fill(0.0)
franka_dof_props["damping"][:7].fill(0.0)

# set DOF control properties for grippers
franka_dof_props["driveMode"][7:].fill(gymapi.DOF_MODE_POS)
franka_dof_props["stiffness"][7:].fill(800.0)
franka_dof_props["damping"][7:].fill(40.0)

# Set up the env grid
num_envs = args.num_envs
num_per_row = int(math.sqrt(num_envs))
spacing = 1.0
env_lower = gymapi.Vec3(-spacing, -spacing, 0.0)
env_upper = gymapi.Vec3(spacing, spacing, spacing)

# default franka pose
pose = gymapi.Transform()
pose.p = gymapi.Vec3(0, 0, 0)
pose.r = gymapi.Quat(0, 0, 0, 1)

print("Creating %d environments" % num_envs)

envs = []
hand_idxs = []
init_pos_list = []
init_orn_list = []

for i in range(num_envs):
    # Create env
    env = gym.create_env(sim, env_lower, env_upper, num_per_row)
    envs.append(env)

    # Add franka
    franka_handle = gym.create_actor(env, franka_asset, pose, "franka", i, 1)

    # Set initial DOF states
    gym.set_actor_dof_states(env, franka_handle, default_dof_state, gymapi.STATE_ALL)

    # Set DOF control properties
    gym.set_actor_dof_properties(env, franka_handle, franka_dof_props)

    # Get inital hand pose
    hand_handle = gym.find_actor_rigid_body_handle(env, franka_handle, "panda_hand")
    hand_pose = gym.get_rigid_transform(env, hand_handle)
    init_pos_list.append([hand_pose.p.x, hand_pose.p.y, hand_pose.p.z])
    init_orn_list.append([hand_pose.r.x, hand_pose.r.y, hand_pose.r.z, hand_pose.r.w])

    # Get global index of hand in rigid body state tensor
    hand_idx = gym.find_actor_rigid_body_index(env, franka_handle, "panda_hand", gymapi.DOMAIN_SIM)
    hand_idxs.append(hand_idx)

# Point camera at middle env
cam_pos = gymapi.Vec3(4, 3, 3)
cam_target = gymapi.Vec3(-4, -3, 0)
middle_env = envs[num_envs // 2 + num_per_row // 2]
gym.viewer_camera_look_at(viewer, middle_env, cam_pos, cam_target)

# ==== prepare tensors =====
# from now on, we will use the tensor API to access and control the physics simulation
gym.prepare_sim(sim)

# initial hand position and orientation tensors
init_pos = torch.Tensor(init_pos_list).view(num_envs, 3)
init_orn = torch.Tensor(init_orn_list).view(num_envs, 4)

if args.use_gpu_pipeline:
    init_pos = init_pos.to('cuda:0')
    init_orn = init_orn.to('cuda:0')

# desired hand positions and orientations
pos_des = init_pos.clone()
orn_des = init_orn.clone()

# Prepare jacobian tensor
# For franka, tensor shape is (num_envs, 10, 6, 9)
_jacobian = gym.acquire_jacobian_tensor(sim, "franka")
jacobian = gymtorch.wrap_tensor(_jacobian)

# Jacobian entries for end effector
hand_index = gym.get_asset_rigid_body_dict(franka_asset)["panda_hand"]
j_eef = jacobian[:, hand_index - 1, :]

# Prepare mass matrix tensor
# For franka, tensor shape is (num_envs, 9, 9)
_massmatrix = gym.acquire_mass_matrix_tensor(sim, "franka")
mm = gymtorch.wrap_tensor(_massmatrix)

kp = 5
kv = 2 * math.sqrt(kp)

# Rigid body state tensor
_rb_states = gym.acquire_rigid_body_state_tensor(sim)
rb_states = gymtorch.wrap_tensor(_rb_states)

# DOF state tensor
_dof_states = gym.acquire_dof_state_tensor(sim)
dof_states = gymtorch.wrap_tensor(_dof_states)
dof_vel = dof_states[:, 1].view(num_envs, 9, 1)
dof_pos = dof_states[:, 0].view(num_envs, 9, 1)

itr = 0
while not gym.query_viewer_has_closed(viewer):

    # Randomize desired hand orientations
    if itr % 250 == 0 and args.orn_control:
        orn_des = torch.rand_like(orn_des)
        orn_des /= torch.norm(orn_des)

    itr += 1

    # Update jacobian and mass matrix
    gym.refresh_rigid_body_state_tensor(sim)
    gym.refresh_dof_state_tensor(sim)
    gym.refresh_jacobian_tensors(sim)
    gym.refresh_mass_matrix_tensors(sim)

    # Get current hand poses
    pos_cur = rb_states[hand_idxs, :3]
    orn_cur = rb_states[hand_idxs, 3:7]

    # Set desired hand positions
    if args.pos_control:
        pos_des[:, 0] = init_pos[:, 0] - 0.1
        pos_des[:, 1] = math.sin(itr / 50) * 0.2
        pos_des[:, 2] = init_pos[:, 2] + math.cos(itr / 50) * 0.2

    # Solve for control (Operational Space Control)
    m_inv = torch.inverse(mm)
    m_eef = torch.inverse(j_eef @ m_inv @ torch.transpose(j_eef, 1, 2))
    orn_cur /= torch.norm(orn_cur, dim=-1).unsqueeze(-1)
    orn_err = orientation_error(orn_des, orn_cur)

    pos_err = kp * (pos_des - pos_cur)

    if not args.pos_control:
        pos_err *= 0

    dpose = torch.cat([pos_err, orn_err], -1)

    u = torch.transpose(j_eef, 1, 2) @ m_eef @ (kp * dpose).unsqueeze(-1) - kv * mm @ dof_vel

    # Set tensor action
    gym.set_dof_actuation_force_tensor(sim, gymtorch.unwrap_tensor(u))

    # Step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # Step rendering
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, False)
    # gym.sync_frame_time(sim)

print("Done")

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
