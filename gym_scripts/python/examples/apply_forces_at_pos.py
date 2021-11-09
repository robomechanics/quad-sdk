"""
Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Apply Forces At Positions (apply_forces_at_pos.py)
----------------------------
This example shows how to apply rigid body forces at given positions using the tensor API.

"""

from isaacgym import gymutil
from isaacgym import gymapi
from isaacgym import gymtorch

import numpy as np
import torch

# initialize gym
gym = gymapi.acquire_gym()

# parse arguments
args = gymutil.parse_arguments(
    description="Example of applying forces to bodies at given positions")

# configure sim
sim_params = gymapi.SimParams()
sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
if args.physics_engine == gymapi.SIM_PHYSX:
    sim_params.substeps = 1
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 4
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.num_threads = args.num_threads
    sim_params.physx.use_gpu = args.use_gpu
elif args.physics_engine == gymapi.SIM_FLEX and not args.use_gpu_pipeline:
    sim_params.flex.shape_collision_margin = 0.25
    sim_params.flex.num_outer_iterations = 4
    sim_params.flex.num_inner_iterations = 10
else:
    raise Exception("GPU pipeline is only available with PhysX")

sim_params.use_gpu_pipeline = args.use_gpu_pipeline
device = args.sim_device if args.use_gpu_pipeline else 'cpu'

sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)
if sim is None:
    raise Exception("Failed to create sim")

# add ground plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)
gym.add_ground(sim, plane_params)

# create viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    raise Exception("Failed to create viewer")

# load ball asset
asset_root = "../../assets"
asset_file = "mjcf/nv_ant.xml"
asset = gym.load_asset(sim, asset_root, asset_file, gymapi.AssetOptions())

num_bodies = gym.get_asset_rigid_body_count(asset)
print('num_bodies', num_bodies)

# default pose
pose = gymapi.Transform()
pose.p.z = 1.0

# set up the env grid
num_envs = 4
num_per_row = int(np.sqrt(num_envs))
env_spacing = 2.0
env_lower = gymapi.Vec3(-env_spacing, -env_spacing, 0.0)
env_upper = gymapi.Vec3(env_spacing, env_spacing, env_spacing)

# set random seed
np.random.seed(17)

envs = []
handles = []
for i in range(num_envs):
    # create env
    env = gym.create_env(sim, env_lower, env_upper, num_per_row)
    envs.append(env)

    # generate random bright color
    c = 0.5 + 0.5 * np.random.random(3)
    color = gymapi.Vec3(c[0], c[1], c[2])

    ahandle = gym.create_actor(env, asset, pose, "actor", i, 1)
    handles.append(ahandle)
    gym.set_rigid_body_color(env, ahandle, 0, gymapi.MESH_VISUAL_AND_COLLISION, color)

mid = 0.5 * env_spacing * (num_per_row - 1)
gym.viewer_camera_look_at(viewer, None, gymapi.Vec3(20, mid, 5), gymapi.Vec3(0, mid, 1))

gym.prepare_sim(sim)

rb_tensor = gym.acquire_rigid_body_state_tensor(sim)
rb_states = gymtorch.wrap_tensor(rb_tensor)
rb_positions = rb_states[:, 0:3].view(num_envs, num_bodies, 3)

force_offset = 0.2

frame_count = 0
while not gym.query_viewer_has_closed(viewer):

    if (frame_count - 99) % 200 == 0:

        gym.refresh_rigid_body_state_tensor(sim)

        # set forces and force positions for ant root bodies (first body in each env)
        forces = torch.zeros((num_envs, num_bodies, 3), device=device, dtype=torch.float)
        force_positions = rb_positions.clone()
        forces[:, 0, 2] = 400
        force_positions[:, 0, 1] += force_offset
        gym.apply_rigid_body_force_at_pos_tensors(sim, gymtorch.unwrap_tensor(forces), gymtorch.unwrap_tensor(force_positions), gymapi.ENV_SPACE)

        force_offset = -force_offset

    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)

    frame_count += 1

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
