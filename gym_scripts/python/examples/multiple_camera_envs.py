"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Multiple cameras with multiple environments example
--------------------
Demonstrates ability to instantiate multiple cameras within multiple
environments in a scene
"""
import os

import numpy as np
from isaacgym import gymapi
from isaacgym import gymutil

gym = gymapi.acquire_gym()

# parse arguments
args = gymutil.parse_arguments(description="Multiple Cameras Example",
                               custom_parameters=[
                                   {"name": "--save_images", "action": "store_true", "help": "Write RGB and Depth Images To Disk"},
                                   {"name": "--up_axis_z", "action": "store_true", "help": ""}])

sim_params = gymapi.SimParams()
plane_params = gymapi.PlaneParams()

sim_params.up_axis = gymapi.UP_AXIS_Y
if args.up_axis_z:
    sim_params.up_axis = gymapi.UP_AXIS_Z

sim_params.use_gpu_pipeline = False
if args.use_gpu_pipeline:
    print("WARNING: Forcing CPU pipeline.")

sim = gym.create_sim(0, 0, args.physics_engine, sim_params)
if sim is None:
    print("*** Failed to create sim")
    quit()

gym.add_ground(sim, plane_params)

# create envs
n_envs = 2
spacing = 1
lower = gymapi.Vec3(-spacing, 0.0, -spacing)
upper = gymapi.Vec3(spacing, spacing, spacing)
num_per_row = int(np.sqrt(n_envs))

env_ptrs = [gym.create_env(sim, lower, upper, num_per_row) for _ in range(n_envs)]
env_idxs = [i for i in range(n_envs)]

camera_props = gymapi.CameraProperties()
camera_props.horizontal_fov = 75.0
camera_props.width = 1920
camera_props.height = 1080


# track cameras
ch_map = {idx: {} for idx in env_idxs}

pos = gymapi.Vec3(1.38, 1.0, 0)
name = 'cam0'

for env_idx in env_idxs:
    env_ptr = env_ptrs[env_idx]
    ch = gym.create_camera_sensor(env_ptr, camera_props)
    ch_map[env_idx][name] = ch
    gym.set_camera_transform(ch, env_ptr, gymapi.Transform(p=pos, r=gymapi.Quat()))
    print("Added {} with handle {} in env {} | View matrix: {}".format(name, ch, env_idx, gym.get_camera_view_matrix(sim, env_ptr, ch)))

pos = gymapi.Vec3(0.5, 9.0, .8)
name = 'cam1'

for env_idx in env_idxs:
    env_ptr = env_ptrs[env_idx]
    ch = gym.create_camera_sensor(env_ptr, camera_props)
    ch_map[env_idx][name] = ch
    gym.set_camera_transform(ch, env_ptr, gymapi.Transform(p=pos, r=gymapi.Quat()))
    print("Added {} with handle {} in env {} | View matrix: {}".format(name, ch, env_idx, gym.get_camera_view_matrix(sim, env_ptr, ch)))

if args.save_images and not os.path.exists("multiple_camera_images"):
    os.mkdir("multiple_camera_images")

gym.render_all_camera_sensors(sim)

for env_idx in env_idxs:
    env_ptr = env_ptrs[env_idx]

    name = 'cam0'
    ch = ch_map[env_idx][name]
    rgb_filename = "multiple_camera_images/rgb_env%d_cam0.png" % (env_idx)
    if args.save_images:
        gym.write_camera_image_to_file(sim, env_ptr, ch, gymapi.IMAGE_COLOR, rgb_filename)

    print("View matrix of {} with handle {} in env {}: {}".format(name, ch, env_idx, gym.get_camera_view_matrix(sim, env_ptr, ch)))

    name = 'cam1'
    ch = ch_map[env_idx][name]
    rgb_filename = "multiple_camera_images/rgb_env%d_cam1.png" % (env_idx)

    if args.save_images:
        gym.write_camera_image_to_file(sim, env_ptr, ch, gymapi.IMAGE_COLOR, rgb_filename)

    print("View matrix of {} with handle {} in env {}: {}".format(name, ch, env_idx, gym.get_camera_view_matrix(sim, env_ptr, ch)))

gym.simulate(sim)
gym.fetch_results(sim, True)
