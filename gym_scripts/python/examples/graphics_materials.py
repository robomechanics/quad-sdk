"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.

Assimp Loading
------------
- Loads a handful of MJCF and URDF assets using assimp to load their meshes
- Demonstrates the usage of `use_mesh_materials` to
  override materials specified in asset files with mesh textures/materials
"""

import math
import numpy as np
from isaacgym import gymapi, gymutil


class AssetDesc:
    def __init__(self, file_name, flip_visual_attachments=False, mesh_normal_mode=gymapi.FROM_ASSET):
        self.file_name = file_name
        self.flip_visual_attachments = flip_visual_attachments
        self.mesh_normal_mode = mesh_normal_mode


asset_descriptors = [
    AssetDesc("urdf/ycb/010_potted_meat_can/010_potted_meat_can.urdf", False, gymapi.COMPUTE_PER_VERTEX),
    AssetDesc("mjcf/open_ai_assets/hand/shadow_hand.xml", False),
    AssetDesc("urdf/sektion_cabinet_model/urdf/sektion_cabinet.urdf", False),
    AssetDesc("urdf/franka_description/robots/franka_panda.urdf", True),
    AssetDesc("urdf/kinova_description/urdf/kinova.urdf", False),
    AssetDesc("urdf/anymal_b_simple_description/urdf/anymal.urdf", True),
    AssetDesc("urdf/kuka_allegro_description/kuka_allegro.urdf", False),
    AssetDesc("urdf/objects/cube_multicolor.urdf", True),
]

args = gymutil.parse_arguments()

# initialize gym
gym = gymapi.acquire_gym()

# configure sim
sim_params = gymapi.SimParams()
sim_params.dt = dt = 1.0 / 60.0
if args.physics_engine == gymapi.SIM_FLEX:
    pass
elif args.physics_engine == gymapi.SIM_PHYSX:
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 6
    sim_params.physx.num_velocity_iterations = 0
    sim_params.physx.num_threads = args.num_threads
    sim_params.physx.use_gpu = args.use_gpu

sim_params.use_gpu_pipeline = False
if args.use_gpu_pipeline:
    print("WARNING: Forcing CPU pipeline.")

sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)

# add ground plane
plane_params = gymapi.PlaneParams()
gym.add_ground(sim, plane_params)

# create viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    print("*** Failed to create viewer")
    quit()

# load asset
asset_root = "../../assets"

assets = []
for asset_desc in asset_descriptors:
    for override in [True, False]:
        asset_file = asset_desc.file_name

        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = True
        asset_options.flip_visual_attachments = asset_desc.flip_visual_attachments
        asset_options.use_mesh_materials = override
        asset_options.mesh_normal_mode = asset_desc.mesh_normal_mode

        print("Loading asset '%s' from '%s'" % (asset_file, asset_root))
        assets.append(gym.load_asset(sim, asset_root, asset_file, asset_options))

# set up the env grid
num_per_row = 2
spacing = 1
env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
env_upper = gymapi.Vec3(spacing, spacing, spacing)

# position the camera
cam_pos = gymapi.Vec3(17.2, 2.0, 16)
cam_target = gymapi.Vec3(5, -2.5, 13)
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

# cache useful handles
envs = []
actor_handles = []

for i, asset in enumerate(assets):
    # create env
    env = gym.create_env(sim, env_lower, env_upper, num_per_row)
    envs.append(env)

    # add actor
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 1.32, 0.0)
    pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107)

    actor_handle = gym.create_actor(env, asset, pose, "actor", i, 1)
    actor_handles.append(actor_handle)

while not gym.query_viewer_has_closed(viewer):
    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)

print("Done")

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
