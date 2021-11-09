"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Kuka bin perfromance test
-------------------------------
Test simulation perfromance and stability of the robotic arm dealing with a set of complex objects in a bin.
"""

from __future__ import print_function, division, absolute_import

import os
import math
import numpy as np
from isaacgym import gymapi
from isaacgym import gymutil
from copy import copy

axes_geom = gymutil.AxesGeometry(0.1)

sphere_rot = gymapi.Quat.from_euler_zyx(0.5 * math.pi, 0, 0)
sphere_pose = gymapi.Transform(r=sphere_rot)
sphere_geom = gymutil.WireframeSphereGeometry(0.03, 12, 12, sphere_pose, color=(1, 0, 0))

colors = [gymapi.Vec3(1.0, 0.0, 0.0),
          gymapi.Vec3(1.0, 127.0/255.0, 0.0),
          gymapi.Vec3(1.0, 1.0, 0.0),
          gymapi.Vec3(0.0, 1.0, 0.0),
          gymapi.Vec3(0.0, 0.0, 1.0),
          gymapi.Vec3(39.0/255.0, 0.0, 51.0/255.0),
          gymapi.Vec3(139.0/255.0, 0.0, 1.0)]

tray_color = gymapi.Vec3(0.24, 0.35, 0.8)
banana_color = gymapi.Vec3(0.85, 0.88, 0.2)
brick_color = gymapi.Vec3(0.9, 0.5, 0.1)


# initialize gym
gym = gymapi.acquire_gym()

# parse arguments
args = gymutil.parse_arguments(
    description="Kuka Bin Test",
    custom_parameters=[
        {"name": "--num_envs", "type": int, "default": 16, "help": "Number of environments to create"},
        {"name": "--num_objects", "type": int, "default": 10, "help": "Number of objects in the bin"},
        {"name": "--object_type", "type": int, "default": 0, "help": "Type of bjects to place in the bin: 0 - box, 1 - meat can, 2 - banana, 3 - mug, 4 - brick, 5 - random"}])

num_envs = args.num_envs
num_objects = args.num_objects
box_size = 0.05

# configure sim
sim_type = args.physics_engine
sim_params = gymapi.SimParams()
if sim_type == gymapi.SIM_FLEX:
    sim_params.substeps = 4
    sim_params.flex.solver_type = 5
    sim_params.flex.num_outer_iterations = 4
    sim_params.flex.num_inner_iterations = 20
    sim_params.flex.relaxation = 0.75
    sim_params.flex.warm_start = 0.8
elif sim_type == gymapi.SIM_PHYSX:
    sim_params.substeps = 2
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 25
    sim_params.physx.num_velocity_iterations = 0
    sim_params.physx.num_threads = args.num_threads
    sim_params.physx.use_gpu = args.use_gpu
    sim_params.physx.rest_offset = 0.001

sim_params.use_gpu_pipeline = False
if args.use_gpu_pipeline:
    print("WARNING: Forcing CPU pipeline.")

sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, sim_type, sim_params)
if sim is None:
    print("*** Failed to create sim")
    quit()

# add ground plane
plane_params = gymapi.PlaneParams()
gym.add_ground(sim, plane_params)

# create viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    print("*** Failed to create viewer")
    quit()

# load assets
asset_root = "../../assets"

table_dims = gymapi.Vec3(0.6, 0.4, 1.0)

pose = gymapi.Transform()
pose.p = gymapi.Vec3(0.0, 0.0, 0.0)
pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107)

asset_options = gymapi.AssetOptions()
asset_options.armature = 0.001
asset_options.fix_base_link = True
asset_options.thickness = 0.002

asset_options.mesh_normal_mode = gymapi.COMPUTE_PER_VERTEX

table_pose = gymapi.Transform()
table_pose.p = gymapi.Vec3(0.7, 0.5 * table_dims.y + 0.001, 0.0)

bin_pose = gymapi.Transform()
bin_pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107)

object_pose = gymapi.Transform()

table_asset = gym.create_box(sim, table_dims.x, table_dims.y, table_dims.z, asset_options)

# load assets of objects in a bin
asset_options.fix_base_link = False

can_asset_file = "urdf/ycb/010_potted_meat_can/010_potted_meat_can.urdf"
banana_asset_file = "urdf/ycb/011_banana/011_banana.urdf"
mug_asset_file = "urdf/ycb/025_mug/025_mug.urdf"
brick_asset_file = "urdf/ycb/061_foam_brick/061_foam_brick.urdf"

object_files = []
object_files.append(can_asset_file)
object_files.append(banana_asset_file)
object_files.append(mug_asset_file)
object_files.append(object_files)

object_assets = []

object_assets.append(gym.create_box(sim, box_size, box_size, box_size, asset_options))
object_assets.append(gym.load_asset(sim, asset_root, can_asset_file, asset_options))
object_assets.append(gym.load_asset(sim, asset_root, banana_asset_file, asset_options))
object_assets.append(gym.load_asset(sim, asset_root, mug_asset_file, asset_options))
object_assets.append(gym.load_asset(sim, asset_root, brick_asset_file, asset_options))

spawn_height = gymapi.Vec3(0.0, 0.3, 0.0)

# load bin asset
bin_asset_file = "urdf/tray/traybox.urdf"

print("Loading asset '%s' from '%s'" % (bin_asset_file, asset_root))
bin_asset = gym.load_asset(sim, asset_root, bin_asset_file, asset_options)

corner = table_pose.p - table_dims * 0.5

asset_root = "../../assets"
kuka_asset_file = "urdf/kuka_allegro_description/kuka_allegro.urdf"

asset_options.fix_base_link = True
asset_options.flip_visual_attachments = False
asset_options.collapse_fixed_joints = True
asset_options.disable_gravity = True

if sim_type == gymapi.SIM_FLEX:
    asset_options.max_angular_velocity = 40.

print("Loading asset '%s' from '%s'" % (kuka_asset_file, asset_root))
kuka_asset = gym.load_asset(sim, asset_root, kuka_asset_file, asset_options)

# set up the env grid
spacing = 1.5
env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
env_upper = gymapi.Vec3(spacing, spacing, spacing)

# cache some common handles for later use
envs = []
kuka_handles = []
tray_handles = []
object_handles = []

# Attractors setup
kuka_attractors = ["iiwa7_link_7"]  # , "thumb_link_3", "index_link_3", "middle_link_3", "ring_link_3"]
attractors_offsets = [gymapi.Transform(), gymapi.Transform(), gymapi.Transform(), gymapi.Transform(), gymapi.Transform()]

# Coordinates to offset attractors to tips of fingers
# thumb
attractors_offsets[1].p = gymapi.Vec3(0.07, 0.01, 0)
attractors_offsets[1].r = gymapi.Quat(0.0, 0.0, 0.216433, 0.976297)
# index, middle and ring
for i in range(2, 5):
    attractors_offsets[i].p = gymapi.Vec3(0.055, 0.015, 0)
    attractors_offsets[i].r = gymapi.Quat(0.0, 0.0, 0.216433, 0.976297)

attractor_handles = {}

print("Creating %d environments" % num_envs)
num_per_row = int(math.sqrt(num_envs))
base_poses = []

for i in range(num_envs):
    # create env
    env = gym.create_env(sim, env_lower, env_upper, num_per_row)
    envs.append(env)

    table_handle = gym.create_actor(env, table_asset, table_pose, "table", i, 0)

    x = corner.x + table_dims.x * 0.5
    y = table_dims.y + box_size + 0.01
    z = corner.z + table_dims.z * 0.5

    bin_pose.p = gymapi.Vec3(x, y, z)
    tray_handles.append(gym.create_actor(env, bin_asset, bin_pose, "bin", i, 0))

    gym.set_rigid_body_color(env, tray_handles[-1], 0, gymapi.MESH_VISUAL_AND_COLLISION, tray_color)

    for j in range(num_objects):
        x = corner.x + table_dims.x * 0.5 + np.random.rand() * 0.35 - 0.2
        y = table_dims.y + box_size * 1.2 * j - 0.05
        z = corner.z + table_dims.z * 0.5 + np.random.rand() * 0.3 - 0.15

        object_pose.p = gymapi.Vec3(x, y, z) + spawn_height

        object_asset = object_assets[0]
        if args.object_type >= 5:
            object_asset = object_assets[np.random.randint(len(object_assets))]
        else:
            object_asset = object_assets[args.object_type]

        object_handles.append(gym.create_actor(env, object_asset, object_pose, "object" + str(j), i, 0))

        if args.object_type == 2:
            color = gymapi.Vec3(banana_color.x + np.random.rand()*0.1, banana_color.y + np.random.rand()*0.05, banana_color.z)
            gym.set_rigid_body_color(env, object_handles[-1], 0, gymapi.MESH_VISUAL_AND_COLLISION, color)
        elif args.object_type == 4:
            color = gymapi.Vec3(brick_color.x + np.random.rand()*0.1, brick_color.y + np.random.rand()*0.04, brick_color.z + np.random.rand()*0.05)
            gym.set_rigid_body_color(env, object_handles[-1], 0, gymapi.MESH_VISUAL_AND_COLLISION, color)
        else:
            gym.set_rigid_body_color(env, object_handles[-1], 0, gymapi.MESH_VISUAL_AND_COLLISION, colors[j % len(colors)])

    # add kuka
    kuka_handle = gym.create_actor(env, kuka_asset, pose, "kuka", i, 1)

    attractor_handles[i] = []
    body_dict = gym.get_actor_rigid_body_dict(env, kuka_handle)
    props = gym.get_actor_rigid_body_states(env, kuka_handle, gymapi.STATE_POS)

    for j, body in enumerate(kuka_attractors):
        attractor_properties = gymapi.AttractorProperties()
        attractor_properties.stiffness = 1e6
        attractor_properties.damping = 5e2
        body_handle = gym.find_actor_rigid_body_handle(env, kuka_handle, body)
        attractor_properties.target = props['pose'][:][body_dict[body]]
        attractor_properties.target.p.y -= 0.15

        # By Default, offset pose is set to origin, so no need to set it
        if j > 0:
            attractor_properties.offset = attractors_offsets[j]
        base_poses.append(attractor_properties.target)
        if j == 0:
            # make attractor in all axes
            attractor_properties.axes = gymapi.AXIS_ALL
        else:
            # make attractor in Translation only
            attractor_properties.axes = gymapi.AXIS_TRANSLATION

        # attractor_properties.target.p.z=0.1
        attractor_properties.rigid_handle = body_handle

        gymutil.draw_lines(axes_geom, gym, viewer, env, attractor_properties.target)
        gymutil.draw_lines(sphere_geom, gym, viewer, env, attractor_properties.target)

        attractor_handle = gym.create_rigid_body_attractor(env, attractor_properties)
        attractor_handles[i].append(attractor_handle)

    kuka_handles.append(kuka_handle)

# get joint limits and ranges for kuka
kuka_dof_props = gym.get_actor_dof_properties(envs[0], kuka_handles[0])
kuka_lower_limits = kuka_dof_props['lower']
kuka_upper_limits = kuka_dof_props['upper']
kuka_ranges = kuka_upper_limits - kuka_lower_limits
kuka_mids = 0.5 * (kuka_upper_limits + kuka_lower_limits)
kuka_num_dofs = len(kuka_dof_props)

# override default stiffness and damping values
kuka_dof_props['stiffness'].fill(100.0)
kuka_dof_props['damping'].fill(100.0)

# Set base to track pose zero to maintain posture
kuka_dof_props["driveMode"][0] = gymapi.DOF_MODE_POS

for env in envs:
    gym.set_actor_dof_properties(env, kuka_handles[i], kuka_dof_props)

# a helper function to initialize all envs


def init():
    for i in range(num_envs):
        # set updated stiffness and damping properties
        gym.set_actor_dof_properties(envs[i], kuka_handles[i], kuka_dof_props)

        kuka_dof_states = gym.get_actor_dof_states(envs[i], kuka_handles[i], gymapi.STATE_NONE)
        for j in range(kuka_num_dofs):
            kuka_dof_states['pos'][j] = kuka_mids[j] - kuka_mids[j] * .5
        gym.set_actor_dof_states(envs[i], kuka_handles[i], kuka_dof_states, gymapi.STATE_POS)


def update_kuka(t):
    gym.clear_lines(viewer)
    for i in range(num_envs):
        for j in range(len(attractor_handles[i])):
            attractor_properties = gym.get_attractor_properties(envs[i], attractor_handles[i][j])
            attr_pose = copy(base_poses[j])
            attr_pose.p.z += 0.2 * np.cos(1.5*t - np.pi*float(i) / num_envs)
            if j == 0:
                attr_pose.p.x += 0.2 * np.sin(1.5*t - np.pi*float(i) / num_envs)
                attr_pose.p.y += 0.08 * np.cos(2.5*t - np.pi*float(i) / num_envs)
            elif j == 1:
                attr_pose.p.x = -0.05 + 0.2 * np.sin(1.5*t - np.pi*float(i) / num_envs)
                attr_pose.p.y += 0.05 + 0.08 * np.cos(2.5*t - np.pi*float(i) / num_envs) + 0.01 * np.cos(5*t - np.pi*j/5)
                attr_pose.p.z += 0.01 * np.cos(8*t - np.pi*j/5)
            else:
                attr_pose.p.x = -0.05 + 0.2 * np.sin(1.5*t - np.pi*float(i) / num_envs)
                attr_pose.p.y += -0.01 + 0.08 * np.cos(2.5*t - np.pi*float(i) / num_envs) + 0.01 * np.cos(5*t - np.pi*j/5)

            gym.set_attractor_target(envs[i], attractor_handles[i][j], attr_pose)
            gymutil.draw_lines(axes_geom, gym, viewer, envs[i], attr_pose)
            gymutil.draw_lines(sphere_geom, gym, viewer, envs[i], attr_pose)


init()

next_kuka_update_time = 0.1
frame = 0

while not gym.query_viewer_has_closed(viewer):
    # check if we should update
    t = gym.get_sim_time(sim)
    if t >= next_kuka_update_time:
        update_kuka(t)
        next_kuka_update_time += 0.01

    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

#    for env in envs:
#        gym.draw_env_rigid_contacts(viewer, env, colors[0], 0.5, True)

    # step rendering
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, False)
    gym.sync_frame_time(sim)

    frame = frame + 1

print("Done")

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
