"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Isaac Gym Graphics Example
--------------------------
This example demonstrates the ability to change the up axis used in Isaac Gym.
The default option is to set Y as up axis. Using gymapi.UpAxis.UP_AXIS_Z, we can
change orientation such that Z is up for both the Viewer and camera sensors.

Requires Pillow (formerly PIL) to write images from python. Use `pip install pillow`
 to get Pillow.
"""


import os
import numpy as np
from isaacgym import gymapi
from isaacgym import gymutil

# acquire the gym interface
gym = gymapi.acquire_gym()

# parse arguments
args = gymutil.parse_arguments(description="Graphics Example",
                               headless=True,
                               custom_parameters=[
                                   {"name": "--save_images", "action": "store_true", "help": "Write RGB and Depth Images To Disk"},
                                   {"name": "--up_axis_z", "action": "store_true", "help": ""}])

if args.save_images:
    from PIL import Image as im

# get default params
sim_params = gymapi.SimParams()
if args.up_axis_z:
    sim_params.up_axis = gymapi.UpAxis.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)
sim_params.flex.shape_collision_margin = 0.04

sim_params.use_gpu_pipeline = False
if args.use_gpu_pipeline:
    print("WARNING: Forcing CPU pipeline.")

# create sim
sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)
if sim is None:
    print("*** Failed to create sim")
    quit()

# Create a default ground plane
plane_params = gymapi.PlaneParams()
if args.up_axis_z:
    plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
plane_params.distance = 2.0
gym.add_ground(sim, plane_params)

if not args.headless:
    # create viewer using the default camera properties
    viewer = gym.create_viewer(sim, gymapi.CameraProperties())
    if viewer is None:
        raise ValueError('*** Failed to create viewer')

# set up the env grid
num_envs = 1
spacing = 2.0
env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
env_upper = gymapi.Vec3(spacing, spacing, spacing)

# list of assets which are used in this example
repeat_assets = 8
asset_files = []
asset_root = "../../assets"
for i in range(repeat_assets):
    asset_files.append("urdf/ball.urdf")

# Load all assets
assets = []
for i in range(len(asset_files)):
    asset_handle = gym.load_asset(sim, asset_root, asset_files[i])
    assets.append(asset_handle)

# Create environments
actor_handles = [[]]
envs = []

# create environments
for i in range(num_envs):
    actor_handles.append([])
    env = gym.create_env(sim, env_lower, env_upper, 4)
    envs.append(env)
    num_assets = len(assets)
    for j in range(num_assets):
        # Lay out assets in a grid within the environment. Start actor 0 at a height of 10 so
        # the camera attachment is apparent in output images
        grid = np.ceil(np.sqrt(num_assets))
        d = 2 * spacing / grid
        x = d * (0.5 + np.mod(j, grid))
        y = 6.0
        z = d * (0.5 + np.floor(j/grid))
        actor_pose = gymapi.Transform()
        if sim_params.up_axis == gymapi.UpAxis.UP_AXIS_Z:
            actor_pose.p = gymapi.Vec3(x, z, y)
        else:
            actor_pose.p = gymapi.Vec3(x, y, z)
        actor_pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107)
        asset_name = "asset_%d" % j
        handle = gym.create_actor(env, assets[j], actor_pose, asset_name, i, 1, 0)
        actor_handles[i].append(handle)


# Create 2 cameras in each environment, one which views the origin of the environment
# and one which is attached to the 0th body of the 0th actor and moves with that actor
if sim_params.up_axis == gymapi.UpAxis.UP_AXIS_Z:
    camera_position = gymapi.Vec3(4.0, 14.0, 10.0)
else:
    camera_position = gymapi.Vec3(14.0, 10.0, 4.0)
camera_target = gymapi.Vec3(0, 0, 0)
gym.viewer_camera_look_at(viewer, None, camera_position, camera_target)

camera_handles = [[]]
for i in range(num_envs):
    camera_handles.append([])
    camera_properties = gymapi.CameraProperties()
    camera_properties.width = 360
    camera_properties.height = 240

    # Set a fixed position and look-target for the first camera
    # position and target location are in the coordinate frame of the environment
    h1 = gym.create_camera_sensor(envs[i], camera_properties)
    gym.set_camera_location(h1, envs[i], camera_position, camera_target)
    camera_handles[i].append(h1)

if not os.path.exists("graphics_images"):
    os.mkdir("graphics_images")
frame_count = 0

# Main simulation loop
while True:
    # step the physics simulation
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # communicate physics to graphics system
    gym.step_graphics(sim)

    # render the camera sensors
    gym.render_all_camera_sensors(sim)

    if args.save_images and np.mod(frame_count, 30) == 0:
        for i in range(num_envs):
            for j in range(1):
                # The gym utility to write images to disk is recommended only for RGB images.
                rgb_filename = "graphics_images/rgb_env%d_cam%d_frame%d.png" % (i, j, frame_count)
                gym.write_camera_image_to_file(sim, envs[i], camera_handles[i][j], gymapi.IMAGE_COLOR, rgb_filename)

    if not args.headless:
        # render the viewer
        gym.draw_viewer(viewer, sim, True)

        # Wait for dt to elapse in real time to sync viewer with
        # simulation rate. Not necessary in headless.
        gym.sync_frame_time(sim)

        # Check for exit condition - user closed the viewer window
        if gym.query_viewer_has_closed(viewer):
            break

    frame_count = frame_count + 1

print('Done')

# cleanup
gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
