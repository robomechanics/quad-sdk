"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Isaac Gym Graphics Example
--------------------------
This example demonstrates the use of several graphics operations of Isaac
Gym, including the following
- Load Textures / Create Textures from Buffer
- Apply Textures to rigid bodies
- Create Camera Sensors
  * Static location camera sensors
  * Camera sensors attached to a rigid body
- Retrieve different types of camera images

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
                                   {"name": "--save_images", "action": "store_true", "help": "Write RGB and Depth Images To Disk"}])


if args.save_images:
    from PIL import Image as im

# get default params
sim_params = gymapi.SimParams()
if args.physics_engine == gymapi.SIM_FLEX:
    sim_params.flex.shape_collision_margin = 0.04
elif args.physics_engine == gymapi.SIM_PHYSX:
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 4
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.num_threads = args.num_threads
    sim_params.physx.use_gpu = args.use_gpu

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
gym.add_ground(sim, gymapi.PlaneParams())

if not args.headless:
    # create viewer using the default camera properties
    viewer = gym.create_viewer(sim, gymapi.CameraProperties())
    if viewer is None:
        raise ValueError('*** Failed to create viewer')

# set up the env grid
num_envs = 8
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

# Load textures from file. Loads all .jpgs from the specified directory as textures
texture_files = os.listdir("../../assets/textures/")
texture_handles = []
for file in texture_files:
    if file.endswith(".jpg"):
        h = gym.create_texture_from_file(sim, os.path.join("../../assets/textures/", file))
        if h == gymapi.INVALID_HANDLE:
            print("Couldn't load texture %s" % file)
        else:
            texture_handles.append(h)

# Create a random RGB and a random grayscale texture in python arrays and
# pass those to gym as a texture
tex_dim = 128
noise_texture = 255.0 * np.random.rand(tex_dim, tex_dim*4)
texture_handles.append(gym.create_texture_from_buffer(sim, tex_dim, tex_dim, noise_texture.astype(np.uint8)))

grayscale_noise = np.zeros((tex_dim, tex_dim*4), dtype=np.uint8)
for i in range(tex_dim):
    offset = 0
    for j in range(tex_dim):
        v = np.random.randint(0, 255, dtype=np.uint8)
        grayscale_noise[i, offset] = v  # red
        grayscale_noise[i, offset + 1] = v  # green
        grayscale_noise[i, offset + 2] = v  # blue
        grayscale_noise[i, offset + 3] = 255  # apha
        offset = offset + 4
texture_handles.append(gym.create_texture_from_buffer(sim, tex_dim, tex_dim, grayscale_noise))

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
        if j == 0:
            y = 6.0
        else:
            y = 0.25
        z = d * (0.5 + np.floor(j/grid))
        actor_pose = gymapi.Transform()
        actor_pose.p = gymapi.Vec3(x, y, z)
        actor_pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107)
        asset_name = "asset_%d" % j
        handle = gym.create_actor(env, assets[j], actor_pose, asset_name, i, 1, 0)
        actor_handles[i].append(handle)


# Create 2 cameras in each environment, one which views the origin of the environment
# and one which is attached to the 0th body of the 0th actor and moves with that actor
camera_handles = [[]]
for i in range(num_envs):
    camera_handles.append([])
    camera_properties = gymapi.CameraProperties()
    camera_properties.width = 360
    camera_properties.height = 240

    # Set a fixed position and look-target for the first camera
    # position and target location are in the coordinate frame of the environment
    h1 = gym.create_camera_sensor(envs[i], camera_properties)
    camera_position = gymapi.Vec3(1.5, 1, 1.5)
    camera_target = gymapi.Vec3(0, 0, 0)
    gym.set_camera_location(h1, envs[i], camera_position, camera_target)
    camera_handles[i].append(h1)

    # Attach camera 2 to the first rigid body of the first actor in the environment, which
    # is the ball. The camera offset is relative to the position of the actor, the camera_rotation
    # is relative to the global coordinate frame, not the actor's rotation
    # In even envs cameras are will be following rigid body position and orientation,
    # in odd env only the position
    h2 = gym.create_camera_sensor(envs[i], camera_properties)
    camera_offset = gymapi.Vec3(1, 0, -1)
    camera_rotation = gymapi.Quat.from_axis_angle(gymapi.Vec3(0, 1, 0), np.deg2rad(135))
    actor_handle = gym.get_actor_handle(envs[i], 0)
    body_handle = gym.get_actor_rigid_body_handle(envs[i], actor_handle, 0)

    gym.attach_camera_to_body(h2, envs[i], body_handle, gymapi.Transform(camera_offset, camera_rotation), gymapi.FOLLOW_TRANSFORM)
    camera_handles[i].append(h2)


# Assign textures to each actor by assigning a different texture to each rigid body within each actor. With
# only balls, each actor has only one rigit body
textures_applied = 0
for i in range(num_envs):
    actor_count = gym.get_actor_count(envs[i])
    for j in range(actor_count):
        actor_handle = gym.get_actor_handle(envs[i], j)
        num_bodies = gym.get_actor_rigid_body_count(envs[i], actor_handle)
        for b in range(num_bodies):
            texture_index = np.mod(textures_applied, len(texture_handles))
            gym.set_rigid_body_texture(envs[i], actor_handle, b, gymapi.MESH_VISUAL_AND_COLLISION, texture_handles[texture_index])
            textures_applied = textures_applied + 1


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
            for j in range(2):
                # The gym utility to write images to disk is recommended only for RGB images.
                rgb_filename = "graphics_images/rgb_env%d_cam%d_frame%d.png" % (i, j, frame_count)
                gym.write_camera_image_to_file(sim, envs[i], camera_handles[i][j], gymapi.IMAGE_COLOR, rgb_filename)

                # Retrieve image data directly. Use this for Depth, Segmentation, and Optical Flow images
                # Here we retrieve a depth image, normalize it to be visible in an
                # output image and then write it to disk using Pillow
                depth_image = gym.get_camera_image(sim, envs[i], camera_handles[i][j], gymapi.IMAGE_DEPTH)

                # -inf implies no depth value, set it to zero. output will be black.
                depth_image[depth_image == -np.inf] = 0

                # clamp depth image to 10 meters to make output image human friendly
                depth_image[depth_image < -10] = -10

                # flip the direction so near-objects are light and far objects are dark
                normalized_depth = -255.0*(depth_image/np.min(depth_image + 1e-4))

                # Convert to a pillow image and write it to disk
                normalized_depth_image = im.fromarray(normalized_depth.astype(np.uint8), mode="L")
                normalized_depth_image.save("graphics_images/depth_env%d_cam%d_frame%d.jpg" % (i, j, frame_count))

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
