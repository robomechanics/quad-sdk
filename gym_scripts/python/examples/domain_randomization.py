"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Domain Randomization Example
----------------------------
An example that demonstrates domain randomization.
- Randomize color
- Randomize texture
- Randomize light parameters
- Randomize camera position
"""

import os
import random
from isaacgym import gymapi
from isaacgym import gymutil

# initialize gym
gym = gymapi.acquire_gym()

# parse arguments
args = gymutil.parse_arguments(
    description="Domain Randomization Example",
    headless=True,
    custom_parameters=[
        {"name": "--save_images", "action": "store_true", "help": "Store Images To Disk"}])

# configure sim
sim_params = gymapi.SimParams()
sim_params.substeps = 2
sim_params.dt = 1.0 / 60.0
if args.physics_engine == gymapi.SIM_FLEX:
    pass
else:
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 4
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.num_threads = args.num_threads
    sim_params.physx.use_gpu = args.use_gpu

sim_params.use_gpu_pipeline = False
if args.use_gpu_pipeline:
    print("WARNING: Forcing CPU pipeline.")

sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)

if sim is None:
    print("*** Failed to create sim")
    quit()

# add ground plane
plane_params = gymapi.PlaneParams()
gym.add_ground(sim, gymapi.PlaneParams())

# create viewer
if not args.headless:
    viewer = gym.create_viewer(sim, gymapi.CameraProperties())
    if viewer is None:
        raise ValueError('*** Failed to create viewer')

# set up the env grid
num_envs = 1
spacing = 0.75
env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
env_upper = gymapi.Vec3(spacing, spacing, spacing)

# create ant asset
asset_root = "../../assets"
asset_file = "mjcf/nv_ant.xml"
print("Loading asset '%s' from '%s'" % (asset_file, asset_root))
ant_asset = gym.load_asset(sim, asset_root, asset_file)
if ant_asset is None:
    raise IOError("Failed to load asset")

envs = []
actor_handles = []
camera_handles = []

# Load textures from file
texture_files = os.listdir("../../assets/textures/")
loaded_texture_handle_list = []
for file in texture_files:
    if file.endswith(".jpg"):
        loaded_texture_handle_list.append(gym.create_texture_from_file(sim, os.path.join("../../assets/textures/", file)))

# Sensor camera properties
cam_pos = gymapi.Vec3(0.0, 3.0, 3.0)
cam_target = gymapi.Vec3(0.0, 0.0, -1.0)
cam_props = gymapi.CameraProperties()
cam_props.width = 360
cam_props.height = 360

# Create environments
print('Creating %d environments' % num_envs)
for i in range(num_envs):
    # create env
    env = gym.create_env(sim, env_lower, env_upper, 2)
    envs.append(env)

    # create ant actor
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0, 0.5, 0)
    pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107)
    ahandle = gym.create_actor(env, ant_asset, pose, 'ant', i, 1)
    actor_handles.append(ahandle)

    # configure DOF properties to move freely
    props = gym.get_actor_dof_properties(env, ahandle)
    props["driveMode"].fill(gymapi.DOF_MODE_NONE)
    props["stiffness"].fill(0.0)
    props["damping"].fill(0.0)
    gym.set_actor_dof_properties(env, ahandle, props)

    # create camera actor
    camera_handle = gym.create_camera_sensor(env, cam_props)
    camera_handles.append(camera_handle)
    body = gym.get_actor_rigid_body_handle(env, ahandle, 0)
    gym.attach_camera_to_body(camera_handle, env, body, gymapi.Transform(p=cam_pos), gymapi.FOLLOW_TRANSFORM)
    gym.set_camera_location(camera_handle, env, cam_pos, cam_target)

# position viewer camera
if not args.headless:
    gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

sequence_number = 0
# Only create the folder if it doesn't exist
if not os.path.exists('dr_output_images'):
    os.mkdir("dr_output_images")

while True:
    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # render camera sensor
    gym.render_all_camera_sensors(sim)

    # update graphics transforms
    gym.step_graphics(sim)

    if not args.headless:
        # render the viewer
        gym.draw_viewer(viewer, sim, True)

        # Wait for dt to elapse in real time to sync viewer with
        # simulation rate. Not necessary in headless.
        gym.sync_frame_time(sim)

        # Check for exit condition - user closed the viewer window
        if gym.query_viewer_has_closed(viewer):
            break

    # randomize body color/texture to box actors and env lights every 100 frames
    if gym.get_frame_count(sim) % 100 == 0:

        for i in range(num_envs):
            env = envs[i]

            # randomize sensor camera position
            y_offset = random.uniform(-1.0, 1.0)
            z_offset = random.uniform(-1.0, 1.0)
            cam_pos_new = cam_pos + gymapi.Vec3(0., y_offset, z_offset)
            gym.set_camera_location(camera_handles[i], env, cam_pos_new, cam_target)

            # randomize colors and textures for rigid body
            num_bodies = gym.get_actor_rigid_body_count(env, actor_handles[-1])
            for n in range(num_bodies):
                gym.set_rigid_body_color(env, actor_handles[-1], n, gymapi.MESH_VISUAL,
                                         gymapi.Vec3(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)))
                gym.set_rigid_body_texture(env, actor_handles[-1], n, gymapi.MESH_VISUAL,
                                           loaded_texture_handle_list[random.randint(0, len(loaded_texture_handle_list)-1)])

            # randomize light parameters
            l_color = gymapi.Vec3(random.uniform(1, 1), random.uniform(1, 1), random.uniform(1, 1))
            l_ambient = gymapi.Vec3(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
            l_direction = gymapi.Vec3(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
            gym.set_light_parameters(sim, 0, l_color, l_ambient, l_direction)

            # save rgb image to disk
            if args.save_images:
                print("writing dr_output_images/rgb_image_%03d_%03d.png" % (sequence_number,  i))
                rgb_image_filename = "dr_output_images/rgb_image_%03d_%03d.png" % (sequence_number,  i)
                gym.write_camera_image_to_file(sim, env, camera_handle, gymapi.IMAGE_COLOR, rgb_image_filename)

        sequence_number = sequence_number + 1

print('Done')

if not args.headless:
    gym.destroy_viewer(viewer)

gym.destroy_sim(sim)
