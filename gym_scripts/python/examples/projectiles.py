"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Projectiles Example
-------------------
An example which shows how to spawn and move assets, illustrates
collision filtering, and how to use the viewer to interact with
the physics simulation.
"""

import numpy as np
from isaacgym import gymutil
from isaacgym import gymapi
from math import sqrt

# initialize gym
gym = gymapi.acquire_gym()

# parse arguments
args = gymutil.parse_arguments(
    description="Projectiles Example: Press SPACE to fire a projectile. Press R to reset the simulation.",
    custom_parameters=[
        {"name": "--num_envs", "type": int, "default": 16, "help": "Number of environments to create"}])

# configure sim
sim_params = gymapi.SimParams()
if args.physics_engine == gymapi.SIM_FLEX:
    sim_params.flex.shape_collision_margin = 0.05
    sim_params.flex.num_inner_iterations = 6
elif args.physics_engine == gymapi.SIM_PHYSX:
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
gym.add_ground(sim, plane_params)

# create viewer. Not optional in this example
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    print("*** Failed to create viewer")
    quit()

# subscribe to input events. This allows input to be used to interact
# with the simulation
gym.subscribe_viewer_keyboard_event(viewer, gymapi.KEY_SPACE, "space_shoot")
gym.subscribe_viewer_keyboard_event(viewer, gymapi.KEY_R, "reset")
gym.subscribe_viewer_mouse_event(viewer, gymapi.MOUSE_LEFT_BUTTON, "mouse_shoot")

# load asset
asset_root = "../../assets"
asset_file = "mjcf/nv_ant.xml"

asset_options = gymapi.AssetOptions()
asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

# set up the grid of environments
num_envs = args.num_envs
num_per_row = int(sqrt(num_envs))
spacing = 2.0

envs = []
actor_handles = []

for i in range(num_envs):
    # create environments
    lower = gymapi.Vec3(-spacing, 0.0, -spacing)
    upper = gymapi.Vec3(spacing, spacing, spacing)
    env = gym.create_env(sim, lower, upper, num_per_row)
    envs.append(env)

    # add actor to environment
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 1.0, 0.0)
    pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107)
    ahandle = gym.create_actor(env, asset, pose, "ant", i, 1)
    actor_handles.append(ahandle)

    # override default DOF properties loaded from MJCF to make the legs move more freely
    props = gym.get_actor_dof_properties(env, ahandle)
    props["driveMode"].fill(gymapi.DOF_MODE_NONE)
    props["stiffness"].fill(0.0)
    props["damping"].fill(0.0)
    gym.set_actor_dof_properties(env, ahandle, props)

# create an extra env to host the projectiles and create
# 20 projectiles which will be cycled
proj_env = gym.create_env(sim, lower, upper, 4)
proj_asset_options = gymapi.AssetOptions()
proj_asset_options.density = 10.
proj_asset = gym.create_box(sim, 0.3, 0.3, 0.3, proj_asset_options)
projectiles = []

for i in range(20):
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(i * 0.5, 1.0, 50)
    pose.r = gymapi.Quat(0, 0, 0, 1)

    # create actors which will collide with actors in any environment
    ahandle = gym.create_actor(proj_env, proj_asset, pose, "projectile" + str(i), -1, 0)

    # set each projectile to a different, random color
    c = 0.5 + 0.5 * np.random.random(3)
    gym.set_rigid_body_color(proj_env, ahandle, 0, gymapi.MESH_VISUAL_AND_COLLISION, gymapi.Vec3(c[0], c[1], c[2]))

    projectiles.append(ahandle)

# save initial state for reset
initial_state = np.copy(gym.get_sim_rigid_body_states(sim, gymapi.STATE_ALL))

proj_index = 0

while not gym.query_viewer_has_closed(viewer):

    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # Get input actions from the viewer and handle them appropriately
    for evt in gym.query_viewer_action_events(viewer):

        if evt.action == "reset" and evt.value > 0:
            gym.set_sim_rigid_body_states(sim, initial_state, gymapi.STATE_ALL)

        elif (evt.action == "space_shoot" or evt.action == "mouse_shoot") and evt.value > 0:
            if evt.action == "mouse_shoot":
                pos = gym.get_viewer_mouse_position(viewer)
                window_size = gym.get_viewer_size(viewer)
                xcoord = round(pos.x * window_size.x)
                ycoord = round(pos.y * window_size.y)
                print(f"Fired projectile with mouse at coords: {xcoord} {ycoord}")

            cam_pose = gym.get_viewer_camera_transform(viewer, proj_env)
            cam_fwd = cam_pose.r.rotate(gymapi.Vec3(0, 0, 1))

            spawn = cam_pose.p
            speed = 25
            vel = cam_fwd * speed

            angvel = 1.57 - 3.14 * np.random.random(3)

            proj_handle = projectiles[proj_index]
            state = gym.get_actor_rigid_body_states(proj_env, proj_handle, gymapi.STATE_NONE)
            state['pose']['p'].fill((spawn.x, spawn.y, spawn.z))
            state['pose']['r'].fill((0, 0, 0, 1))
            state['vel']['linear'].fill((vel.x, vel.y, vel.z))
            state['vel']['angular'].fill((angvel[0], angvel[1], angvel[2]))
            gym.set_actor_rigid_body_states(proj_env, proj_handle, state, gymapi.STATE_ALL)

            proj_index = (proj_index + 1) % len(projectiles)

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, False)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
