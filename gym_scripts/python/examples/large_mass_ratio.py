"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Large mass-ratio test
-------------------------------
A test of the simulation stability stacks of boxes with large mass ratio. There are two scenarios available:
- Same size boxes, the next level boxes are 10 times heavier compared to the box it is standing on
- Inverted pyramid of boxes, they have the same density but the next level boxes are 4 times larger and 4*4*4 = 64 heavier than previous level one 
"""

from isaacgym import gymutil
from isaacgym import gymapi
from math import sqrt

# initialize gym
gym = gymapi.acquire_gym()

# parse arguments
args = gymutil.parse_arguments(
    description="Large Mass Ratio Test",
    custom_parameters=[
        {"name": "--num_envs", "type": int, "default": 100, "help": "Number of environments to create"},
        {"name": "--inverted_pyramid_test", "action": "store_true", "help": "Run test with stack of boxes of increasing sizes but the same density"}])

inverted_pyramid_test = args.inverted_pyramid_test

# configure sim
sim_params = gymapi.SimParams()
if args.physics_engine == gymapi.SIM_FLEX:
    sim_params.substeps = 6
    sim_params.flex.num_inner_iterations = 5
    sim_params.flex.num_inner_iterations = 24
    sim_params.flex.relaxation = 0.7
    sim_params.flex.warm_start = 0.25
    displacement = 0.02
elif args.physics_engine is gymapi.SIM_PHYSX:
    sim_params.substeps = 6
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 100
    sim_params.physx.num_velocity_iterations = 0
    sim_params.physx.num_threads = args.num_threads
    sim_params.physx.use_gpu = args.use_gpu
    sim_params.physx.rest_offset = 0.001
    displacement = 0.002

sim_params.use_gpu_pipeline = False
if args.use_gpu_pipeline:
    print("WARNING: Forcing CPU pipeline.")

sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)

if sim is None:
    print("*** Failed to create sim")
    quit()

# create viewer using the default camera properties
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    raise ValueError('*** Failed to create viewer')

# add ground plane
plane_params = gymapi.PlaneParams()
gym.add_ground(sim, gymapi.PlaneParams())

# set up the env grid
num_envs = args.num_envs
num_per_row = int(sqrt(num_envs))
spacing = 4.5
if inverted_pyramid_test:
    spacing = 12.0

# look at the first env
cam_pos = gymapi.Vec3(8.0, 6.5, 3.0)
cam_target = gymapi.Vec3(10.0, 6.0, 10.0)
if not inverted_pyramid_test:
    cam_pos.y = 3.0
    cam_target.y = 1.9

gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

box_color = gymapi.Vec3(0.1, 0.9, 0.05)
env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
env_upper = gymapi.Vec3(spacing, spacing, spacing)

# create list to mantain environment and asset handles
envs = []
box_handles = []
box_sizes = []
actor_handles = []

# create box assets w/ varying densities (measured in kg/m^3)
box_size = 0.5
box_density = 1

# AssetOptions enables loading assets with different properties
# Properties include density, angularDamping, maxAngularVelocity, linearDamping, maxLinearVelocity etc.
asset_options = gymapi.AssetOptions()
asset_options.thickness = 0.005

if not inverted_pyramid_test:
    stack_height = 5

    for dx in range(stack_height):
        asset_options.density = box_density
        asset_box = gym.create_box(sim, box_size, box_size, box_size, asset_options)
        box_handles.append(asset_box)
        box_density *= 10

    for i in range(num_envs):
        # create env
        env = gym.create_env(sim, env_lower, env_upper, num_per_row)
        envs.append(env)

        box_color = gymapi.Vec3(0.2, 0.75, 0.18)
        color_step = box_color.y / (stack_height - 1)

        # add moving boxes to env
        for b in range(stack_height):
            name = 'box_{}'.format(b)
            actor_handles.append(gym.create_actor(env, box_handles[b], gymapi.Transform(
                p=gymapi.Vec3(0., 0.5 * box_size + (box_size + displacement) * b + 0.001, 0.)), name, i, 0))
            gym.set_rigid_body_color(env, actor_handles[-1], 0, gymapi.MESH_VISUAL, box_color)
            box_color.x += 0.85 * color_step * b
            box_color.y -= color_step * b
            box_color.z -= 0.04 * b
else:
    stack_height = 5
    scale_factor = 2
    color_step = box_color.x / (stack_height - 1)

    new_box_size = box_size
    for dx in range(stack_height):
        asset_options.density = 100
        asset_box = gym.create_box(sim, new_box_size, new_box_size, new_box_size, asset_options)
        box_handles.append(asset_box)
        box_sizes.append(new_box_size)
        new_box_size *= scale_factor

    print('Creating %d environments' % num_envs)
    for i in range(num_envs):
        # create env
        env = gym.create_env(sim, env_lower, env_upper, num_per_row)
        envs.append(env)

        start_height = 0.001

        box_color = gymapi.Vec3(0.9, 0.25, 0.15)
        color_step = box_color.x / (stack_height - 1)

        # add moving boxes to env
        for b in range(stack_height):
            name = 'box_{}'.format(b)
            start_height += (0.5 * box_sizes[b] + displacement)
            actor_handles.append(gym.create_actor(env, box_handles[b], gymapi.Transform(p=gymapi.Vec3(0., start_height, 0.)), name, i, 0))

            box_color.x -= 0.9 * color_step * b
            box_color.y -= 0.02 * color_step * b
            box_color.z += 0.8 * color_step * b
            gym.set_rigid_body_color(env, actor_handles[-1], 0, gymapi.MESH_VISUAL, box_color)

            start_height += 0.5 * box_sizes[b]

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

print('Done')

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
