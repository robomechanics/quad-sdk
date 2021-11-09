"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Visualize Transforms
--------------------
Compute and visualize relative transforms for different locations on a cabinet
- Compute transforms on different rigid bodies on cabinet with offsets
- Visualize transforms using wireframe axes/sphere
"""

import os
import math
from isaacgym import gymapi
from isaacgym import gymutil

# Initialize gym
gym = gymapi.acquire_gym()

# Parse arguments
args = gymutil.parse_arguments(description="Visualize Transforms")

# configure sim
sim_params = gymapi.SimParams()
sim_params.gravity = gymapi.Vec3(0.0, -9.8, 0.0)
sim_params.dt = 1.0 / 60.0
sim_params.substeps = 2
if args.physics_engine == gymapi.SIM_FLEX:
    sim_params.flex.solver_type = 5
    sim_params.flex.num_outer_iterations = 4
    sim_params.flex.num_inner_iterations = 15
    sim_params.flex.relaxation = 0.75
    sim_params.flex.warm_start = 0.5
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

# Add ground plane
plane_params = gymapi.PlaneParams()
gym.add_ground(sim, plane_params)

# Create viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    print("*** Failed to create viewer")
    quit()

# Load asset relative to the current working directory
print("Working directory: %s" % os.getcwd())
asset_root = "../../assets"
asset_file = "urdf/sektion_cabinet_model/urdf/sektion_cabinet.urdf"
asset_options = gymapi.AssetOptions()
asset_options.armature = 0.01
# Fix the base link of the cabinet so it does not move
asset_options.fix_base_link = True
asset_options.use_mesh_materials = True
print("Loading asset '%s' from '%s'" % (asset_file, asset_root))
asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

# Set up the environment grid with two environments
num_envs = 2
spacing = 1.0
env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
env_upper = gymapi.Vec3(spacing, spacing, spacing)


class Cabinet:

    DRAWER_GRASP = gymapi.Vec3(0.3, 0.0, 0.01)
    LEFT_DOOR_GRASP = gymapi.Vec3(0.03, 0.35, 0.185)
    RIGHT_DOOR_GRASP = gymapi.Vec3(0.03, -0.35, 0.185)

    TOP_DRAWER_INDEX = 8
    BOTTOM_DRAWER_INDEX = 6
    RIGHT_DOOR_INDEX = 4
    LEFT_DOOR_INDEX = 2

    def __init__(self, env, actor):
        self.env = env
        self.actor = actor

    def get_grasp_points(self):

        poses = gym.get_actor_rigid_body_states(self.env, self.actor, gymapi.STATE_POS)['pose']

        # Get pose for all of the handles
        top_drawer_handle_pose = gymapi.Transform.from_buffer(poses[self.TOP_DRAWER_INDEX])
        bottom_drawer_handle_pose = gymapi.Transform.from_buffer(poses[self.BOTTOM_DRAWER_INDEX])
        left_door_handle_pose = gymapi.Transform.from_buffer(poses[self.LEFT_DOOR_INDEX])
        right_door_handle_pose = gymapi.Transform.from_buffer(poses[self.RIGHT_DOOR_INDEX])

        # Offset drawer transforms to compute grasp locations
        top_drawer_point = top_drawer_handle_pose.transform_point(self.DRAWER_GRASP)
        bottom_drawer_point = bottom_drawer_handle_pose.transform_point(self.DRAWER_GRASP)
        left_door_handle_point = left_door_handle_pose.transform_point(self.LEFT_DOOR_GRASP)
        right_door_handle_point = right_door_handle_pose.transform_point(self.RIGHT_DOOR_GRASP)

        # Create transform from grasp location and handle rotation
        top_drawer_grasp = gymapi.Transform(top_drawer_point, top_drawer_handle_pose.r)
        bottom_drawer_grasp = gymapi.Transform(bottom_drawer_point, bottom_drawer_handle_pose.r)
        left_door_handle_grasp = gymapi.Transform(left_door_handle_point, left_door_handle_pose.r)
        right_door_handle_grasp = gymapi.Transform(right_door_handle_point, right_door_handle_pose.r)

        return top_drawer_grasp, bottom_drawer_grasp, left_door_handle_grasp, right_door_handle_grasp


cabinets = []

print("Creating %d environments" % num_envs)
for i in range(num_envs):
    # Create env
    env = gym.create_env(sim, env_lower, env_upper, 1)

    # Add cabinet model
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 0.41, 0.0)
    pose.r = gymapi.Quat.from_euler_zyx(-0.5 * math.pi, 0, 0)
    ahandle = gym.create_actor(env, asset, pose, "cabinet", i, 1)

    # Cache rigid body handles in cabinet actor to refer to later
    cab = Cabinet(env, ahandle)
    dof_props = gym.get_actor_dof_properties(env, ahandle)

    # Set stiffness and damping of joint drives
    dof_props['stiffness'].fill(1000000.0)
    dof_props['damping'].fill(500.0)
    dof_props["driveMode"] = gymapi.DOF_MODE_POS
    gym.set_actor_dof_properties(env, ahandle, dof_props)
    cabinets.append(cab)

# Look at the first env
cam_pos = gymapi.Vec3(3, 1.5, 3)
cam_target = gymapi.Vec3(0, 0.5, 0)
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

# Create helper geometry used for visualization
# Create an wireframe axis
axes_geom = gymutil.AxesGeometry(0.1)
# Create a wireframe sphere
sphere_rot = gymapi.Quat.from_euler_zyx(0.5 * math.pi, 0, 0)
sphere_pose = gymapi.Transform(r=sphere_rot)
sphere_geom = gymutil.WireframeSphereGeometry(0.02, 12, 12, sphere_pose, color=(1, 1, 0))

while not gym.query_viewer_has_closed(viewer):

    # Step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    gym.clear_lines(viewer)

    for i in range(num_envs):
        # Get cabinet from environment, each environment only has one cabinet
        cab = cabinets[i]

        # Get the transforms we want to visualize
        top_drawer_grasp, bottom_drawer_grasp, left_door_handle_grasp, right_door_handle_grasp = cab.get_grasp_points()

        # Top drawer
        gymutil.draw_lines(axes_geom, gym, viewer, cab.env, top_drawer_grasp)
        gymutil.draw_lines(sphere_geom, gym, viewer, cab.env, top_drawer_grasp)

        # Bottom drawer
        gymutil.draw_lines(axes_geom, gym, viewer, cab.env, bottom_drawer_grasp)
        gymutil.draw_lines(sphere_geom, gym, viewer, cab.env, bottom_drawer_grasp)

        # Left door
        gymutil.draw_lines(axes_geom, gym, viewer, cab.env, left_door_handle_grasp)
        gymutil.draw_lines(sphere_geom, gym, viewer, cab.env, left_door_handle_grasp)

        # Right door
        gymutil.draw_lines(axes_geom, gym, viewer, cab.env, right_door_handle_grasp)
        gymutil.draw_lines(sphere_geom, gym, viewer, cab.env, right_door_handle_grasp)

    # Update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)

print("Exiting")
gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
