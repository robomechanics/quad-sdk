"""
Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Terrain examples
-------------------------
Demonstrates the use terrain meshes.
Press 'R' to reset the  simulation
"""

import numpy as np
from numpy.random import choice
from numpy.random.mtrand import triangular
from scipy import interpolate
import os

from isaacgym import gymutil, gymapi
from isaacgym.terrain_utils import *
from math import sqrt
import math

def clamp(x, min_value, max_value):
    return max(min(x, max_value), min_value)

# initialize gym
gym = gymapi.acquire_gym()

# parse arguments
args = gymutil.parse_arguments()

# configure sim
sim_params = gymapi.SimParams()
sim_params.up_axis = gymapi.UpAxis.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)

if args.physics_engine == gymapi.SIM_FLEX:
    print("WARNING: Terrain creation is not supported for Flex! Switching to PhysX")
    args.physics_engine = gymapi.SIM_PHYSX
sim_params.substeps = 2
sim_params.physx.solver_type = 1
sim_params.physx.num_position_iterations = 4
sim_params.physx.num_velocity_iterations = 0
sim_params.physx.num_threads = args.num_threads
sim_params.physx.use_gpu = args.use_gpu

sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)
if sim is None:
    print("*** Failed to create sim")
    quit()

# load ball asset
# asset_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, "assets")
# asset_file = "urdf/ball.urdf"
# asset = gym.load_asset(sim, asset_root, asset_file, gymapi.AssetOptions())
asset_root = "../../assets"
asset_file = "urdf/spirit_description/urdf/spirit.urdf"
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True
# asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
asset_options.use_mesh_materials = True
print("Loading asset '%s' from '%s'" % (asset_file, asset_root))
asset = gym.load_asset(sim, asset_root, asset_file, gymapi.AssetOptions())

# set up the env grid
num_envs = 4
spacing = 1.5
env_lower = gymapi.Vec3(5.0, 0.0, 5.0)
env_upper = gymapi.Vec3(8.0, 0.0, 8.0)
initial_pose = gymapi.Transform()
initial_pose.p = gymapi.Vec3(0.0, 2.0, 0.0)
initial_pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107)

envs = []
actor_handles = []
for i in range(num_envs):
    # create env
    env = gym.create_env(sim, env_lower, env_upper, 2)
    # spirit = gym.create_actor(env, asset, initial_pose, 'spirit', i/2, i%2)
    props = gym.get_asset_dof_properties(asset)
    num_dofs = gym.get_asset_dof_count(asset)

    dof_states = np.zeros(num_dofs, dtype=gymapi.DofState.dtype)
    dof_types = [gym.get_asset_dof_type(asset, i) for i in range(num_dofs)]
    dof_positions = dof_states['pos']
    stiffnesses = props['stiffness']
    dampings = props['damping']
    armatures = props['armature']
    has_limits = props['hasLimits']
    lower_limits = props['lower']
    upper_limits = props['upper']

    # initialize default positions, limits, and speeds (make sure they are in reasonable ranges)
    defaults = np.zeros(num_dofs)
    speeds = np.zeros(num_dofs)
    for i in range(num_dofs):
        if has_limits[i]:
            if dof_types[i] == gymapi.DOF_ROTATION:
                lower_limits[i] = clamp(lower_limits[i], -math.pi, math.pi)
                upper_limits[i] = clamp(upper_limits[i], -math.pi, math.pi)
            # make sure our default position is in range
            if lower_limits[i] > 0.0:
                defaults[i] = lower_limits[i]
            elif upper_limits[i] < 0.0:
                defaults[i] = upper_limits[i]
        else:
            # set reasonable animation limits for unlimited joints
            if dof_types[i] == gymapi.DOF_ROTATION:
                # unlimited revolute joint
                lower_limits[i] = -math.pi
                upper_limits[i] = math.pi
            elif dof_types[i] == gymapi.DOF_TRANSLATION:
                # unlimited prismatic joint
                lower_limits[i] = -1.0
                upper_limits[i] = 1.0
    # set DOF position to default
    dof_positions[i] = defaults[i]
    # set speed depending on DOF type and range of motion
    if dof_types[i] == gymapi.DOF_ROTATION:
        speeds[i] = clamp(2 * (upper_limits[i] - lower_limits[i]), 0.25 * math.pi, 3.0 * math.pi)
    else:
        speeds[i] = clamp(2 * (upper_limits[i] - lower_limits[i]), 0.1, 7.0)


    actor_handle = gym.create_actor(env, asset, initial_pose, "actor", i, 1)
    actor_handles.append(actor_handle)

    # set default DOF positions
    gym.set_actor_dof_states(env, actor_handle, dof_states, gymapi.STATE_ALL)
    envs.append(env)

    

# create a local copy of initial state, which we can send back for reset
initial_state = np.copy(gym.get_sim_rigid_body_states(sim, gymapi.STATE_ALL))

# create all available terrain types
num_terains = 6
terrain_width = 12.
terrain_length = 12.
horizontal_scale = 0.25  # [m]
vertical_scale = 0.005  # [m]
num_rows = int(terrain_width/horizontal_scale)
num_cols = int(terrain_length/horizontal_scale)
heightfield = np.zeros((num_terains*num_rows, num_cols), dtype=np.int16)


def new_sub_terrain(): return SubTerrain(width=num_rows, length=num_cols, vertical_scale=vertical_scale, horizontal_scale=horizontal_scale)


# heightfield[0:num_rows, :] = random_uniform_terrain(new_sub_terrain(), min_height=-0.2, max_height=0.2, step=0.2, downsampled_scale=0.5).height_field_raw
# heightfield[num_rows:2*num_rows, :] = sloped_terrain(new_sub_terrain(), slope=-0.5).height_field_raw
# heightfield[2*num_rows:3*num_rows, :] = pyramid_sloped_terrain(new_sub_terrain(), slope=-0.5).height_field_raw
# heightfield[3*num_rows:4*num_rows, :] = discrete_obstacles_terrain(new_sub_terrain(), max_height=0.5, min_size=1., max_size=5., num_rects=20).height_field_raw
# heightfield[4*num_rows:5*num_rows, :] = wave_terrain(new_sub_terrain(), num_waves=2., amplitude=1.).height_field_raw
# heightfield[5*num_rows:6*num_rows, :] = stairs_terrain(new_sub_terrain(), step_width=0.75, step_height=-0.5).height_field_raw
# heightfield[6*num_rows:7*num_rows, :] = pyramid_stairs_terrain(new_sub_terrain(), step_width=0.75, step_height=-0.5).height_field_raw
# heightfield[7*num_rows:8*num_rows, :] = stepping_stones_terrain(new_sub_terrain(), stone_size=1.,
#                                                                 stone_distance=1., max_height=0.5, platform_size=0.).height_field_raw
heightfield[0:num_rows, :] = random_uniform_terrain(new_sub_terrain(), min_height=-0.2, max_height=0.2, step=0.2, downsampled_scale=0.5).height_field_raw
heightfield[1*num_rows:2*num_rows, :] = pyramid_sloped_terrain(new_sub_terrain(), slope=-0.5).height_field_raw
heightfield[2*num_rows:3*num_rows, :] = discrete_obstacles_terrain(new_sub_terrain(), max_height=0.5, min_size=1., max_size=5., num_rects=20).height_field_raw
heightfield[3*num_rows:4*num_rows, :] = wave_terrain(new_sub_terrain(), num_waves=2., amplitude=0.5).height_field_raw
heightfield[4*num_rows:5*num_rows, :] = pyramid_stairs_terrain(new_sub_terrain(), step_width=0.75, step_height=-0.5).height_field_raw
#heightfield[5*num_rows:6*num_rows, :] = stepping_stones_terrain(new_sub_terrain(), stone_size=1.,
 #                                                               stone_distance=1., max_height=0.5, platform_size=0.).height_field_raw
heightfield[5*num_rows:6*num_rows, :] = stairs_terrain(new_sub_terrain(), step_width=0.75, step_height=0.5).height_field_raw

# add the terrain as a triangle mesh
vertices, triangles = convert_heightfield_to_trimesh(heightfield, horizontal_scale=horizontal_scale, vertical_scale=vertical_scale, slope_threshold=1.5)
tm_params = gymapi.TriangleMeshParams()
tm_params.nb_vertices = vertices.shape[0]
tm_params.nb_triangles = triangles.shape[0]
tm_params.transform.p.x = -1.
tm_params.transform.p.y = -1.
gym.add_triangle_mesh(sim, vertices.flatten(), triangles.flatten(), tm_params)

# create viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    print("*** Failed to create viewer")
    quit()

cam_pos = gymapi.Vec3(-5, -5, 15)
cam_target = gymapi.Vec3(0, 0, 10)
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

# subscribe to spacebar event for reset
gym.subscribe_viewer_keyboard_event(viewer, gymapi.KEY_R, "reset")
current_dof = 0
dt = 1.0 / 60.0

ANIM_SEEK_LOWER = 1
ANIM_SEEK_UPPER = 2
ANIM_SEEK_DEFAULT = 3
ANIM_FINISHED = 4

# initialize animation state
anim_state = ANIM_SEEK_LOWER
while not gym.query_viewer_has_closed(viewer):

    # Get input actions from the viewer and handle them appropriately
    for evt in gym.query_viewer_action_events(viewer):
        if evt.action == "reset" and evt.value > 0:
            gym.set_sim_rigid_body_states(sim, initial_state, gymapi.STATE_ALL)

    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    speed = speeds[current_dof]

    if anim_state == ANIM_SEEK_LOWER:
            dof_positions[current_dof] -= speed * dt
            if dof_positions[current_dof] <= lower_limits[current_dof]:
                dof_positions[current_dof] = lower_limits[current_dof]
                anim_state = ANIM_SEEK_UPPER
    elif anim_state == ANIM_SEEK_UPPER:
            dof_positions[current_dof] += speed * dt
            if dof_positions[current_dof] >= upper_limits[current_dof]:
                dof_positions[current_dof] = upper_limits[current_dof]
                anim_state = ANIM_SEEK_DEFAULT
    if anim_state == ANIM_SEEK_DEFAULT:
            dof_positions[current_dof] -= speed * dt
            if dof_positions[current_dof] <= defaults[current_dof]:
                dof_positions[current_dof] = defaults[current_dof]
                anim_state = ANIM_FINISHED
    elif anim_state == ANIM_FINISHED:
            dof_positions[current_dof] = defaults[current_dof]
            current_dof = (current_dof + 1) % num_dofs
            anim_state = ANIM_SEEK_LOWER
            # print("Animating DOF %d ('%s')" % (current_dof, dof_names[current_dof]))
        
    for i in range(num_envs):
        gym.set_actor_dof_states(envs[i], actor_handles[i], dof_states, gymapi.STATE_POS)

            # if args.show_axis:
                # get the DOF frame (origin and axis)
        dof_handle = gym.get_actor_dof_handle(envs[i], actor_handles[i], current_dof)
        frame = gym.get_dof_frame(envs[i], dof_handle)

                    # draw a line from DOF origin along the DOF axis
        p1 = frame.origin
        p2 = frame.origin + frame.axis * 0.7
        color = gymapi.Vec3(1.0, 0.0, 0.0)
        gymutil.draw_line(p1, p2, color, gym, viewer, envs[i])

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
