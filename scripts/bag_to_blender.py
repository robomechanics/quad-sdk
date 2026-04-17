#!/usr/bin/env python3
r"""Import Go2 robot meshes into Blender and animate from extracted bag data.

Usage:
    blender --background --python bag_to_blender.py -- \
        --json recording.json \
        --urdf_meshes /path/to/go2_description/models/go2/meshes \
        [--terrain /path/to/terrain.stl] \
        [--output render.blend] \
        [--render /path/to/output_frames/]

This script:
  1. Builds the Go2 kinematic tree using Empty objects
  2. Imports DAE visual meshes and parents them to the empties
  3. Optionally imports a terrain STL
  4. Reads the JSON produced by extract_bag.py and keyframes body pose + joints
     via forward kinematics (computing world transforms directly)
  5. Saves a .blend file (and optionally renders frames)
"""

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import bpy
import mathutils

# ---------------------------------------------------------------------------
# Go2 kinematic constants (from const.xacro)
# ---------------------------------------------------------------------------
LEG_OFFSET_X = 0.1934
LEG_OFFSET_Y = 0.0465
THIGH_OFFSET = 0.0955
THIGH_LENGTH = 0.213
CALF_LENGTH = 0.213

# Per-leg: (name, mirror, front_hind)
# mirror:  1 = left (+Y), -1 = right (-Y)
# front_hind:  1 = front (+X), -1 = rear (-X)
LEGS = [
    ("FR", -1,  1),
    ("FL",  1,  1),
    ("RR", -1, -1),
    ("RL",  1, -1),
]

PI = math.pi


# ---------------------------------------------------------------------------
# DAE parser
# ---------------------------------------------------------------------------
def parse_dae(filepath):
    """Parse a COLLADA .dae file and return list of (verts, faces, color)."""
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    tree = ET.parse(filepath)
    root = tree.getroot()

    mat_colors = {}
    for effect in root.findall(".//c:library_effects/c:effect", ns):
        eid = effect.get("id", "")
        diff = effect.find(".//c:diffuse/c:color", ns)
        if diff is not None:
            vals = [float(v) for v in diff.text.split()]
            mat_colors[eid] = tuple(vals[:4])

    results = []
    for geom in root.findall(".//c:library_geometries/c:geometry", ns):
        mesh_el = geom.find("c:mesh", ns)
        if mesh_el is None:
            continue

        sources = {}
        for src in mesh_el.findall("c:source", ns):
            fa = src.find("c:float_array", ns)
            if fa is not None:
                sources[src.get("id")] = [float(v) for v in fa.text.split()]

        pos_source_id = None
        vertices_el = mesh_el.find("c:vertices", ns)
        if vertices_el is not None:
            for inp in vertices_el.findall("c:input", ns):
                if inp.get("semantic") == "POSITION":
                    pos_source_id = inp.get("source", "").lstrip("#")

        for tri_el in (list(mesh_el.findall("c:triangles", ns)) +
                       list(mesh_el.findall("c:polylist", ns))):
            vertex_offset = 0
            stride = 1
            for inp in tri_el.findall("c:input", ns):
                off = int(inp.get("offset", 0))
                stride = max(stride, off + 1)
                if inp.get("semantic") == "VERTEX":
                    vertex_offset = off

            pos_data = sources.get(pos_source_id, [])
            verts = [(pos_data[i], pos_data[i+1], pos_data[i+2])
                     for i in range(0, len(pos_data), 3)]

            p_el = tri_el.find("c:p", ns)
            if p_el is None:
                continue
            indices = [int(v) for v in p_el.text.split()]

            vcount_el = tri_el.find("c:vcount", ns)
            if vcount_el is not None:
                vcounts = [int(v) for v in vcount_el.text.split()]
            else:
                vcounts = [3] * int(tri_el.get("count", 0))

            faces = []
            idx = 0
            for vc in vcounts:
                face = []
                for _ in range(vc):
                    face.append(indices[idx + vertex_offset])
                    idx += stride
                faces.append(tuple(face))

            # Material color lookup
            mat_sym = tri_el.get("material", "")
            color = None
            for bm in root.findall(
                    ".//c:library_visual_scenes//c:bind_material"
                    "//c:instance_material", ns):
                if bm.get("symbol") == mat_sym:
                    target = bm.get("target", "").lstrip("#")
                    for mat_el in root.findall(
                            ".//c:library_materials/c:material", ns):
                        if mat_el.get("id") == target:
                            ie = mat_el.find("c:instance_effect", ns)
                            if ie is not None:
                                eff_id = ie.get("url", "").lstrip("#")
                                color = mat_colors.get(eff_id)

            results.append((verts, faces, color))
    return results


def create_mesh_objects(name, dae_path):
    """Create Blender mesh objects from a DAE file."""
    if not os.path.exists(dae_path):
        print(f"  WARNING: mesh not found: {dae_path}")
        return []

    geoms = parse_dae(dae_path)
    objects = []
    for gi, (verts, faces, color) in enumerate(geoms):
        mesh_name = f"{name}_{gi}" if len(geoms) > 1 else name
        mesh = bpy.data.meshes.new(mesh_name)
        mesh.from_pydata(verts, [], faces)
        mesh.update()
        obj = bpy.data.objects.new(mesh_name, mesh)
        bpy.context.collection.objects.link(obj)
        if color:
            mat = bpy.data.materials.new(f"{mesh_name}_mat")
            mat.diffuse_color = color
            mesh.materials.append(mat)
        objects.append(obj)
    return objects


# ---------------------------------------------------------------------------
# Clear scene
# ---------------------------------------------------------------------------
def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for block in bpy.data.meshes:
        if block.users == 0:
            bpy.data.meshes.remove(block)


# ---------------------------------------------------------------------------
# Build robot as an Empty hierarchy + meshes
# ---------------------------------------------------------------------------
def build_robot(mesh_dir):
    """Build Go2 kinematic tree using Empties. Returns dict of empties by name.

    Hierarchy:
        base (Empty, animated with body pose)
         └─ trunk (Empty, fixed at origin)
             ├─ {LEG}_hip_joint (Empty, rotates around X)
             │    └─ {LEG}_thigh_joint (Empty, rotates around Y)
             │         └─ {LEG}_calf_joint (Empty, rotates around Y)
             ...

    Meshes are parented to their corresponding Empty with no offset.
    """
    empties = {}

    def make_empty(name, parent=None, location=(0, 0, 0)):
        e = bpy.data.objects.new(name, None)
        e.empty_display_type = 'ARROWS'
        e.empty_display_size = 0.03
        bpy.context.collection.objects.link(e)
        if parent:
            e.parent = parent
        e.location = mathutils.Vector(location)
        empties[name] = e
        return e

    def parent_meshes(objs, empty, rotation=(0, 0, 0)):
        # DAE meshes are authored with legs along -Y; the DAE scene matrix
        # rotates -90 deg around X to align with URDF convention (legs along -Z).
        # Apply that base rotation, then any per-mesh URDF rotation on top.
        base_rot = mathutils.Euler((PI / 2, 0, 0), 'XYZ').to_matrix()
        extra_rot = mathutils.Euler(rotation, 'XYZ').to_matrix()
        combined = (extra_rot @ base_rot).to_euler('XYZ')
        for obj in objs:
            obj.parent = empty
            obj.location = (0, 0, 0)
            obj.rotation_euler = combined

    # Root empties
    base = make_empty("base")
    trunk = make_empty("trunk", parent=base)

    # Trunk mesh
    trunk_objs = create_mesh_objects(
        "trunk", os.path.join(mesh_dir, "trunk.dae"))
    parent_meshes(trunk_objs, trunk)

    # Legs
    for leg_name, mirror, front_hind in LEGS:
        mirror_dae = (mirror == 1)
        front_hind_dae = (front_hind == 1)

        hip_x = LEG_OFFSET_X * front_hind
        hip_y = LEG_OFFSET_Y * mirror

        hip = make_empty(f"{leg_name}_hip_joint", parent=trunk,
                         location=(hip_x, hip_y, 0))

        # Hip mesh with rotation per URDF
        hip_objs = create_mesh_objects(
            f"{leg_name}_hip", os.path.join(mesh_dir, "hip.dae"))
        rot = [0, 0, 0]
        if not mirror_dae and front_hind_dae:
            rot = [PI, 0, 0]
        elif mirror_dae and not front_hind_dae:
            rot = [0, PI, 0]
        elif not mirror_dae and not front_hind_dae:
            rot = [PI, PI, 0]
        parent_meshes(hip_objs, hip, rotation=tuple(rot))

        # Thigh
        thigh = make_empty(f"{leg_name}_thigh_joint", parent=hip,
                           location=(0, THIGH_OFFSET * mirror, 0))

        thigh_file = "thigh.dae" if mirror_dae else "thigh_mirror.dae"
        thigh_objs = create_mesh_objects(
            f"{leg_name}_thigh", os.path.join(mesh_dir, thigh_file))
        parent_meshes(thigh_objs, thigh)

        # Calf
        calf = make_empty(f"{leg_name}_calf_joint", parent=thigh,
                          location=(0, 0, -THIGH_LENGTH))

        calf_objs = create_mesh_objects(
            f"{leg_name}_calf", os.path.join(mesh_dir, "calf.dae"))
        parent_meshes(calf_objs, calf)

    print(f"Built robot from meshes in {mesh_dir}")
    return empties


# ---------------------------------------------------------------------------
# Import terrain
# ---------------------------------------------------------------------------
def import_terrain(terrain_path):
    """Import an STL terrain mesh."""
    if not os.path.exists(terrain_path):
        print(f"Terrain file not found: {terrain_path}")
        return None
    bpy.ops.wm.stl_import(filepath=terrain_path)
    terrain = bpy.context.selected_objects[0]
    terrain.name = "Terrain"
    mat = bpy.data.materials.new("TerrainMat")
    mat.diffuse_color = (0.4, 0.4, 0.4, 1.0)
    terrain.data.materials.append(mat)
    return terrain


# ---------------------------------------------------------------------------
# Animate
# ---------------------------------------------------------------------------
def animate(empties, data):
    """Keyframe empties from extracted bag data."""
    fps = data["fps"]
    frames = data["frames"]

    bpy.context.scene.render.fps = max(1, int(round(fps)))
    bpy.context.scene.frame_start = 1
    bpy.context.scene.frame_end = len(frames)

    base = empties["base"]
    base.rotation_mode = 'QUATERNION'

    # Collect joint empties
    joint_empties = {}
    for name, emp in empties.items():
        if name.endswith("_joint"):
            emp.rotation_mode = 'XYZ'
            joint_empties[name] = emp

    for i, frame in enumerate(frames):
        fi = i + 1  # Blender frames are 1-indexed

        # Body pose — direct ROS coordinates (both Z-up)
        rx, ry, rz = frame["pos"]
        base.location = (rx, ry, rz)
        base.keyframe_insert(data_path="location", frame=fi)

        qx, qy, qz, qw = frame["quat"]
        base.rotation_quaternion = (qw, qx, qy, qz)
        base.keyframe_insert(data_path="rotation_quaternion", frame=fi)

        # Joint angles
        for joint_name, angle in frame["joints"].items():
            emp = joint_empties.get(joint_name)
            if emp is None:
                continue
            if "hip_joint" in joint_name:
                emp.rotation_euler = (angle, 0, 0)
            else:
                emp.rotation_euler = (0, angle, 0)
            emp.keyframe_insert(data_path="rotation_euler", frame=fi)

    print(f"Animated {len(frames)} frames at {fps} FPS")


# ---------------------------------------------------------------------------
# Camera and lighting
# ---------------------------------------------------------------------------
def setup_scene(empties):
    """Add camera, sun, ground plane, and render settings."""
    base = empties["base"]

    # Sun
    light_data = bpy.data.lights.new("Sun", type='SUN')
    light_data.energy = 3.0
    light_obj = bpy.data.objects.new("Sun", light_data)
    light_obj.location = (5, -5, 10)
    light_obj.rotation_euler = (math.radians(30), math.radians(10),
                                math.radians(45))
    bpy.context.collection.objects.link(light_obj)

    # Camera — parent to an empty that tracks the robot's XY position
    cam_track = bpy.data.objects.new("CameraTracker", None)
    cam_track.empty_display_size = 0.01
    bpy.context.collection.objects.link(cam_track)

    # CameraTracker copies robot XY position
    follow = cam_track.constraints.new(type='COPY_LOCATION')
    follow.target = base
    follow.use_x = True
    follow.use_y = True
    follow.use_z = False

    cam_data = bpy.data.cameras.new("Camera")
    cam_data.lens = 50
    cam_obj = bpy.data.objects.new("Camera", cam_data)
    cam_obj.parent = cam_track
    cam_obj.location = (1.2, -1.2, 0.6)  # offset from tracker
    bpy.context.collection.objects.link(cam_obj)
    bpy.context.scene.camera = cam_obj

    # Camera looks at the robot
    track = cam_obj.constraints.new(type='TRACK_TO')
    track.target = base
    track.track_axis = 'TRACK_NEGATIVE_Z'
    track.up_axis = 'UP_Y'

    # Ground plane
    if "Terrain" not in bpy.data.objects:
        bpy.ops.mesh.primitive_plane_add(size=50, location=(0, 0, 0))
        ground = bpy.context.active_object
        ground.name = "Ground"
        mat = bpy.data.materials.new("GroundMat")
        mat.diffuse_color = (0.3, 0.35, 0.3, 1.0)
        ground.data.materials.append(mat)

    # Render settings
    bpy.context.scene.render.engine = 'BLENDER_EEVEE'
    bpy.context.scene.render.resolution_x = 1920
    bpy.context.scene.render.resolution_y = 1080


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []

    parser = argparse.ArgumentParser(
        description="Animate Go2 robot in Blender from bag-extracted JSON")
    parser.add_argument("--json", required=True,
                        help="Path to JSON from extract_bag.py")
    parser.add_argument("--urdf_meshes", required=True,
                        help="Path to go2_description meshes dir")
    parser.add_argument("--terrain", default=None,
                        help="Optional path to terrain STL file")
    parser.add_argument("--output", default="go2_sim.blend",
                        help="Output .blend file path")
    parser.add_argument("--render", default=None,
                        help="If set, render frames to this directory")
    args = parser.parse_args(argv)

    with open(args.json) as f:
        data = json.load(f)
    print(f"Loaded {len(data['frames'])} frames at {data['fps']} FPS")

    clear_scene()
    empties = build_robot(args.urdf_meshes)

    if args.terrain:
        import_terrain(args.terrain)

    animate(empties, data)
    setup_scene(empties)

    bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(args.output))
    print(f"Saved {args.output}")

    if args.render:
        os.makedirs(args.render, exist_ok=True)
        bpy.context.scene.render.filepath = os.path.join(args.render, "frame_")
        bpy.context.scene.render.image_settings.file_format = 'PNG'
        bpy.ops.render.render(animation=True)
        print(f"Rendered frames to {args.render}")


if __name__ == "__main__":
    main()
