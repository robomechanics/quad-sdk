r"""Isaac Sim <-> ROS 2 bridge for quad-sdk.

Replaces Gazebo in the quad-sdk pipeline. Each physics step the bridge
publishes /clock and <ns>/state/ground_truth (quad_msgs/RobotState), and
applies tau = kp*(pos_set - q) + kd*(vel_set - q_dot) + tau_ff from the
latest <ns>/control/joint_command (quad_msgs/LegCommandArray).

Joint ordering: each robot's URDF is pre-renamed so the 12 revolute
joints are j_{abad,hip,knee}_{0..3}. Isaac alphabetises these into
(abad_0..3, hip_0..3, knee_0..3); the controller uses leg-then-joint
order. CTRL_TO_ISAAC permutes between the two.

Foot ordering: leg 0=FL, 1=BL, 2=FR, 3=BR.

Usage:
    conda activate isaaclab
    bash isaac_plugins/scripts/run_isaac_bridge.sh \
        --robot spirit --scene flat [--terrain rough_25cm] [--cinematic]
"""

from __future__ import annotations

import argparse
import pathlib
import sys
from pathlib import Path


def _promote_bundled_rclpy() -> None:
    """Promote Isaac's bundled rclpy ahead of /opt/ros on sys.path.

    Must run before AppLauncher. Without it the Python 3.12 rclpy from
    /opt/ros/jazzy resolves first and the ABI mismatch crashes import.
    """
    try:
        import isaacsim
        import os
    except ImportError:
        return
    bundle_root = pathlib.Path(isaacsim.__file__).parent / (
        'exts/isaacsim.ros2.bridge/jazzy'
    )
    if not bundle_root.exists():
        return

    bundled_rclpy = bundle_root / 'rclpy'
    if bundled_rclpy.exists():
        path = str(bundled_rclpy)
        if path in sys.path:
            sys.path.remove(path)
        sys.path.insert(0, path)

    bundled_lib = bundle_root / 'lib'
    if bundled_lib.exists():
        existing = os.environ.get('LD_LIBRARY_PATH', '')
        parts = [p for p in existing.split(':') if p and p != str(bundled_lib)]
        os.environ['LD_LIBRARY_PATH'] = ':'.join([str(bundled_lib)] + parts)


_promote_bundled_rclpy()

from isaaclab.app import AppLauncher  # noqa: E402
import numpy as np  # noqa: E402


WORKSPACE_INSTALL = Path('/home/rml/ros2_ws/install')

ROBOT_REGISTRY: dict[str, dict] = {
    'spirit': {
        'urdf': (
            WORKSPACE_INSTALL
            / 'spirit_description/share/spirit_description'
            / 'models/spirit/urdf/spirit_isaac_clean.urdf'
        ),
        'toe_links': ['toe0', 'toe1', 'toe2', 'toe3'],
        'sit_q_per_leg': (0.0, 0.0, 0.0),
        # 'armature': (4.4316e-3, 4.4316e-3, 1.77264e-2),
        'toe_damping': 0.1,
        'spawn_y': 0.0,
        'spawn_z': 0.3,
    },
    'go2': {
        'urdf': (
            WORKSPACE_INSTALL
            / 'go2_description/share/go2_description'
            / 'models/go2/urdf/go2_isaac_clean.urdf'
        ),
        'toe_links': ['toe0', 'toe1', 'toe2', 'toe3'],
        'sit_q_per_leg': (0.0, 1.36, -2.65),
        'spawn_y': 0.25,
        'spawn_z': 0.3,
    },
}

PHYSICS_HZ = 1000.0
PHYSICS_DT = 1.0 / PHYSICS_HZ
RENDER_DT = 1.0 / 60.0

PHYSX_POSITION_ITERS = 64
PHYSX_VELOCITY_ITERS = 16
GROUND_FRICTION = 100.0
FOOT_CONTACT_Z_THRESHOLD = 0.04

CTRL_TO_ISAAC = np.array(
    [(c // 3) + 4 * (c % 3) for c in range(12)], dtype=np.int64
)
ISAAC_TO_CTRL = np.argsort(CTRL_TO_ISAAC)

CTRL_JOINT_ORDER: tuple[str, ...] = tuple(
    f'j_{joint}_{leg}'
    for leg in range(4)
    for joint in ('abad', 'hip', 'knee')
)

CAMERA_PRESETS: dict[str, tuple[float, float, float]] = {
    'chase':  (-5.0, 3.0, 1.4),
    'hero':   (2.5, 0.0, 0.4),
    'side':   (0.0, 4.0, 0.6),
    'behind': (-1.5, 0.0, 0.6),
    'bird':   (0.0, 0.0, 4.0),
    'wide':   (-6.0, 4.0, 2.5),
}

HDRI_SKY_PATH = (
    '/home/rml/anaconda3/envs/isaaclab/lib/python3.11/'
    'site-packages/isaacsim/extscache/'
    'omni.kit.window.usd_paths-1.0.8+69cbf6ad/data/sunflowers.hdr'
)


def parse_args() -> argparse.Namespace:
    """Parse CLI args; AppLauncher consumes its own subset before us."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='robot_1')
    parser.add_argument(
        '--robot', choices=sorted(ROBOT_REGISTRY), default='spirit',
        help='Robot from ROBOT_REGISTRY (selects URDF + spawn config).',
    )
    parser.add_argument(
        '--urdf', default=None,
        help='Override URDF path; defaults to the registry entry.',
    )
    parser.add_argument(
        '--scene', choices=('flat', 'underbrush'), default='flat',
        help='flat=ground only; underbrush=ground + four-cord vines.',
    )
    parser.add_argument(
        '--terrain', default=None,
        help=(
            'Terrain STL: short name (resolved against gazebo_scripts '
            'models/<name>/meshes/<name>.stl) or absolute .stl path. '
            'Suppresses --ground; pose from matching world .sdf.'
        ),
    )
    parser.add_argument(
        '--spawn-z', type=float, default=None,
        help=(
            'Spawn height (m); overrides per-robot default. Raise above '
            'max bump height + ~0.4 m on rough terrains.'
        ),
    )
    parser.add_argument(
        '--cinematic', action='store_true', default=False,
        help='Follow camera + better lighting for video capture.',
    )
    parser.add_argument(
        '--camera-preset', choices=tuple(CAMERA_PRESETS),
        default='chase',
        help='Follow-camera preset (--cinematic only).',
    )
    parser.add_argument(
        '--camera-offset', nargs=3, type=float,
        metavar=('BACK', 'RIGHT', 'UP'), default=None,
        help='Literal camera offset (m); overrides --camera-preset.',
    )
    parser.add_argument(
        '--camera-focal', type=float, default=35.0,
        help='Camera focal length (mm). Default 35.',
    )
    parser.add_argument(
        '--ground', choices=('isaaclab', 'default', 'custom', 'layered'),
        default='isaaclab',
        help=(
            'Ground style: isaaclab=TerrainImporterCfg (training-parity), '
            'default=Isaac built-in grid, custom=dark concrete, '
            'layered=both stacked.'
        ),
    )
    parser.add_argument(
        '--physx-gpu', action='store_true', default=False,
        help='Enable PhysX GPU dynamics (off by default).',
    )
    parser.add_argument(
        '--tau-scale', type=float, default=1.0,
        help='Scale commanded joint torques before applying (default 1.0).',
    )
    parser.add_argument(
        '--record', metavar='PATH', nargs='?', default=None,
        const='__AUTO__',
        help='Auto-record viewport to MP4 (no PATH = ~/Videos/<auto>.mp4).',
    )
    parser.add_argument('--record-delay', type=float, default=5.0)
    parser.add_argument('--record-duration', type=float, default=30.0)
    parser.add_argument('--record-fps', type=int, default=60)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()

    robot_cfg = ROBOT_REGISTRY[args.robot]
    args.urdf = args.urdf or str(robot_cfg['urdf'])
    args.toe_links = list(robot_cfg['toe_links'])
    args.sit_q_per_leg = tuple(robot_cfg['sit_q_per_leg'])
    args.armature = tuple(robot_cfg.get('armature') or (0.0, 0.0, 0.0))
    args.toe_damping = float(robot_cfg.get('toe_damping', 0.0))
    args.spawn_x = float(robot_cfg.get('spawn_x', 0.0))
    args.spawn_y = float(robot_cfg.get('spawn_y', 0.0))
    cli_spawn_z = getattr(args, 'spawn_z', None)
    args.spawn_z = float(
        cli_spawn_z if cli_spawn_z is not None else robot_cfg['spawn_z']
    )

    args.terrain_stl = _resolve_terrain_stl(args.terrain)

    if args.record == '__AUTO__':
        from datetime import datetime
        videos_dir = Path.home() / 'Videos'
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        args.record = str(
            videos_dir / f'{args.robot}_{args.scene}_{stamp}.mp4'
        )
    return args


def _resolve_terrain_stl(terrain_arg: str | None) -> Path | None:
    """Resolve --terrain (short name or path) to an absolute STL Path."""
    if not terrain_arg:
        return None
    candidate = Path(terrain_arg)
    if not (candidate.is_absolute() or candidate.suffix == '.stl'):
        candidate = (
            WORKSPACE_INSTALL
            / 'gazebo_scripts/share/gazebo_scripts/models'
            / terrain_arg / 'meshes' / f'{terrain_arg}.stl'
        )
    if not candidate.is_file():
        raise SystemExit(
            f'--terrain: STL not found at {candidate}. For short '
            'names, ensure gazebo_scripts is built '
            '(colcon build --packages-select gazebo_scripts).'
        )
    return candidate


def _step(msg: str) -> None:
    """Log a startup progress marker to stderr (line-buffered)."""
    print(f'[bridge] {msg}', file=sys.stderr, flush=True)


def _world_to_body_rotmat(quat_wxyz: np.ndarray) -> np.ndarray:
    """Build R_bw (world -> body) from a (w,x,y,z) quaternion."""
    qw, qx, qy, qz = quat_wxyz
    return np.array([
        [1 - 2 * (qy * qy + qz * qz),
         2 * (qx * qy + qz * qw),
         2 * (qx * qz - qy * qw)],
        [2 * (qx * qy - qz * qw),
         1 - 2 * (qx * qx + qz * qz),
         2 * (qy * qz + qx * qw)],
        [2 * (qx * qz + qy * qw),
         2 * (qy * qz - qx * qw),
         1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def _enable_physx_gpu(stage) -> None:
    """Enable PhysX GPU dynamics on /World/physicsScene."""
    from pxr import PhysxSchema
    scene_prim = stage.GetPrimAtPath('/World/physicsScene')
    if not (scene_prim and scene_prim.IsValid()):
        print(
            '[bridge] WARN: --physx-gpu set but /World/physicsScene '
            'not found; staying on CPU',
            file=sys.stderr,
        )
        return
    scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    scene_api.CreateEnableGPUDynamicsAttr().Set(True)
    scene_api.CreateBroadphaseTypeAttr().Set('GPU')
    _step('PhysX GPU dynamics ENABLED on /World/physicsScene')


def _spawn_custom_ground(stage, z_offset: float = 0.0) -> None:
    """Spawn the recolorable dark-concrete custom plane at /World/ground."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    GROUND_SIZE = 100.0
    GROUND_THICKNESS = 0.1
    GROUND_COLOR = Gf.Vec3f(0.09, 0.07, 0.06)
    ground_path = '/World/ground'

    ground_xform = UsdGeom.Xform.Define(stage, ground_path)
    ground_xform.AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, z_offset - GROUND_THICKNESS / 2.0)
    )
    cube = UsdGeom.Cube.Define(stage, f'{ground_path}/geometry')
    cube.GetSizeAttr().Set(1.0)
    UsdGeom.XformCommonAPI(cube).SetScale(
        (GROUND_SIZE, GROUND_SIZE, GROUND_THICKNESS)
    )
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(cube.GetPrim())

    pmat = UsdShade.Material.Define(
        stage, f'{ground_path}/PhysicsMaterial'
    )
    pmat_api = UsdPhysics.MaterialAPI.Apply(pmat.GetPrim())
    pmat_api.CreateStaticFrictionAttr(GROUND_FRICTION)
    pmat_api.CreateDynamicFrictionAttr(GROUND_FRICTION)
    pmat_api.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(
        pmat,
        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose='physics',
    )

    vmat = UsdShade.Material.Define(
        stage, f'{ground_path}/VisualMaterial'
    )
    shader = UsdShade.Shader.Define(
        stage, f'{ground_path}/VisualMaterial/Shader'
    )
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput(
        'diffuseColor', Sdf.ValueTypeNames.Color3f
    ).Set(GROUND_COLOR)
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.9)
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.0)
    vmat.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), 'surface'
    )
    UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(
        vmat,
        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
    )


def _spawn_isaaclab_ground() -> None:
    """Spawn the IsaacLab TerrainImporterCfg plane (matches training)."""
    from isaaclab.terrains import TerrainImporter, TerrainImporterCfg
    from isaaclab.sim import RigidBodyMaterialCfg
    TerrainImporter(TerrainImporterCfg(
        prim_path='/World/ground',
        terrain_type='plane',
        collision_group=-1,
        num_envs=1,
        env_spacing=1.0,
        physics_material=RigidBodyMaterialCfg(
            friction_combine_mode='multiply',
            restitution_combine_mode='multiply',
            static_friction=GROUND_FRICTION,
            dynamic_friction=GROUND_FRICTION,
            restitution=0.0,
        ),
    ))


def _setup_ground(world, stage, ground_style: str) -> None:
    """Spawn the configured ground plane."""
    if ground_style == 'isaaclab':
        _step('ground: IsaacLab TerrainImporterCfg (matches training)')
        _spawn_isaaclab_ground()
    elif ground_style == 'default':
        _step('ground: Isaac default plane (checkered grid)')
        world.scene.add_default_ground_plane(
            static_friction=GROUND_FRICTION,
            dynamic_friction=GROUND_FRICTION,
        )
    elif ground_style == 'custom':
        _step('ground: custom dark concrete plane')
        _spawn_custom_ground(stage)
    elif ground_style == 'layered':
        _step('ground: layered (default grid on top, custom dark below)')
        world.scene.add_default_ground_plane(
            static_friction=GROUND_FRICTION,
            dynamic_friction=GROUND_FRICTION,
        )
        _spawn_custom_ground(stage, z_offset=-0.02)


def _read_terrain_pose_from_sdf(
    stl_path: Path,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Read <include><pose> for the named model from the matching world .sdf.

    Returns (xyz, rpy); falls back to ((0,0,0), (0,0,0)).
    """
    import xml.etree.ElementTree as ET

    translate = (0.0, 0.0, 0.0)
    rotate = (0.0, 0.0, 0.0)
    share_dir = stl_path.parent.parent.parent.parent
    world_sdf = share_dir / 'worlds' / f'{stl_path.stem}.sdf'
    if not world_sdf.is_file():
        _step(f'no world sdf at {world_sdf}; terrain placed at origin')
        return translate, rotate
    try:
        root = ET.parse(str(world_sdf)).getroot()
    except ET.ParseError as exc:
        print(
            f'[bridge] WARN: failed to parse {world_sdf}: {exc}; '
            'using (0,0,0)',
            file=sys.stderr,
        )
        return translate, rotate

    model_name = stl_path.stem
    for include in root.iter('include'):
        uri_el = include.find('uri')
        if uri_el is None:
            continue
        uri = (uri_el.text or '').strip()
        if not (uri.endswith(f'/{model_name}')
                or uri.endswith(f'://{model_name}')):
            continue
        pose_el = include.find('pose')
        if pose_el is not None and pose_el.text:
            parts = pose_el.text.split()
            if len(parts) >= 6:
                translate = (float(parts[0]), float(parts[1]),
                             float(parts[2]))
                rotate = (float(parts[3]), float(parts[4]),
                          float(parts[5]))
        break
    _step(f'terrain pose from {world_sdf.name}: '
          f'xyz={translate}, rpy={rotate}')
    return translate, rotate


def _spawn_terrain_mesh(stage, stl_path: Path) -> None:
    """Load a triangle-mesh terrain STL as a static collider."""
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade
    import trimesh

    _step(f'terrain: loading STL {stl_path}')
    translate, rotate = _read_terrain_pose_from_sdf(stl_path)

    mesh = trimesh.load(str(stl_path), process=False)
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
    points = mesh.vertices.astype('float32')
    faces = mesh.faces.astype('int32')

    terrain_path = '/World/terrain'
    xform = UsdGeom.Xform.Define(stage, terrain_path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if any(abs(r) > 1e-9 for r in rotate):
        xform.AddRotateXYZOp().Set(Gf.Vec3f(
            np.degrees(rotate[0]),
            np.degrees(rotate[1]),
            np.degrees(rotate[2]),
        ))

    terrain_mesh = UsdGeom.Mesh.Define(stage, f'{terrain_path}/geometry')
    terrain_mesh.CreatePointsAttr(points.tolist())
    terrain_mesh.CreateFaceVertexCountsAttr([3] * faces.shape[0])
    terrain_mesh.CreateFaceVertexIndicesAttr(faces.flatten().tolist())
    UsdPhysics.CollisionAPI.Apply(terrain_mesh.GetPrim())
    coll = UsdPhysics.MeshCollisionAPI.Apply(terrain_mesh.GetPrim())
    coll.CreateApproximationAttr().Set('none')

    pmat = UsdShade.Material.Define(
        stage, f'{terrain_path}/PhysicsMaterial'
    )
    pmat_api = UsdPhysics.MaterialAPI.Apply(pmat.GetPrim())
    pmat_api.CreateStaticFrictionAttr(GROUND_FRICTION)
    pmat_api.CreateDynamicFrictionAttr(GROUND_FRICTION)
    pmat_api.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI(terrain_mesh.GetPrim()).Bind(
        pmat,
        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose='physics',
    )
    _step(f'terrain: {len(points)} verts, {len(faces)} tris '
          f'at {terrain_path} (collider applied)')


def _spawn_baseline_dome(stage) -> None:
    """Minimal dome light so the scene is visible without --cinematic."""
    from pxr import UsdLux
    dome = UsdLux.DomeLight.Define(stage, '/World/baseline_dome')
    dome.CreateIntensityAttr(300.0)


def _setup_cinematic_lighting(stage) -> None:
    """Three-point + HDRI sky + bumped RTX settings."""
    from pxr import Gf, UsdGeom, UsdLux

    for default_light in (
        '/World/defaultDistantLight',
        '/World/defaultDomeLight',
        '/World/baseline_dome',
    ):
        if stage.GetPrimAtPath(default_light).IsValid():
            stage.RemovePrim(default_light)

    sun = UsdLux.DistantLight.Define(stage, '/World/key_sun')
    sun.CreateIntensityAttr(4500.0)
    sun.CreateAngleAttr(0.6)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.92, 0.78))
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(-55.0, 0.0, 35.0)
    )
    UsdLux.ShadowAPI.Apply(
        sun.GetPrim()
    ).CreateShadowEnableAttr().Set(True)

    sky = UsdLux.DomeLight.Define(stage, '/World/fill_sky')
    sky.CreateIntensityAttr(550.0)
    sky.CreateTextureFileAttr().Set(HDRI_SKY_PATH)
    sky.CreateTextureFormatAttr().Set('latlong')

    rim = UsdLux.DistantLight.Define(stage, '/World/rim')
    rim.CreateIntensityAttr(3000.0)
    rim.CreateAngleAttr(1.0)
    rim.CreateColorAttr(Gf.Vec3f(0.60, 0.78, 1.05))
    UsdGeom.Xformable(rim.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(-30.0, 0.0, -150.0)
    )

    try:
        import carb
        cs = carb.settings.get_settings()
        cs.set('/rtx/post/aa/op', 3)
        cs.set('/rtx/post/aa/enabled', True)
        cs.set('/rtx/shadows/enabled', True)
        cs.set('/rtx/shadows/sampleCount', 4)
        cs.set('/rtx/indirectDiffuse/enabled', True)
        cs.set('/rtx/ambientOcclusion/enabled', True)
        cs.set('/rtx/reflections/enabled', True)
        cs.set('/rtx/post/tonemap/op', 5)
    except Exception as exc:  # noqa: BLE001
        _step(f'render-quality settings not all available: {exc}')


def _import_robot_urdf(urdf_path: str) -> str:
    """Run Isaac's URDF importer; return the imported prim path."""
    import omni.kit.commands
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension('isaacsim.asset.importer.urdf')
    from isaacsim.asset.importer.urdf import _urdf

    cfg = _urdf.ImportConfig()
    cfg.merge_fixed_joints = False
    cfg.fix_base = False
    cfg.import_inertia_tensor = True
    cfg.distance_scale = 1.0
    cfg.density = 0.0
    cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_NONE
    cfg.default_drive_strength = 0.0
    cfg.default_position_drive_damping = 0.0
    cfg.convex_decomp = False
    cfg.self_collision = False
    cfg.create_physics_scene = False
    cfg.make_default_prim = False

    parse_ok, robot_model = omni.kit.commands.execute(
        'URDFParseFile', urdf_path=urdf_path, import_config=cfg,
    )
    if not parse_ok:
        raise SystemExit('URDFParseFile returned False')

    import_ok, prim_path = omni.kit.commands.execute(
        'URDFImportRobot',
        urdf_path=urdf_path,
        urdf_robot=robot_model,
        import_config=cfg,
    )
    if not import_ok:
        raise SystemExit('URDFImportRobot returned False')
    return str(prim_path)


def _apply_cinematic_robot_material(stage, prim_path: str) -> None:
    """Override URDF importer materials with polished-black settings."""
    from pxr import Gf, UsdShade

    diffuse = Gf.Vec3f(0.03, 0.03, 0.04)
    inputs = {
        'diffuseColor': diffuse,
        'roughness': 0.35,
        'metallic': 0.20,
        'diffuse_color_constant': diffuse,
        'reflection_roughness_constant': 0.35,
        'metallic_constant': 0.20,
        'diffuse_texture': '',
        'albedo_add': 0.0,
    }
    PRESERVE_TOKENS = ('scots_rose',)

    root = stage.GetPrimAtPath(prim_path)
    if not (root and root.IsValid()):
        return
    shader_count = 0
    modified_inputs = 0
    skipped = 0
    stack = [root]
    while stack:
        prim = stack.pop()
        if prim.IsA(UsdShade.Shader):
            if any(tok in str(prim.GetPath()) for tok in PRESERVE_TOKENS):
                skipped += 1
            else:
                shader = UsdShade.Shader(prim)
                shader_count += 1
                for name, val in inputs.items():
                    inp = shader.GetInput(name)
                    if inp is not None and inp:
                        try:
                            inp.Set(val)
                            modified_inputs += 1
                        except Exception:  # noqa: BLE001
                            pass
        stack.extend(prim.GetAllChildren())
    _step(f'robot: {shader_count} shaders blacked, '
          f'{modified_inputs} inputs set, {skipped} preserved')


def _find_articulation_root(stage) -> str:
    """Return the path of the first ArticulationRootAPI prim on the stage."""
    from pxr import UsdPhysics
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    raise SystemExit('No ArticulationRootAPI prim found after URDF import')


def _set_articulation_solver_iters(stage, art_path: str) -> None:
    """Raise PhysX solver iters to close the gap with Gazebo/ODE."""
    from pxr import PhysxSchema
    api = PhysxSchema.PhysxArticulationAPI.Apply(
        stage.GetPrimAtPath(art_path)
    )
    api.CreateSolverPositionIterationCountAttr().Set(PHYSX_POSITION_ITERS)
    api.CreateSolverVelocityIterationCountAttr().Set(PHYSX_VELOCITY_ITERS)
    _step(f'articulation solver iters: pos={PHYSX_POSITION_ITERS}, '
          f'vel={PHYSX_VELOCITY_ITERS}')


_JOINT_NAME_RE_SRC = r'^j_(abad|hip|knee)_([0-3])$'
_JOINT_OFFSET = {'abad': 0, 'hip': 1, 'knee': 2}


def _apply_toe_damping(stage, body_root: str, toe_links, damping: float) -> None:
    """Set PhysxRigidBodyAPI linear/angular damping on toe links."""
    if damping <= 0.0:
        return
    from pxr import PhysxSchema
    applied = 0
    for name in toe_links:
        prim = stage.GetPrimAtPath(f'{body_root}/{name}')
        if not (prim and prim.IsValid()):
            continue
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        api.CreateLinearDampingAttr().Set(damping)
        api.CreateAngularDampingAttr().Set(damping)
        applied += 1
    _step(f'toe damping {damping} applied on {applied} toe rigid bodies')


def _apply_joint_armature(stage, armature) -> None:
    """Set PhysxJointAPI.armature on j_{abad,hip,knee}_{0..3} joints."""
    import re
    from pxr import PhysxSchema, UsdPhysics
    if not any(armature):
        return
    joint_re = re.compile(_JOINT_NAME_RE_SRC)
    applied = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        match = joint_re.match(prim.GetName())
        if match is None:
            continue
        value = float(armature[_JOINT_OFFSET[match.group(1)]])
        PhysxSchema.PhysxJointAPI.Apply(prim).CreateArmatureAttr().Set(value)
        applied += 1
    _step(f'armature applied on {applied} joints: '
          f'abad={armature[0]:.4e}, hip={armature[1]:.4e}, '
          f'knee={armature[2]:.4e}')


def _author_initial_state(stage, art_path: str, args) -> None:
    """Write base pose and joint sit positions into USD pre-reset.

    Post-reset writes on a drive-less floating articulation lose to
    gravity before they land, so we author state directly.
    """
    import re
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    spawn_prim = stage.GetPrimAtPath(art_path)
    spawn_xform = UsdGeom.Xformable(spawn_prim)
    translate_op = None
    for op in spawn_xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if translate_op is None:
        translate_op = spawn_xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(
        float(args.spawn_x), float(args.spawn_y), float(args.spawn_z)
    ))
    _step(f'articulation root translated to '
          f'(x={args.spawn_x}, y={args.spawn_y}, z={args.spawn_z})')

    joint_re = re.compile(_JOINT_NAME_RE_SRC)
    joints_set = 0
    unmatched = []
    for joint_prim in stage.Traverse():
        if not joint_prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        match = joint_re.match(joint_prim.GetName())
        if match is None:
            unmatched.append(joint_prim.GetName())
            continue
        sit_val = float(
            args.sit_q_per_leg[_JOINT_OFFSET[match.group(1)]]
        )
        state_api = PhysxSchema.JointStateAPI.Apply(joint_prim, 'angular')
        state_api.CreatePositionAttr().Set(sit_val)
        state_api.CreateVelocityAttr().Set(0.0)
        joints_set += 1
    _step(f'joint initial state set on {joints_set} revolute joints '
          f'(unmatched: {unmatched})')


def _spawn_underbrush_vines(stage) -> None:
    """Spawn the four-cord underbrush vine scenario."""
    sys.path.insert(
        0,
        '/home/rml/ros2_ws/src/quad-sdk/quad_simulator/'
        'isaac_plugins/scripts',
    )
    from spawn_vines import spawn_underbrush_scenario
    vine_paths = spawn_underbrush_scenario(stage)
    _step(f'spawned {len(vine_paths)} vines')


def _validate_dof_order(articulation) -> None:
    """Warn loudly if Isaac's DOF order doesn't match CTRL_TO_ISAAC."""
    try:
        dof_names = list(articulation.dof_names)
    except AttributeError:
        return
    mismatches = []
    for c, expected in enumerate(CTRL_JOINT_ORDER):
        isaac_idx = int(CTRL_TO_ISAAC[c])
        if isaac_idx >= len(dof_names):
            mismatches.append(
                f'ctrl[{c}] {expected} -> isaac[{isaac_idx}] OOB'
            )
        elif dof_names[isaac_idx] != expected:
            mismatches.append(
                f'ctrl[{c}] {expected} -> isaac[{isaac_idx}] '
                f'{dof_names[isaac_idx]}'
            )
    if mismatches:
        print(
            '[bridge] WARN: CTRL_TO_ISAAC permutation mismatch -- '
            'commands will route to wrong joints:',
            file=sys.stderr,
        )
        for line in mismatches:
            print(f'    {line}', file=sys.stderr)


def _apply_sit_pose(articulation, sit_q_per_leg) -> None:
    """Fold legs into the configured sit pose."""
    sit = np.asarray(sit_q_per_leg, dtype=np.float32)
    stance_isaac = np.zeros(12, dtype=np.float32)
    for leg in range(4):
        for joint_off in range(3):
            isaac_idx = int(CTRL_TO_ISAAC[3 * leg + joint_off])
            stance_isaac[isaac_idx] = sit[joint_off]
    articulation.set_joint_positions(stance_isaac)
    articulation.set_joint_velocities(np.zeros(12, dtype=np.float32))


def _setup_follow_camera(stage, args):
    """Create the cinematic follow camera; return (camera, offset).

    Returns (None, None) when --cinematic is off.
    """
    if not args.cinematic:
        return None, None
    from pxr import Gf, UsdGeom

    offset = (
        np.array(args.camera_offset, dtype=np.float64)
        if args.camera_offset is not None
        else np.array(CAMERA_PRESETS[args.camera_preset])
    )
    camera = UsdGeom.Camera.Define(stage, '/World/follow_camera')
    camera.CreateFocalLengthAttr(args.camera_focal)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
    try:
        from omni.kit.viewport.utility import get_active_viewport
        vp = get_active_viewport()
        if vp is not None:
            vp.set_active_camera('/World/follow_camera')
            _step('viewport switched to /World/follow_camera')
    except Exception as exc:  # noqa: BLE001
        _step(f'could not set viewport camera: {exc}')
    _step(f'camera offset={offset.tolist()} focal={args.camera_focal}')
    return camera, offset


class _Recorder:
    """Auto-start/stop the Movie Capture extension on a wall-clock schedule."""

    def __init__(self, output_path: str, delay_s: float,
                 duration_s: float, fps: int) -> None:
        import time
        self._path = output_path
        self._delay = delay_s
        self._duration = duration_s
        self._fps = fps
        self._wall_start = time.monotonic()
        self._started = False
        self._stopped = False

    def tick(self) -> None:
        if self._stopped:
            return
        import time
        elapsed = time.monotonic() - self._wall_start
        if not self._started and elapsed >= self._delay:
            if self._start_capture():
                _step(f'recording started: {self._path}')
                self._started = True
            else:
                _step('auto-record unavailable in this Isaac Sim '
                      'version; use Window > Capture manually')
                self._stopped = True
        elif self._started and elapsed >= self._delay + self._duration:
            self._stop_capture()
            _step(f'recording stopped after {self._duration:.1f}s')
            self._stopped = True

    def finalize(self) -> None:
        if self._started and not self._stopped:
            self._stop_capture()
            _step('recording stopped (shutdown)')

    def _start_capture(self) -> bool:
        out = Path(self._path)
        out.parent.mkdir(parents=True, exist_ok=True)
        try:
            from omni.kit.capture.viewport import (
                CaptureExtension, CaptureOptions,
            )
            opts = CaptureOptions()
            for name, val in (
                ('file_name', out.stem),
                ('output_folder', str(out.parent)),
                ('file_type', '.mp4'),
                ('movie_type', '.mp4'),
                ('fps', self._fps),
                ('movie_save_individual_frames', False),
                ('hdr_output', False),
                ('range_type', 'time'),
                ('range_min', 0.0),
                ('range_max', 3600.0),
            ):
                try:
                    setattr(opts, name, val)
                except (AttributeError, TypeError):
                    pass
            ext = CaptureExtension.get_instance()
            for method in ('start_capture', 'start'):
                fn = getattr(ext, method, None)
                if fn is None:
                    continue
                try:
                    fn(opts)
                except TypeError:
                    fn()
                return True
        except Exception as exc:  # noqa: BLE001
            _step(f'CaptureExtension API failed: '
                  f'{type(exc).__name__}: {exc}')
        return False

    def _stop_capture(self) -> None:
        try:
            from omni.kit.capture.viewport import CaptureExtension
            ext = CaptureExtension.get_instance()
            for method in ('stop_capture', 'stop', 'cancel'):
                fn = getattr(ext, method, None)
                if fn is not None:
                    fn()
                    return
        except Exception as exc:  # noqa: BLE001
            _step(f'stop_capture API failed: {exc}')


def main() -> None:
    """Launch Isaac, wire up ROS 2, and run the bridge loop."""
    args = parse_args()

    if not Path(args.urdf).exists():
        raise SystemExit(
            f'URDF not found at {args.urdf}; run the prep pipeline '
            '(xacro -> resolve_package_urls -> rename_joints -> '
            'strip_gazebo_tags) first'
        )

    _step('starting AppLauncher')
    app_launcher = AppLauncher(args)
    sim_app = app_launcher.app
    _step('AppLauncher up')

    import os
    import isaacsim.core.utils.stage as stage_utils
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Gf, UsdGeom
    _step('Isaac imports done')

    enable_extension('isaacsim.ros2.bridge')
    _step('ros2 bridge extension enabled; importing rclpy')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from quad_msgs.msg import (
        FootState, LegCommandArray, RobotState,
    )
    _step('rclpy imported')

    _step('creating World on a fresh stage')
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=PHYSICS_DT,
        rendering_dt=RENDER_DT,
    )
    stage = stage_utils.get_current_stage()
    _step('World created')

    if args.physx_gpu:
        _enable_physx_gpu(stage)

    if args.terrain_stl is not None:
        _spawn_terrain_mesh(stage, args.terrain_stl)
    else:
        _setup_ground(world, stage, args.ground)

    _spawn_baseline_dome(stage)
    if args.cinematic:
        _step('cinematic: three-point lighting + render settings')
        _setup_cinematic_lighting(stage)

    _step(f'importing URDF directly: {args.urdf}')
    prim_path = _import_robot_urdf(args.urdf)
    _step(f'URDF imported as prim {prim_path}')

    if args.cinematic:
        _apply_cinematic_robot_material(stage, prim_path)

    art_path = _find_articulation_root(stage)
    _step(f'articulation found at {art_path}')

    _set_articulation_solver_iters(stage, art_path)
    _apply_joint_armature(stage, args.armature)
    _author_initial_state(stage, art_path, args)

    articulation = SingleArticulation(prim_path=art_path, name=args.robot)
    world.scene.add(articulation)

    body_root = art_path.rsplit('/', 1)[0]
    toe_prims = [
        SingleRigidPrim(prim_path=f'{body_root}/{name}', name=name)
        for name in args.toe_links
    ]
    for toe in toe_prims:
        world.scene.add(toe)
    _apply_toe_damping(stage, body_root, args.toe_links, args.toe_damping)
    _step('articulation + toes added to scene; resetting world')

    if args.scene == 'underbrush':
        _step('spawning underbrush vine scenario')
        _spawn_underbrush_vines(stage)

    world.reset()
    _step('world.reset() done')

    _validate_dof_order(articulation)
    _apply_sit_pose(articulation, args.sit_q_per_leg)
    _step(f'sit pose applied: q_per_leg={args.sit_q_per_leg}')

    follow_camera, follow_offset = _setup_follow_camera(stage, args)
    if follow_camera is not None:
        _step('cinematic follow camera ready')

    n_dof = articulation.num_dof
    articulation.get_articulation_controller().set_gains(
        kps=np.zeros(n_dof), kds=np.zeros(n_dof),
    )

    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    _step(f'RMW_IMPLEMENTATION={os.environ["RMW_IMPLEMENTATION"]}')

    _step('rclpy.init()')
    rclpy.init()
    _step(f'creating Node in namespace {args.namespace}')
    node = Node('isaac_bridge', namespace=args.namespace)
    _step('Node created')

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    state_pub = node.create_publisher(RobotState, 'state/ground_truth', qos)
    clock_pub = node.create_publisher(Clock, '/clock', 10)

    last_cmd: dict[str, LegCommandArray | None] = {'msg': None}

    def cmd_cb(msg: LegCommandArray) -> None:
        last_cmd['msg'] = msg

    node.create_subscription(
        LegCommandArray, 'control/joint_command', cmd_cb, qos,
    )

    sim_time = {'t': 0.0}
    body_filtered: dict[str, np.ndarray | None] = {'p': None}

    def _update_follow_camera() -> None:
        if follow_camera is None or follow_offset is None:
            return
        body_pos, _ = articulation.get_world_pose()
        body_pos = np.asarray(body_pos, dtype=np.float64)
        if body_filtered['p'] is None:
            body_filtered['p'] = body_pos.copy()
        else:
            body_filtered['p'] = (
                0.92 * body_filtered['p'] + 0.08 * body_pos
            )
        eye = body_filtered['p'] + follow_offset
        view = Gf.Matrix4d()
        view.SetLookAt(
            Gf.Vec3d(*eye),
            Gf.Vec3d(*body_filtered['p']),
            Gf.Vec3d(0.0, 0.0, 1.0),
        )
        cam_xform = UsdGeom.Xformable(follow_camera.GetPrim())
        cam_xform.ClearXformOpOrder()
        cam_xform.AddTransformOp().Set(view.GetInverse())

    def physics_step(dt: float) -> None:
        sim_time['t'] += dt
        _update_follow_camera()

        q_iso = np.asarray(articulation.get_joint_positions())
        qd_iso = np.asarray(articulation.get_joint_velocities())
        if hasattr(articulation, 'get_measured_joint_efforts'):
            eff_iso = np.asarray(articulation.get_measured_joint_efforts())
        else:
            eff_iso = np.asarray(articulation.get_applied_joint_efforts())

        body_pos, body_quat_wxyz = articulation.get_world_pose()
        body_pos = np.asarray(body_pos)
        body_quat_wxyz = np.asarray(body_quat_wxyz)
        body_lin = np.asarray(articulation.get_linear_velocity())
        body_ang = (
            _world_to_body_rotmat(body_quat_wxyz)
            @ np.asarray(articulation.get_angular_velocity())
        )

        foot_world = []
        foot_lin = []
        for toe in toe_prims:
            p, _ = toe.get_world_pose()
            foot_world.append(np.asarray(p))
            foot_lin.append(np.asarray(toe.get_linear_velocity()))

        secs = int(sim_time['t'])
        nsec = int((sim_time['t'] - secs) * 1e9)
        clock_msg = Clock()
        clock_msg.clock.sec = secs
        clock_msg.clock.nanosec = nsec
        clock_pub.publish(clock_msg)

        msg = RobotState()
        msg.header.stamp.sec = secs
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id = 'map'

        msg.body.pose.position.x = float(body_pos[0])
        msg.body.pose.position.y = float(body_pos[1])
        msg.body.pose.position.z = float(body_pos[2])
        msg.body.pose.orientation.w = float(body_quat_wxyz[0])
        msg.body.pose.orientation.x = float(body_quat_wxyz[1])
        msg.body.pose.orientation.y = float(body_quat_wxyz[2])
        msg.body.pose.orientation.z = float(body_quat_wxyz[3])
        msg.body.twist.linear.x = float(body_lin[0])
        msg.body.twist.linear.y = float(body_lin[1])
        msg.body.twist.linear.z = float(body_lin[2])
        msg.body.twist.angular.x = float(body_ang[0])
        msg.body.twist.angular.y = float(body_ang[1])
        msg.body.twist.angular.z = float(body_ang[2])

        msg.joints.header = msg.header
        msg.joints.name = list(CTRL_JOINT_ORDER)
        msg.joints.position = q_iso[CTRL_TO_ISAAC].tolist()
        msg.joints.velocity = qd_iso[CTRL_TO_ISAAC].tolist()
        msg.joints.effort = eff_iso[CTRL_TO_ISAAC].tolist()

        msg.feet.header = msg.header
        msg.feet.feet = []
        for i in range(4):
            fs = FootState()
            fs.header = msg.header
            fs.position.x = float(foot_world[i][0])
            fs.position.y = float(foot_world[i][1])
            fs.position.z = float(foot_world[i][2])
            fs.velocity.x = float(foot_lin[i][0])
            fs.velocity.y = float(foot_lin[i][1])
            fs.velocity.z = float(foot_lin[i][2])
            fs.contact = bool(foot_world[i][2] < FOOT_CONTACT_Z_THRESHOLD)
            msg.feet.feet.append(fs)
        state_pub.publish(msg)

        tau_iso = np.zeros(n_dof, dtype=np.float32)
        cmd = last_cmd['msg']
        if cmd is not None and len(cmd.leg_commands) >= 4:
            for c in range(12):
                leg = c // 3
                joint_off = c % 3
                if joint_off >= len(cmd.leg_commands[leg].motor_commands):
                    continue
                mc = cmd.leg_commands[leg].motor_commands[joint_off]
                iso_idx = int(CTRL_TO_ISAAC[c])
                tau_iso[iso_idx] = (
                    mc.kp * (mc.pos_setpoint - q_iso[iso_idx])
                    + mc.kd * (mc.vel_setpoint - qd_iso[iso_idx])
                    + mc.torque_ff
                )
        if args.tau_scale != 1.0:
            tau_iso *= args.tau_scale
        articulation.set_joint_efforts(tau_iso)

    world.add_physics_callback('quad_bridge_step', physics_step)
    _step('physics callback registered; entering main loop')

    recorder: _Recorder | None = None
    if args.record:
        recorder = _Recorder(
            args.record, args.record_delay,
            args.record_duration, args.record_fps,
        )
        _step(
            f'will start recording to {args.record} after '
            f'{args.record_delay:.1f}s, run for '
            f'{args.record_duration:.1f}s @ {args.record_fps} fps'
        )

    node.get_logger().info(
        f'Isaac bridge ready. Articulation: {art_path}, namespace: '
        f'{args.namespace}, physics: {PHYSICS_HZ:.0f} Hz'
    )
    try:
        while sim_app.is_running():
            rclpy.spin_once(node, timeout_sec=0.0)
            world.step(render=True)
            if recorder is not None:
                recorder.tick()
    finally:
        if recorder is not None:
            recorder.finalize()

    rclpy.shutdown()
    sim_app.close()


if __name__ == '__main__':
    main()
