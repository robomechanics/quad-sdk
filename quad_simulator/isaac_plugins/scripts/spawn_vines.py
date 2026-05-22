"""Spawn compliant-cord vines in an Isaac Sim stage.

Self-contained port of the underbrush training pipeline's vine spawner.
Chains of spherical-joint cylinders pinned to kinematic anchors at each
end (a FixedJoint between two static bodies would fault PhysX).
"""

from __future__ import annotations

import math

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

# Values mirror the debug-tuned RobotVinesEnvCfg in the underbrush
# training pipeline, adjusted to be stiffer here (RL-soft vines sag
# into the ground at the demo spawn z).
LINK_LENGTH = 0.18
LINK_RADIUS = 0.020
LINK_MASS = 0.06
NUM_LINKS = 6
CONE_ANGLE_DEG = 35.0

LOOP_PRISMATIC_LIMIT = 0.15
DRIVE_STIFFNESS = 100.0
DRIVE_DAMPING = 8.0
DRIVE_MAX_FORCE = 100.0
LINK_LINEAR_DAMPING = 5.0
LINK_ANGULAR_DAMPING_RB = 5.0

JOINT_ARMATURE = 5.0e-4
JOINT_ANGULAR_DAMPING = 10.0

VINE_COLOR = (0.05, 0.30, 0.05)


def _make_anchor(
    stage: Usd.Stage,
    anchor_path: str,
    anchor_xyz: tuple[float, float, float],
    yaw_deg: float,
) -> str:
    """Create a kinematic rigid-body anchor at ``anchor_xyz`` with yaw."""
    anchor_xform = UsdGeom.Xform.Define(stage, anchor_path)
    anchor_xform.AddTranslateOp().Set(Gf.Vec3d(*anchor_xyz))
    anchor_xform.AddRotateZOp().Set(yaw_deg)
    rb = UsdPhysics.RigidBodyAPI.Apply(anchor_xform.GetPrim())
    rb.CreateKinematicEnabledAttr().Set(True)
    mass = UsdPhysics.MassAPI.Apply(anchor_xform.GetPrim())
    mass.GetMassAttr().Set(0.001)
    return anchor_path


def _spawn_single_vine(
    stage: Usd.Stage,
    vine_path: str,
    center_xyz: tuple[float, float, float],
    yaw_rad: float,
    num_links: int = NUM_LINKS,
    link_length: float = LINK_LENGTH,
    link_radius: float = LINK_RADIUS,
    link_mass: float = LINK_MASS,
    cone_angle_deg: float = CONE_ANGLE_DEG,
    prismatic_limit: float = LOOP_PRISMATIC_LIMIT,
    drive_stiffness: float = DRIVE_STIFFNESS,
    drive_damping: float = DRIVE_DAMPING,
    drive_max_force: float = DRIVE_MAX_FORCE,
    joint_armature: float = JOINT_ARMATURE,
    joint_angular_damping: float = JOINT_ANGULAR_DAMPING,
    color: tuple[float, float, float] = VINE_COLOR,
) -> None:
    """Spawn a single horizontal taut vine at ``center_xyz`` with yaw."""
    UsdGeom.Xform.Define(stage, vine_path)
    color_v = Gf.Vec3f(*color)
    cx, cy, cz = center_xyz

    total_length = num_links * link_length
    half_len = total_length / 2.0
    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)

    left_anchor_xyz = (cx - half_len * cos_y, cy - half_len * sin_y, cz)
    right_anchor_xyz = (cx + half_len * cos_y, cy + half_len * sin_y, cz)

    yaw_deg = math.degrees(yaw_rad)
    parent_path = vine_path.rsplit('/', 1)[0]
    base = vine_path.rsplit('/', 1)[1]
    left_anchor = _make_anchor(
        stage, f'{parent_path}/{base}_anchor_left', left_anchor_xyz, yaw_deg
    )
    right_anchor = _make_anchor(
        stage, f'{parent_path}/{base}_anchor_right', right_anchor_xyz, yaw_deg
    )

    # Zero restitution -- otherwise sagging vines bounce on ground touch.
    vine_pmat_path = f'{vine_path}/PhysicsMaterial'
    vine_pmat = UsdShade.Material.Define(stage, vine_pmat_path)
    vmat_api = UsdPhysics.MaterialAPI.Apply(vine_pmat.GetPrim())
    vmat_api.CreateStaticFrictionAttr(0.5)
    vmat_api.CreateDynamicFrictionAttr(0.5)
    vmat_api.CreateRestitutionAttr(0.0)

    # Matte PreviewSurface so cords don't render as flat un-shaded under PBR.
    vine_vmat_path = f'{vine_path}/VisualMaterial'
    vine_vmat = UsdShade.Material.Define(stage, vine_vmat_path)
    vine_shader = UsdShade.Shader.Define(
        stage, f'{vine_vmat_path}/Shader'
    )
    vine_shader.CreateIdAttr('UsdPreviewSurface')
    vine_shader.CreateInput(
        'diffuseColor', Sdf.ValueTypeNames.Color3f
    ).Set(Gf.Vec3f(0.12, 0.20, 0.10))
    vine_shader.CreateInput(
        'roughness', Sdf.ValueTypeNames.Float
    ).Set(0.95)
    vine_shader.CreateInput(
        'metallic', Sdf.ValueTypeNames.Float
    ).Set(0.0)
    vine_vmat.CreateSurfaceOutput().ConnectToSource(
        vine_shader.ConnectableAPI(), 'surface'
    )

    prev_link_path: str | None = None

    for i in range(num_links):
        link_path = f'{vine_path}/link_{i}'
        x_local = -half_len + (i + 0.5) * link_length
        link_world = (cx + x_local * cos_y, cy + x_local * sin_y, cz)

        link_xform = UsdGeom.Xform.Define(stage, link_path)
        link_xform.AddTranslateOp().Set(Gf.Vec3d(*link_world))
        link_xform.AddRotateZOp().Set(yaw_deg)

        cyl = UsdGeom.Cylinder.Define(stage, f'{link_path}/geometry')
        cyl.GetRadiusAttr().Set(link_radius)
        cyl.GetHeightAttr().Set(link_length)
        cyl.GetAxisAttr().Set('X')
        cyl.GetDisplayColorAttr().Set([color_v])
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(
            vine_pmat,
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose='physics',
        )
        UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(
            vine_vmat,
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        )

        UsdPhysics.RigidBodyAPI.Apply(link_xform.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(link_xform.GetPrim())
        mass_api.GetMassAttr().Set(link_mass)
        ixx = (1.0 / 12.0) * link_mass * (3 * link_radius**2 + link_length**2)
        izz = 0.5 * link_mass * link_radius**2
        mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(ixx, ixx, izz))

        rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(link_xform.GetPrim())
        rb_api.CreateLinearDampingAttr().Set(LINK_LINEAR_DAMPING)
        rb_api.CreateAngularDampingAttr().Set(LINK_ANGULAR_DAMPING_RB)

        joint = UsdPhysics.SphericalJoint.Define(stage, f'{link_path}/joint')
        end_l = Gf.Vec3f(-link_length / 2.0, 0.0, 0.0)
        end_r = Gf.Vec3f(link_length / 2.0, 0.0, 0.0)
        if i == 0:
            joint.GetBody0Rel().SetTargets([Sdf.Path(left_anchor)])
            joint.GetBody1Rel().SetTargets([Sdf.Path(link_path)])
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            joint.GetLocalPos1Attr().Set(end_l)
        else:
            assert prev_link_path is not None
            joint.GetBody0Rel().SetTargets([Sdf.Path(prev_link_path)])
            joint.GetBody1Rel().SetTargets([Sdf.Path(link_path)])
            joint.GetLocalPos0Attr().Set(end_r)
            joint.GetLocalPos1Attr().Set(end_l)

        physx_joint = PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())
        physx_joint.CreateArmatureAttr().Set(joint_armature)
        joint.GetConeAngle0LimitAttr().Set(cone_angle_deg)
        joint.GetConeAngle1LimitAttr().Set(cone_angle_deg)

        if joint_angular_damping > 0.0:
            for axis_token in ('rotX', 'rotY', 'rotZ'):
                ang = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis_token)
                ang.GetStiffnessAttr().Set(0.0)
                ang.GetDampingAttr().Set(joint_angular_damping)
                ang.GetTargetPositionAttr().Set(0.0)

        prev_link_path = link_path

    assert prev_link_path is not None
    prismatic = UsdPhysics.PrismaticJoint.Define(
        stage, f'{vine_path}/loop_prismatic'
    )
    prismatic.GetBody0Rel().SetTargets([Sdf.Path(right_anchor)])
    prismatic.GetBody1Rel().SetTargets([Sdf.Path(prev_link_path)])
    prismatic.GetAxisAttr().Set('X')
    prismatic.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    prismatic.GetLocalPos1Attr().Set(Gf.Vec3f(link_length / 2.0, 0.0, 0.0))
    prismatic.GetLowerLimitAttr().Set(-prismatic_limit)
    prismatic.GetUpperLimitAttr().Set(prismatic_limit)

    drive = UsdPhysics.DriveAPI.Apply(prismatic.GetPrim(), 'linear')
    drive.GetStiffnessAttr().Set(drive_stiffness)
    drive.GetDampingAttr().Set(drive_damping)
    drive.GetTargetPositionAttr().Set(0.0)
    drive.GetMaxForceAttr().Set(drive_max_force)


def spawn_underbrush_scenario(
    stage: Usd.Stage,
    parent_path: str = '/World/Vines',
) -> list[str]:
    """Spawn the four-cord scenario from ROS1 underbrush.launch.

    ROS1 positions are the starting end of a +Y-extending chain;
    convert to center+yaw=pi/2.
    """
    UsdGeom.Xform.Define(stage, parent_path)

    starts = [
        ('underbrush_0', (0.60, -0.50, 0.20)),
        ('underbrush_1', (1.03, -0.50, 0.12)),
        ('underbrush_2', (1.03, -0.50, 0.23)),
        ('underbrush_3', (1.36, -0.50, 0.15)),
    ]
    half_len = (NUM_LINKS * LINK_LENGTH) / 2.0
    yaw = math.pi / 2.0

    paths: list[str] = []
    for name, (sx, sy, sz) in starts:
        cx, cy = sx, sy + half_len
        path = f'{parent_path}/{name}'
        _spawn_single_vine(
            stage=stage,
            vine_path=path,
            center_xyz=(cx, cy, sz),
            yaw_rad=yaw,
        )
        paths.append(path)
    return paths
