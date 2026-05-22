"""Convert the Spirit URDF to USD via Isaac Sim's URDF importer.

Isaac Sim 5.1's URDF Importer GUI silently fails on this URDF; running
it via omni.kit.commands surfaces real errors and is reproducible.

Usage:
    ./isaaclab.sh -p import_urdf_to_usd.py
"""

from __future__ import annotations

import sys
from pathlib import Path

from isaaclab.app import AppLauncher  # must precede any omni.* import

URDF_INPUT = Path('/tmp/spirit_isaac_clean.urdf')
USD_OUTPUT = Path(
    '/home/rml/ros2_ws/src/quad-sdk/quad_simulator/'
    'spirit_description/models/spirit/usd/spirit.usd'
)


def main() -> None:
    if not URDF_INPUT.exists():
        sys.exit(f'URDF not found at {URDF_INPUT}; run the prep pipeline first')

    USD_OUTPUT.parent.mkdir(parents=True, exist_ok=True)

    app_launcher = AppLauncher(headless=True)
    simulation_app = app_launcher.app

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
    cfg.create_physics_scene = True
    cfg.make_default_prim = True

    print(f'Parsing {URDF_INPUT}')
    parse_ok, robot_model = omni.kit.commands.execute(
        'URDFParseFile',
        urdf_path=str(URDF_INPUT),
        import_config=cfg,
    )
    if not parse_ok:
        sys.exit('URDFParseFile returned False; URDF parse failed')

    n_links = len(robot_model.links) if robot_model else -1
    n_joints = len(robot_model.joints) if robot_model else -1
    print(f'Parsed: {n_links} links, {n_joints} joints')

    print(f'Writing {USD_OUTPUT}')
    import_ok, prim_path = omni.kit.commands.execute(
        'URDFImportRobot',
        urdf_path=str(URDF_INPUT),
        urdf_robot=robot_model,
        import_config=cfg,
        dest_path=str(USD_OUTPUT),
    )
    if not import_ok:
        sys.exit('URDFImportRobot returned False; USD write failed')

    print(f'Imported as prim {prim_path}')
    print(f'USD size: {USD_OUTPUT.stat().st_size} bytes')
    simulation_app.close()


if __name__ == '__main__':
    main()
