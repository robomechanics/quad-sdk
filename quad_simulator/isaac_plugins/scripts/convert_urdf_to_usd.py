r"""One-shot URDF -> USD converter for quad-sdk robots.

Wraps IsaacLab's UrdfConverter and adds quad-sdk post-processing:
    * Apply per-robot armature (reflected motor inertia, J_rotor * N^2);
      URDF can't express it but PhysX needs it.
    * Disable joint drives so robot_driver is the only torque source.
    * Flatten + save as .usda for diff-friendly artifacts.

Run once per robot, commit the .usda. To add a robot, extend
ARMATURE_TABLE below.

Usage:
    python convert_urdf_to_usd.py \
        --robot spirit \
        --urdf  /tmp/spirit_isaac_clean.urdf \
        --out   /path/to/spirit_description/usd/spirit.usda \
        [--gui]
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


# Per-robot (joint_regex, armature_kg_m2). First match wins.
# Spirit: J_rotor = 1.231e-4 kg.m^2 (from spirit_rotors.sdf.xacro),
# gear ratios abad/hip=6, knee=12 -> armature = J_rotor * N^2.
ARMATURE_TABLE: dict[str, list[tuple[str, float]]] = {
    'spirit': [
        (r'^j_abad_[0-3]$', 4.4316e-3),
        (r'^j_hip_[0-3]$',  4.4316e-3),
        (r'^j_knee_[0-3]$', 1.77264e-2),
    ],
}


def parse_args() -> argparse.Namespace:
    """Parse CLI args; AppLauncher consumes its own subset before us."""
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0]
    )
    parser.add_argument(
        '--robot', required=True, choices=sorted(ARMATURE_TABLE),
        help='Robot key - selects the armature table.',
    )
    parser.add_argument(
        '--urdf', required=True, type=Path, help='Input URDF path.',
    )
    parser.add_argument(
        '--out', required=True, type=Path,
        help='Output .usda path (will be overwritten).',
    )
    parser.add_argument(
        '--gui', action='store_true',
        help='Open the converted USD in Isaac for visual inspection.',
    )
    parser.add_argument(
        '--no-armature', action='store_true',
        help='Skip armature application; useful for debugging instability.',
    )

    # Let AppLauncher accept its own flags (includes --headless).
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


def main() -> None:
    """Convert URDF to USD, apply armature, save as .usda."""
    args = parse_args()

    if args.gui:
        args.headless = False

    if not args.urdf.is_file():
        raise SystemExit(f'URDF not found: {args.urdf}')
    args.out = args.out.expanduser().resolve()
    args.out.parent.mkdir(parents=True, exist_ok=True)

    from isaaclab.app import AppLauncher
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    import re
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
    from pxr import Usd, UsdPhysics, PhysxSchema

    # Drives off so robot_driver is the only torque source.
    print(f'[convert] running UrdfConverter on {args.urdf}')
    tmp_usd_dir = args.out.parent / '_urdf_tmp'
    cfg = UrdfConverterCfg(
        asset_path=str(args.urdf),
        usd_dir=str(tmp_usd_dir),
        fix_base=False,
        merge_fixed_joints=False,
        force_usd_conversion=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=0.0,
                damping=0.0,
            ),
            target_type='none',
        ),
    )
    converter = UrdfConverter(cfg)
    src_usd = Path(converter.usd_path)
    print(f'[convert] UrdfConverter wrote {src_usd}')

    table = ARMATURE_TABLE[args.robot]
    compiled = [(re.compile(pat), val) for pat, val in table]

    print(f'[convert] opening {src_usd} to apply armature')
    stage = Usd.Stage.Open(str(src_usd))
    if stage is None:
        raise SystemExit(f'Could not open generated USD: {src_usd}')

    if args.no_armature:
        print('[convert] --no-armature set: skipping armature step')
    else:
        set_count = 0
        skipped = []
        for prim in stage.Traverse():
            if not prim.IsA(UsdPhysics.Joint):
                continue
            name = prim.GetName()
            for rx, val in compiled:
                if rx.match(name):
                    physx = PhysxSchema.PhysxJointAPI.Apply(prim)
                    physx.CreateArmatureAttr().Set(float(val))
                    print(f'[convert]   armature {name} = {val:.5e}')
                    set_count += 1
                    break
            else:
                skipped.append(name)

        print(f'[convert] armature applied on {set_count} joints; '
              f'{len(skipped)} joint(s) had no table match: {skipped}')

    # UrdfConverter writes a layered stage referencing payloads in an
    # intermediate dir; Flatten() composes it into one self-contained
    # artifact so we can delete the intermediate dir afterwards.
    args.out.unlink(missing_ok=True)
    flat = stage.Flatten()
    flat.Export(str(args.out))
    print(f'[convert] saved (flattened) {args.out}')

    if tmp_usd_dir.exists():
        shutil.rmtree(tmp_usd_dir, ignore_errors=True)

    if args.gui:
        print('[convert] --gui set: opening result for inspection. '
              'Close the window to exit.')
        import omni.usd
        omni.usd.get_context().open_stage(str(args.out))
        import omni.kit.app
        app = omni.kit.app.get_app_interface()
        try:
            while app.is_running():
                app.update()
        except KeyboardInterrupt:
            pass

    simulation_app.close()


if __name__ == '__main__':
    main()
