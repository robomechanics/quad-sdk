#!/usr/bin/env python3
"""Rename Spirit URDF's numeric joint names to identifiers Isaac/USD accept.

The Spirit URDF names joints with bare integers (0..11). USD prim paths cannot
start with a digit, and Isaac Sim's URDF importer either rejects or mangles
these names. We rewrite to leg-based identifiers:

    abad joint  hip joint   knee joint
leg 0:    8          0           1     -> j_abad_0  j_hip_0  j_knee_0
leg 1:    9          2           3     -> j_abad_1  j_hip_1  j_knee_1
leg 2:   10          4           5     -> j_abad_2  j_hip_2  j_knee_2
leg 3:   11          6           7     -> j_abad_3  j_hip_3  j_knee_3

Updates joint definitions, transmission references, motor/transmission names.
Link names ('hip0', 'upper0', etc.) are unchanged — they already start with
a letter.

Usage:
    rename_joints.py <input.urdf> <output.urdf>
"""

from __future__ import annotations

import sys
from pathlib import Path
from xml.etree import ElementTree as ET

JOINT_RENAME = {
    "8":  "j_abad_0", "0": "j_hip_0", "1": "j_knee_0",
    "9":  "j_abad_1", "2": "j_hip_1", "3": "j_knee_1",
    "10": "j_abad_2", "4": "j_hip_2", "5": "j_knee_2",
    "11": "j_abad_3", "6": "j_hip_3", "7": "j_knee_3",
}


def rename(input_path: Path, output_path: Path) -> None:
    tree = ET.parse(input_path)
    root = tree.getroot()

    renamed_joints = 0
    renamed_refs = 0
    renamed_motors = 0
    renamed_transmissions = 0

    for joint in root.iter("joint"):
        name = joint.get("name")
        if name in JOINT_RENAME:
            joint.set("name", JOINT_RENAME[name])
            if joint.get("type") is not None:
                renamed_joints += 1
            else:
                renamed_refs += 1

    for actuator in root.iter("actuator"):
        name = actuator.get("name", "")
        if name.startswith("motor_"):
            suffix = name.removeprefix("motor_")
            if suffix in JOINT_RENAME:
                actuator.set("name", "motor_" + JOINT_RENAME[suffix])
                renamed_motors += 1

    for transmission in root.iter("transmission"):
        name = transmission.get("name", "")
        if name.startswith("transmission_"):
            suffix = name.removeprefix("transmission_")
            if suffix in JOINT_RENAME:
                transmission.set("name", "transmission_" + JOINT_RENAME[suffix])
                renamed_transmissions += 1

    tree.write(output_path, xml_declaration=True, encoding="utf-8")
    print(f"Renamed {renamed_joints} joint definitions")
    print(f"Renamed {renamed_refs} joint references (in transmissions)")
    print(f"Renamed {renamed_motors} motor names")
    print(f"Renamed {renamed_transmissions} transmission names")
    print(f"Wrote {output_path}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        sys.exit(1)
    rename(Path(sys.argv[1]), Path(sys.argv[2]))
