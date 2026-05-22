#!/usr/bin/env python3
"""Strip Gazebo/Isaac-importer tags from a URDF before Isaac Sim import.

Removes <gazebo>, <transmission>, <ros2_control> blocks (Isaac doesn't
read them but the importer leaves orphan prims) and the
joint dont_collapse="true" attribute (keeps toes as leaf rigid bodies
so the bridge's SingleRigidPrim wrappers can read pose/velocity).

Also repairs link inertials that would otherwise trip Isaac's URDF
importer:
  - Links with no <inertial> block get a tiny default one. Without
    this Isaac assigns a default mass (~1 kg) per link, which for
    go2 dumps ~9 kg of phantom mass on the calf chain and wrecks
    the controller's dynamics.
  - Links with <mass value="0"/> (and zero inertia tensor) get a
    tiny non-zero mass. Isaac would otherwise log "negative mass"
    warnings and fall back to a sphere approximation.

Usage:
    strip_gazebo_tags.py <input.urdf> <output.urdf>
"""

from __future__ import annotations

import sys
from pathlib import Path
from xml.etree import ElementTree as ET

DEFAULT_MASS = 1e-3
DEFAULT_INERTIA = 1e-6


def _make_inertial(mass: float = DEFAULT_MASS,
                   inertia: float = DEFAULT_INERTIA) -> ET.Element:
    inertial = ET.Element('inertial')
    ET.SubElement(
        inertial, 'origin', {'rpy': '0 0 0', 'xyz': '0 0 0'},
    )
    ET.SubElement(inertial, 'mass', {'value': repr(mass)})
    ET.SubElement(inertial, 'inertia', {
        'ixx': repr(inertia), 'ixy': '0', 'ixz': '0',
        'iyy': repr(inertia), 'iyz': '0', 'izz': repr(inertia),
    })
    return inertial


def _inject_default_inertials(root: ET.Element) -> int:
    """Add a tiny <inertial> to every <link> missing one."""
    injected = 0
    for link in root.iter('link'):
        if link.find('inertial') is None:
            link.append(_make_inertial())
            injected += 1
    return injected


def _repair_zero_mass_inertials(root: ET.Element) -> int:
    """Replace <mass value="0"/> + zero inertia tensors with tiny defaults."""
    repaired = 0
    for link in root.iter('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass_el = inertial.find('mass')
        if mass_el is None:
            continue
        try:
            mass = float(mass_el.get('value', '0'))
        except ValueError:
            mass = 0.0
        if mass > 0.0:
            continue
        mass_el.set('value', repr(DEFAULT_MASS))
        inertia_el = inertial.find('inertia')
        if inertia_el is not None:
            for axis in ('ixx', 'iyy', 'izz'):
                try:
                    if float(inertia_el.get(axis, '0')) <= 0.0:
                        inertia_el.set(axis, repr(DEFAULT_INERTIA))
                except ValueError:
                    inertia_el.set(axis, repr(DEFAULT_INERTIA))
        repaired += 1
    return repaired


def strip(input_path: Path, output_path: Path) -> None:
    """Remove Gazebo-only blocks (and Isaac-importer hint attrs) from a URDF."""
    tree = ET.parse(input_path)
    root = tree.getroot()

    targets = ('gazebo', 'transmission', 'ros2_control')
    removed = {tag: 0 for tag in targets}

    for parent in root.iter():
        for child in list(parent):
            tag = child.tag.split('}')[-1]
            if tag in targets:
                parent.remove(child)
                removed[tag] += 1

    dont_collapse_stripped = 0
    for joint in root.iter('joint'):
        if 'dont_collapse' in joint.attrib:
            del joint.attrib['dont_collapse']
            dont_collapse_stripped += 1

    inertials_injected = _inject_default_inertials(root)
    inertials_repaired = _repair_zero_mass_inertials(root)

    tree.write(output_path, xml_declaration=True, encoding='utf-8')
    for tag in targets:
        print(f'Removed {removed[tag]} <{tag}> blocks')
    print(f'Stripped dont_collapse attr from {dont_collapse_stripped} joint(s)')
    print(f'Injected default <inertial> on {inertials_injected} link(s)')
    print(f'Repaired zero-mass <inertial> on {inertials_repaired} link(s)')
    print(f'Wrote {output_path}')


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        sys.exit(1)
    strip(Path(sys.argv[1]), Path(sys.argv[2]))
