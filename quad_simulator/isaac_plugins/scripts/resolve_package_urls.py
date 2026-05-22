#!/usr/bin/env python3
"""Resolve package:// URLs in a URDF to absolute filesystem paths.

Isaac Sim's URDF importer does not understand package:// URIs. This script
expands them by querying ament_index for each referenced ROS2 package and
rewriting the URI to a file:// or absolute path.

Usage:
    resolve_package_urls.py <input.urdf> <output.urdf>
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

PACKAGE_URI_RE = re.compile(r'package://([^/"\s]+)(/[^"\s]*)')


def resolve(input_path: Path, output_path: Path) -> None:
    text = input_path.read_text()

    package_cache: dict[str, str] = {}

    def _replace(match: re.Match) -> str:
        package_name, rel_path = match.group(1), match.group(2)
        if package_name not in package_cache:
            package_cache[package_name] = get_package_share_directory(package_name)
        return package_cache[package_name] + rel_path

    rewritten = PACKAGE_URI_RE.sub(_replace, text)

    missing = [
        m.group(0)
        for m in PACKAGE_URI_RE.finditer(rewritten)
    ]
    if missing:
        raise RuntimeError(f"Unresolved package URIs remain: {missing}")

    output_path.write_text(rewritten)
    print(f"Resolved {len(package_cache)} package(s): {sorted(package_cache)}")
    print(f"Wrote {output_path}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        sys.exit(1)
    resolve(Path(sys.argv[1]), Path(sys.argv[2]))
