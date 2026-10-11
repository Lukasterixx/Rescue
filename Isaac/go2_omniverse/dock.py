"""The Go2 EDU's Jetson expansion dock and the two rails on it that the D1 stands on: the meshes, and how to read them.

Copied from Lukas's D1Training repository (`dock.py`, 054983a), with the meshes moved to `dock_meshes/`. The meshes are
Unitree's CAD ("GO2 simplified, with expansion dock", 2023-09-11), tessellated and registered to the Go2's `base_link`
by D1Training's `description/dock_from_cad.py`, so they are drawn on the trunk with no placement of their own. They
are drawn only: no collider and no mass (`arm_weld.py`). The rails' tops are the mount's height,
`arm_weld.D1_MOUNT_POS` (D1Training F-117). The colours are a choice, not the robot's. Provenance and licence:
`dock_meshes/NOTICE.md`.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

MESH_DIR = Path(__file__).resolve().parent / "dock_meshes"

#: (name, binary STL in MESH_DIR, RGB): the dock's body, then its rails.
DOCK_MESHES = (("expansion_dock", "expansion_dock.stl", (0.30, 0.30, 0.32)),
               ("expansion_dock_rails", "expansion_dock_rails.stl", (0.72, 0.73, 0.75)))

_STL_RECORD = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])


def read_binary_stl(path: str | Path) -> np.ndarray:
    """A binary STL's triangles, (n, 3, 3), in the file's units (metres, for these)."""
    data = Path(path).read_bytes()
    count = int.from_bytes(data[80:84], "little")
    if len(data) != 84 + count * _STL_RECORD.itemsize:
        raise ValueError(f"{path}: not a binary STL of {count} triangles ({len(data)} bytes)")
    return np.frombuffer(data, dtype=_STL_RECORD, count=count, offset=84)["vertices"].astype(np.float64)
