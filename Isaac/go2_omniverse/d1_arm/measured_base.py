"""Make `meshes/base_link_measured.STL`, the D1's base at the size Lukas measured on the real arm.

    python3 d1_arm/measured_base.py

Unitree's `meshes/base_link.STL` is smaller and shorter than the real base. It is kept, unchanged, beside the
result. Measured on the arm, 2026-09-30:

| | `base_link.STL` | the real arm |
| --- | --- | --- |
| Cylinder diameter | 105.0 mm | 126 mm |
| Mounting holes, along the rail | 108 mm apart | 130 mm |
| Mounting holes, across the rail | 58 mm apart | 58 mm (2026-10-06) |
| Bottom of the base to the rotating top plate (Joint1) | 53.3 mm | 76 mm |
| Rotating plate to the shoulder pivot (Joint2) | 56.3 mm | the same |

The two plan dimensions agree on one factor, 1.20, so the base is scaled by 1.20 in x and y about its axis.
Its height does not follow that factor (76 / 53.3 = 1.43), so height is not scaled. The flange, the bottom
5.0 mm, is kept, the cylinder from there to Joint1 is stretched to end at 76.0 mm, and anything above Joint1
rides up with it by 22.7 mm. Everything above the base was checked unchanged (the shoulder height above), and
the rest of the arm matches Unitree's published 550 mm reach, so no other mesh moves. `d1.urdf` points at this
file and puts Joint1 at 76.0 mm.

Across the rail the holes did not grow: they are 58 mm apart on the real arm, as on Unitree's (Lukas, 2026-10-06), and
over the rails' screws. So the flange and its lip (Unitree's bottom 7.0 mm) keep Unitree's across-rail size at the
mounting tabs: y is scaled 1.0 where |x| is beyond 51.5 mm on Unitree's base, which holds the holes whole (they span
51.75-56.25 mm), and 1.20 inside 48 mm, blended smoothly between. Each point's y moves by a factor of its x alone, and
the blend is wide enough for the mesh's triangles to follow it: none flips against the plain 1.20 scaling. Where the
lip meets the cylinder, at the tabs, the two differ by up to 1.8 mm. The cylinder above keeps the plain 1.20.

Not measured: the flange's outline (the tabs' length along the rail is scaled 1.20 with the holes; their width
across is Unitree's), and where the real base's mass sits.

Since 2026-10-09 the arm above Joint1 has Unitree's D1-550 CAD lengths (F-138), which put the shoulder pivot 54.8 mm
above Joint1 (130.8 mm above the base's bottom face; the CAD's plate top is at 75.5). That is 1.5 mm under the
2026-09-30 reading in the table above (76 + 56.3 = 132.3 mm). This script and its mesh are unchanged by it.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SOURCE = HERE / "meshes" / "base_link.STL"
OUT = HERE / "meshes" / "base_link_measured.STL"

PLAN_SCALE = 1.20                 # 126 / 105 (cylinder), 130 / 108 (holes)
TABS_ACROSS_SCALE = 1.0           # the holes across the rail: 58 mm apart, real and Unitree's
TABS_BLEND_X_M = (0.0480, 0.0515)  # |x| on Unitree's base over which the flange's across-rail scale goes 1.20 -> 1.0
TABS_TOP_M = 0.0070               # the flange and its lip, on Unitree's base: what keeps the tabs' across-rail size
FLANGE_TOP_M = 0.0050             # the flange keeps its thickness
JOINT1_MODEL_M = 0.0533           # d1.urdf's Joint1 before
JOINT1_REAL_M = 0.0760            # measured: bottom of the base to the rotating plate

_RECORD = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])


def base_height(z: np.ndarray) -> np.ndarray:
    """The measured base's height for a point at height `z` (metres) on Unitree's."""
    stretch = (JOINT1_REAL_M - FLANGE_TOP_M) / (JOINT1_MODEL_M - FLANGE_TOP_M)
    return np.where(z <= FLANGE_TOP_M, z,
                    np.where(z <= JOINT1_MODEL_M, FLANGE_TOP_M + (z - FLANGE_TOP_M) * stretch,
                             z + (JOINT1_REAL_M - JOINT1_MODEL_M)))


def across_scale(x: np.ndarray, z: np.ndarray) -> np.ndarray:
    """The across-rail (y) scale for a point at `x`, `z` (metres) on Unitree's base: 1.20, except the flange's tabs."""
    w = np.clip((np.abs(x) - TABS_BLEND_X_M[0]) / (TABS_BLEND_X_M[1] - TABS_BLEND_X_M[0]), 0.0, 1.0)
    w = w * w * (3.0 - 2.0 * w) * (z <= TABS_TOP_M + 1e-6)
    return PLAN_SCALE + (TABS_ACROSS_SCALE - PLAN_SCALE) * w


def measured_point(p) -> np.ndarray:
    """A point (metres, base_link frame) on Unitree's base, carried to the measured one."""
    p = np.asarray(p, dtype=np.float64)
    return np.stack([p[..., 0] * PLAN_SCALE, p[..., 1] * across_scale(p[..., 0], p[..., 2]), base_height(p[..., 2])],
                    axis=-1)


def main() -> None:
    data = SOURCE.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    if len(data) != 84 + count * _RECORD.itemsize:
        raise SystemExit(f"{SOURCE}: not a binary STL")
    tri = np.frombuffer(data, dtype=_RECORD, count=count, offset=84)["vertices"].astype(np.float64)
    out = measured_point(tri)
    normals = np.cross(out[:, 1] - out[:, 0], out[:, 2] - out[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-15)
    records = np.zeros(count, dtype=_RECORD)
    records["normal"], records["vertices"] = normals, out
    header = b"D1 base_link, scaled to the base measured on the real arm (d1_arm/measured_base.py)".ljust(80)[:80]
    OUT.write_bytes(header + np.uint32(count).tobytes() + records.tobytes())
    lo, hi = out.reshape(-1, 3).min(axis=0), out.reshape(-1, 3).max(axis=0)
    print(f"wrote {OUT}: {count} triangles, x {lo[0]*1000:.1f}..{hi[0]*1000:.1f}, y {lo[1]*1000:.1f}..{hi[1]*1000:.1f}, "
          f"z {lo[2]*1000:.1f}..{hi[2]*1000:.1f} mm")


if __name__ == "__main__":
    main()
