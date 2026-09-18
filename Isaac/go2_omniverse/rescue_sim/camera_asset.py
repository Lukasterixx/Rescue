"""The RealSense D435's case, drawn on the wrist where the mount puts the camera. Visual only.

Copied from Lukas's D1Training repository (commit 5e19028): the frame registration from
`demos/cup/pick_demo/camera_body.py` and the USD builder from `camera_asset.py`, unchanged apart from dropping what
the simulator does not use (clearance reports, the view-obstruction guard). The mesh is Intel's own CAD from
`realsense2_description` (Apache-2.0, `assets/realsense/NOTICE.md`).

Everything is in the colour optical frame -- x right, y down, z forward -- so the prim takes the mount's own pose.
`MESH_TO_OPTICAL` is derived from `_d435.urdf.xacro`'s offsets. The colour lens element is removed, because the
simulated camera is a pinhole at the sensor plane and that lens sits on its axis half a millimetre away.

No collider, no mass, no rigid-body properties: it cannot change the physics. The wrist camera's near clip
(`realsense.NEAR_CLIP_M`) keeps it out of the camera's own images, as in D1Training (F-054, F-052).

Needs `pxr` and `trimesh`, so it runs inside the Isaac app, after `AppLauncher`.
"""
from __future__ import annotations

from functools import lru_cache
import hashlib
import json
from pathlib import Path

import numpy as np

ASSET_DIR = Path(__file__).resolve().parent / "assets" / "realsense"
MESH_PATH = ASSET_DIR / "d435_housing.ply"

# Measured on D435I 238222076237 (pick_demo.realsense calibrate, 2026-09-17): the depth origin -- the
# left infrared imager -- sits this far along the colour frame's +x. Positive: to the right of colour.
COLOUR_FROM_LEFT_IMAGER_M = 0.014857
# Measured on the same camera, from the two infrared streams' extrinsics: 50.05 mm, against the
# datasheet's nominal 50 mm.
STEREO_BASELINE_M = 0.05004734918475151

# From realsense2_description/urdf/_d435.urdf.xacro, which cites the datasheet Rev 007 Fig. 4-4 p.65.
GLASS_AHEAD_OF_OPTICAL_M = 0.0042        # optical centres sit this far behind the front cover glass
GLASS_BEHIND_PLATE_M = 0.0001            # glass this far behind the front aluminium plate
MESH_DEPTH_FROM_SCREW_M = 0.0175         # depth origin, along the case, from the tripod screw axis
NOMINAL_COLOUR_FROM_DEPTH_M = 0.015      # the xacro's nominal for COLOUR_FROM_LEFT_IMAGER_M
MOUNT_FROM_CENTRE_M = 0.0149             # tripod screw, into the case from the front plate
# The mesh origin's offset from the depth origin along the optical axis, as the xacro composes it.
MESH_ORIGIN_BEHIND_OPTICAL_M = MOUNT_FROM_CENTRE_M - GLASS_BEHIND_PLATE_M - GLASS_AHEAD_OF_OPTICAL_M
# Where the mesh's long-axis origin (the tripod screw axis) sits in the colour optical frame. The
# xacro's nominal colour offset is used, not the measured one, so that the CAD registers against
# itself: the mesh's own colour lens barrel then lands on the optical axis, which is the check in
# `demos/cup/tests/test_pick_demo.py`. The bench camera's measured 14.857 mm differs from the nominal 15 mm by
# 0.14 mm -- device tolerance, not a registration choice.
SCREW_TO_COLOUR_M = MESH_DEPTH_FROM_SCREW_M + NOMINAL_COLOUR_FROM_DEPTH_M

# Datasheet nominal, kept for the docstring's sake; the CAD's own extent is what the code uses.
HOUSING_SIZE_NOMINAL_M = (0.090, 0.025, 0.025)


def mesh_to_optical() -> np.ndarray:
    """4x4 taking a point of `d435_housing.ply` into the colour optical frame.

    Derived from `_d435.urdf.xacro`. The mesh's own axes are x along the case, y up, z out of the
    front plate. The xacro mounts it in the camera link with rpy (pi/2, 0, pi/2) at
    (glass + plate, -depth_py, 0), and the colour optical frame is the link rotated by
    rpy (-pi/2, 0, -pi/2) at (0, +colour_offset, 0). Composing the two gives, for a mesh point p:

        optical x =  (screw_to_colour) - p.x        with screw_to_colour = depth_py + colour offset
        optical y = -p.y
        optical z =  p.z + glass + plate

    The rotation is a half turn about the optical z: the mesh runs along its +x away from the colour
    lens, the optical frame's +x runs back towards it, and the mesh's +y is up where the optical +y is
    down.
    """
    out = np.array([
        [-1.0, 0.0, 0.0, SCREW_TO_COLOUR_M],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, GLASS_AHEAD_OF_OPTICAL_M + GLASS_BEHIND_PLATE_M],
        [0.0, 0.0, 0.0, 1.0],
    ])
    return out


MESH_TO_OPTICAL = mesh_to_optical()


@lru_cache(maxsize=1)
def mesh_optical() -> tuple:
    """The case mesh in the colour optical frame, as (points (N,3), triangles (T,3)).

    Cached: the file is 7 MB and 231186 triangles, and every caller wants the same transform applied.
    Needs `trimesh`, which the Isaac environment has.
    """
    import trimesh

    mesh = trimesh.load(str(MESH_PATH))
    points = np.asarray(mesh.vertices, dtype=float)
    points = (MESH_TO_OPTICAL @ np.c_[points, np.ones(len(points))].T).T[:, :3]
    return points, np.asarray(mesh.faces, dtype=np.int64)


# The widest colour frustum of anything in `realsense.PRESETS` is the D455 preset's: 0.833 of the depth
# across, 0.625 down. Clearing 0.90 leaves every model here an open aperture with room to spare, and
# still only reaches the lens element -- the barrel around it starts 2.85 mm off the axis.
LENS_CLEARANCE_TAN = 0.90
BUILDER_VERSION = 1


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def carve_lens(points: np.ndarray, faces: np.ndarray, tan: float = LENS_CLEARANCE_TAN):
    """Drop every triangle inside the clearance cone. Returns (faces_kept, dropped_count).

    A triangle is inside if any of it lies at positive depth and its x and y spans both overlap the
    cone measured at the far end of its own depth range, which is where the cone is widest -- the same
    conservative test D1Training's `camera_body.view_obstruction` uses.
    """
    tri = points[faces]
    zmax = tri[:, :, 2].max(axis=1)
    half = tan * zmax
    inside = ((zmax > 0.0)
              & (tri[:, :, 0].min(axis=1) <= half) & (tri[:, :, 0].max(axis=1) >= -half)
              & (tri[:, :, 1].min(axis=1) <= half) & (tri[:, :, 1].max(axis=1) >= -half))
    return faces[~inside], int(inside.sum())


def build_camera_usd(out_dir, source=None, colour=(0.62, 0.63, 0.65), roughness: float = 0.35,
                     tan: float = LENS_CLEARANCE_TAN) -> dict:
    """Write (or reuse) the visual camera asset. Returns its path and the numbers that define it.

    The asset's frame **is** the colour optical frame: x right, y down, z forward, origin at the
    colour sensor. Spawn it with the mount's own pose and it lands where the camera is.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

    source = Path(source or MESH_PATH).resolve()
    params = {"source_sha256": _sha256(source), "colour": list(colour), "roughness": roughness,
              "clearance_tan": tan, "transform": np.asarray(MESH_TO_OPTICAL).round(9).tolist(),
              "builder": BUILDER_VERSION}
    key = hashlib.sha256(json.dumps(params, sort_keys=True).encode()).hexdigest()[:12]
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"d435_{key}.usd"
    info_path = out_dir / f"d435_{key}.json"
    if out_path.is_file() and info_path.is_file():
        return json.loads(info_path.read_text())

    points, faces = mesh_optical()
    kept, dropped = carve_lens(points, faces, tan)
    lo, hi = points[np.unique(kept)].min(axis=0), points[np.unique(kept)].max(axis=0)

    if out_path.exists():
        out_path.unlink()
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/RealSenseD435i")
    stage.SetDefaultPrim(root.GetPrim())

    look = UsdShade.Material.Define(stage, "/RealSenseD435i/Looks/Aluminium")
    shader = UsdShade.Shader.Define(stage, "/RealSenseD435i/Looks/Aluminium/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.85)
    look.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    mesh = UsdGeom.Mesh.Define(stage, "/RealSenseD435i/Visual")
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(kept), 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(kept.astype(np.int32).ravel()))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(
        np.array([points.min(axis=0), points.max(axis=0)], dtype=np.float32)))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(look)
    stage.GetRootLayer().Save()

    info = {
        "usd_path": str(out_path), "source": str(source), **params,
        "triangles": int(len(kept)), "triangles_removed": dropped, "vertices": int(len(points)),
        "bounds_optical_m": [[round(float(v), 6) for v in lo], [round(float(v), 6) for v in hi]],
        "frame": "colour optical: x right, y down, z forward, origin at the colour sensor",
        "note": ("Intel's d435.dae (realsense2_description, Apache-2.0), registered by "
                 "_d435.urdf.xacro's offsets, with the colour lens element removed so a pinhole "
                 "camera at the sensor plane has a clear aperture. Visual only."),
    }
    info_path.write_text(json.dumps(info, indent=2) + "\n")
    return info
