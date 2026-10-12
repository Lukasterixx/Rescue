"""The competition's arenas for VIP-Rescue's website: a 3D model of each, in its own frame, and the walls a robot
there aligns its route to.

    python3 -m competition.web_export                       # into $VIP_RESCUE_ROOT (default ~/VIP-Rescue)
    python3 -m competition.web_export --out /some/maps/dir  # or anywhere

It writes, under the website's `maps/` directory:
  arenas/index.json       every arena: title, model, walls, start pose, START/END markers, bounds
  arenas/<key>.glb        the arena's meshes (glTF 2.0, one primitive per colour), in the arena's frame
  arenas/<key>.walls.json the arena's walls: 5 cm cells where something stands 0.65-3.0 m above the floor the robot
                          starts on, the height band VIP-Rescue's flatten_node draws /map2d from
  routes/krail_square.json the K-Rail Square's course route (training/measure_square.py's), as a route file

THE ARENA'S FRAME is the lane's origin with the world's axes (`rescue_sim.levels.Level.origin`): for the K-Rail
Square its centre, +x east, +y north. The rescue sim's `/sim/teleport` takes poses in it, and a route drawn on the
model is in it. glTF is +y up, so the models are stored turned (x, z, -y); a viewer turns them back with a +90 deg
rotation about x.

THE WALLS are sampled from the collision meshes, every face of them (a horizontal one in the band, such as the
stairs' landing, is a wall to /map2d too). The band is measured from the start floor (the spawn's height less the
0.42 m it drops the robot from): flatten_node keeps points from 0.3 m above the robot's base, and the base stands
about 0.35 m above the floor. The maze's tarp is left out of the model, so the maze can be seen from above, but not
out of the walls.

Left out: the gravel lane's 7,800 loose stones (its gravel surface stays), textures (each material is its colour).
No Isaac imports: the system python3 runs it.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import struct

import numpy as np

from .geometry import COLORS, Options, build_lanes

SCHEMA = "vip-rescue.arenas/1"
ROUTE_SCHEMA = "vip-rescue.route/1"
WALL_CELL_M = 0.05
WALL_BAND_M = (0.65, 3.0)            # above the start floor
SPAWN_DROP_M = 0.42                  # the spawn's height above the floor it stands on
SAMPLE_SPACING_M = 0.015
HIDDEN_MATERIALS = {"tarp"}          # in the walls, not in the model
# The K-Rail Square's course route and start, in its frame (training/measure_square.py, ROUTE).
SQUARE_ROUTE = [(-1.85, 0.6), (-0.75, 0.6), (-0.15, 0.85), (0.25, 0.85), (1.0, -0.3), (1.0, -0.65), (1.9, -0.65),
                (1.9, 0.9)]


def material_color(name: str) -> tuple[float, float, float]:
    """A material's colour; the acuity targets' Landolt-C pages are white."""
    return COLORS.get(name, (0.92, 0.92, 0.90))


# ------------------------------------------------------------------------------------------ the model
def glb(groups: dict[str, tuple[np.ndarray, np.ndarray]]) -> bytes:
    """A glTF 2.0 binary: one mesh, one primitive per material, flat-shaded (positions and normals per corner).

    `groups` maps a material name to (vertices (N, 3), faces (M, 3)) in the arena's frame, +z up."""
    buffer = bytearray()
    accessors, views, primitives, materials = [], [], [], []
    for index, (name, (vertices, faces)) in enumerate(sorted(groups.items())):
        corners = vertices[faces].reshape(-1, 3)                          # (3M, 3)
        tri = vertices[faces]
        normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        lengths = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = np.where(lengths > 1e-12, normals / np.maximum(lengths, 1e-12), [0.0, 0.0, 1.0])
        normals = np.repeat(normals, 3, axis=0)
        # glTF is +y up: (x, y, z) -> (x, z, -y).
        to_gltf = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float)
        for data, kind in ((corners @ to_gltf.T, "POSITION"), (normals @ to_gltf.T, "NORMAL")):
            raw = np.ascontiguousarray(data, dtype="<f4").tobytes()
            views.append({"buffer": 0, "byteOffset": len(buffer), "byteLength": len(raw), "target": 34962})
            accessor = {"bufferView": len(views) - 1, "componentType": 5126, "count": len(data), "type": "VEC3"}
            if kind == "POSITION":
                accessor["min"] = data.min(axis=0).astype(float).tolist()
                accessor["max"] = data.max(axis=0).astype(float).tolist()
            accessors.append(accessor)
            buffer.extend(raw)
            buffer.extend(b"\0" * (-len(buffer) % 4))
        primitives.append({"attributes": {"POSITION": len(accessors) - 2, "NORMAL": len(accessors) - 1},
                           "material": index})
        r, g, b = material_color(name)
        materials.append({"name": name, "pbrMetallicRoughness": {"baseColorFactor": [r, g, b, 1.0],
                                                                 "metallicFactor": 0.0, "roughnessFactor": 0.85},
                          "doubleSided": True})
    document = {
        "asset": {"version": "2.0", "generator": "Rescue competition/web_export.py"},
        "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0, "name": "arena"}],
        "meshes": [{"primitives": primitives}], "materials": materials,
        "buffers": [{"byteLength": len(buffer)}], "bufferViews": views, "accessors": accessors,
    }
    text = json.dumps(document, separators=(",", ":")).encode()
    text += b" " * (-len(text) % 4)
    chunks = struct.pack("<II", len(text), 0x4E4F534A) + text + struct.pack("<II", len(buffer), 0x004E4942) + buffer
    return struct.pack("<III", 0x46546C67, 2, 12 + len(chunks)) + chunks


def model_groups(lane) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    origin = np.asarray(lane.origin, dtype=float)
    parts: dict[str, list] = {}
    for mesh in lane.meshes:
        if mesh.material in HIDDEN_MATERIALS:
            continue
        overrides = {i: name for name, ids in mesh.material_faces.items() for i in np.asarray(ids).tolist()}
        by_material: dict[str, list[int]] = {}
        for i in range(len(mesh.faces)):
            by_material.setdefault(overrides.get(i, mesh.material), []).append(i)
        for name, ids in by_material.items():
            parts.setdefault(name, []).append((mesh.vertices - origin, mesh.faces[ids]))
    groups = {}
    for name, pieces in parts.items():
        vertices, faces, offset = [], [], 0
        for v, f in pieces:
            vertices.append(v)
            faces.append(f + offset)
            offset += len(v)
        groups[name] = (np.concatenate(vertices), np.concatenate(faces))
    return groups


# ------------------------------------------------------------------------------------------ the walls
def surface_samples(vertices: np.ndarray, faces: np.ndarray, spacing: float = SAMPLE_SPACING_M) -> np.ndarray:
    """Points over every face, about `spacing` apart (a regular barycentric grid per triangle)."""
    out = []
    for a, b, c in vertices[faces]:
        n = max(1, int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / spacing)))
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing="ij")
        keep = i + j <= n
        u, v = i[keep] / n, j[keep] / n
        out.append(a + np.outer(u, b - a) + np.outer(v, c - a))
    return np.concatenate(out) if out else np.zeros((0, 3))


def wall_cells(lane) -> tuple[list[list[float]], float]:
    """The arena's wall cells (centres, arena frame) and the start floor's height they are measured from."""
    origin = np.asarray(lane.origin, dtype=float)
    floor = lane.spawn[2] - SPAWN_DROP_M - origin[2]
    low, high = floor + WALL_BAND_M[0], floor + WALL_BAND_M[1]
    cells = set()
    for mesh in lane.meshes:
        if not mesh.collision:
            continue
        vertices = mesh.vertices - origin
        z = vertices[:, 2]
        if z.max() < low or z.min() > high:
            continue
        points = surface_samples(vertices, mesh.faces)
        points = points[(points[:, 2] >= low) & (points[:, 2] <= high)]
        for x, y in np.floor(points[:, :2] / WALL_CELL_M).astype(int):
            cells.add((int(x), int(y)))
    centres = sorted(((x + 0.5) * WALL_CELL_M, (y + 0.5) * WALL_CELL_M) for x, y in cells)
    return [[round(x, 3), round(y, 3)] for x, y in centres], floor


# ------------------------------------------------------------------------------------------ the index
def yaw_deg(rotation_wxyz) -> float:
    w, _, _, z = rotation_wxyz
    return math.degrees(2.0 * math.atan2(z, w))


def markers(lane) -> list[dict]:
    """The START and END zones, as outlines in the arena frame: the square's pads, a lane's end floors."""
    origin = np.asarray(lane.origin, dtype=float)
    out = []
    named = {"entry_pad": ("START", "blue"), "return_pad": ("END", "green")}
    for mesh in lane.meshes:
        if mesh.name in named:
            label, material = named[mesh.name]
            xy = mesh.vertices[:, :2] - origin[:2]
            (x0, y0), (x1, y1) = xy.min(axis=0), xy.max(axis=0)
            out.append({"label": label, "color": material, "polygon": [[round(float(x), 3), round(float(y), 3)]
                                                                         for x, y in ((x0, y0), (x1, y0), (x1, y1),
                                                                                      (x0, y1))]})
    for floor in lane.floors:
        if floor.name in ("blue_end", "green_end"):
            label, material = ("START", "blue") if floor.name == "blue_end" else ("END", "green")
            cx, cy = floor.center
            sx, sy = floor.size
            out.append({"label": label, "color": material,
                        "polygon": [[round(cx + dx * sx / 2, 3), round(cy + dy * sy / 2, 3)]
                                    for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]})
    return out


def export(maps_dir: Path, options: Options = Options(gravel="static")) -> dict:
    lanes = build_lanes(options)
    arenas_dir = maps_dir / "arenas"
    arenas_dir.mkdir(parents=True, exist_ok=True)
    index = {"schema": SCHEMA, "generator": "Rescue Isaac/go2_omniverse/competition/web_export.py",
             "wall_band_m": list(WALL_BAND_M), "wall_cell_m": WALL_CELL_M, "arenas": []}
    for lane in lanes:
        groups = model_groups(lane)
        (arenas_dir / f"{lane.key}.glb").write_bytes(glb(groups))
        cells, floor = wall_cells(lane)
        (arenas_dir / f"{lane.key}.walls.json").write_text(json.dumps(
            {"schema": "vip-rescue.walls/1", "frame": f"arena {lane.key}", "resolution": WALL_CELL_M,
             "band": [round(floor + WALL_BAND_M[0], 3), round(floor + WALL_BAND_M[1], 3)], "floor_z": round(floor, 3),
             "cells": cells}, separators=(",", ":")) + "\n")
        everything = np.concatenate([v for v, _ in groups.values()])
        origin = np.asarray(lane.origin, dtype=float)
        index["arenas"].append({
            "id": lane.key, "title": lane.title, "group": lane.arena, "setting": lane.setting,
            "subtitle": getattr(lane, "subtitle", ""),
            "model": f"arenas/{lane.key}.glb", "walls": f"arenas/{lane.key}.walls.json",
            "floor_z": round(floor, 3),
            "start": {"x": round(lane.spawn[0] - origin[0], 3), "y": round(lane.spawn[1] - origin[1], 3),
                      "yaw_deg": round(yaw_deg(lane.spawn_rotation), 2)},
            "bounds": [round(float(v), 3) for v in (*everything[:, :2].min(axis=0), *everything[:, :2].max(axis=0))],
            "markers": markers(lane), "wall_cells": len(cells),
        })
    (arenas_dir / "index.json").write_text(json.dumps(index, indent=1) + "\n")
    square = next(a for a in index["arenas"] if a["id"] == "square")
    walls = json.loads((arenas_dir / "square.walls.json").read_text())
    routes_dir = maps_dir / "routes"
    routes_dir.mkdir(parents=True, exist_ok=True)
    (routes_dir / "krail_square.json").write_text(json.dumps(route_file(
        "K-Rail Square course", square, SQUARE_ROUTE, walls), indent=1) + "\n")
    return index


def route_file(name: str, arena: dict, waypoints, walls: dict) -> dict:
    """A route file (VIP-Rescue's route/route_file.hpp reads it) on an arena, with the arena's walls in it."""
    return {
        "schema": ROUTE_SCHEMA, "name": name,
        "reference": {"kind": "arena", "id": arena["id"], "title": arena["title"]},
        "start": arena["start"],
        "waypoints": [[float(x), float(y)] for x, y in waypoints],
        "speed": 0.5,
        "walls": {"resolution": walls["resolution"], "band": walls["band"], "cells": walls["cells"]},
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    root = Path(os.environ.get("VIP_RESCUE_ROOT") or Path.home() / "VIP-Rescue").expanduser()
    parser.add_argument("--out", type=Path,
                        default=root / "Docker/Isaac/go2_ws/src/go2_control_cpp/maps",
                        help="the website's maps directory (default: VIP-Rescue's go2_control_cpp/maps)")
    args = parser.parse_args(argv)
    index = export(args.out)
    total = sum((args.out / a["model"]).stat().st_size for a in index["arenas"])
    print(f"[web_export] {len(index['arenas'])} arenas into {args.out / 'arenas'} ({total / 1e6:.1f} MB of models)")
    for arena in index["arenas"]:
        print(f"[web_export]   {arena['id']:22s} {arena['wall_cells']:5d} wall cells, start "
              f"({arena['start']['x']}, {arena['start']['y']}, {arena['start']['yaw_deg']} deg)")


if __name__ == "__main__":
    main()
