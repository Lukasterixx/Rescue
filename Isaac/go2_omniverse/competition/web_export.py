"""The competition's arenas for VIP-Rescue's website: a 3D model of each, in its own frame, and its terrain, which
the Routes tab draws routes over and the robot aligns them to.

    python3 -m competition.web_export                       # into $VIP_RESCUE_ROOT (default ~/VIP-Rescue)
    python3 -m competition.web_export --out /some/maps/dir  # or anywhere

It writes, under the website's `maps/` directory:
  arenas/index.json          every arena: title, model, terrain, start pose, START/END markers, bounds
  arenas/<key>.glb           the arena's meshes (glTF 2.0, one primitive per colour), in the arena's frame
  arenas/<key>.terrain.json  the arena's terrain (below), on a 5 cm grid in the arena's frame
  routes/krail_square.json   the K-Rail Square's course route (training/measure_square.py's), as a route file

THE ARENA'S FRAME is the lane's origin with the world's axes (`rescue_sim.levels.Level.origin`): for the K-Rail
Square its centre, +x east, +y north. The rescue sim's `/sim/teleport` takes poses in it, and a route drawn on the
model is in it. glTF is +y up, so the models are stored turned (x, z, -y); a viewer turns them back with a +90 deg
rotation about x.

THE TERRAIN ("vip-rescue.terrain/1") is, for every 5 cm cell:
  levels  the heights the robot can stand at there: the top of a walkable surface (a `scan` mesh: floors, decks,
          K-rails, treads, the stairs' landing, pallet tops, pads; or the hall floor) with HEADROOM_M clear above it,
          so the floor under a deck is not one and the floor under the stairs' landing is. A cell keeps a level only
          if each of its four 2.5 cm columns can stand within MAX_STEP_M of it: a 5 cm railing member blocks a cell,
          a deck's edge does not.
  solids  the heights something stands at there, from every collision mesh, cut into 2.5 cm vertical columns
          (a closed mesh's crossings pair up into its insides) and merged over the cell's columns.
From these the website follows a route's height over the arena (a robot base BASE_HEIGHT_M above where it stands),
finds what it cannot walk onto, and finds the walls: in the band /map2d is drawn from, WALL_BAND_M above the surface
the robot stands on (flatten_node keeps points from 0.3 m above the base). A route aligns to what that band holds as
seen from the route's start, floors included (`alignment_walls`): a landing above the band is occupied in /map2d
seen from the floor, so it is in the walls the robot aligns to, though not one the route is checked against.

The maze's tarp is left out of the model, so the maze can be seen from above, but not out of the terrain. Left out of
both: the gravel lane's 7,800 loose stones (its gravel surface stays), textures (each material is its colour).
No Isaac imports: the system python3 runs it.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
from pathlib import Path
import struct

import numpy as np

from .geometry import COLORS, Options, build_lanes

SCHEMA = "vip-rescue.arenas/2"
TERRAIN_SCHEMA = "vip-rescue.terrain/1"
ROUTE_SCHEMA = "vip-rescue.route/1"
CELL_M = 0.05                        # the terrain's grid
COLUMN_M = 0.025                     # the columns the meshes are cut into: two to a cell each way
BASE_HEIGHT_M = 0.33                 # base_link over the surface the Go2 stands on (0.332 m in the rescue sim)
MAX_STEP_M = 0.25                    # the highest step a route may climb or drop in one go
HEADROOM_M = 0.5                     # clear height a level needs over it (crouching under the stairs' landing fits)
WALL_BAND_M = (0.65, 3.0)            # /map2d's band, over the surface the robot stands on
MARGIN_M = 0.5                       # hall floor kept round the arena
SPAWN_DROP_M = 0.42                  # the spawn's height above the floor it stands on
HIDDEN_MATERIALS = {"tarp"}          # in the terrain, not in the model
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


# ------------------------------------------------------------------------------------------ the terrain
def _columns(lane, origin, x0, y0, nx, ny):
    """Every collision mesh cut into vertical columns COLUMN_M apart: {(i, j): [(z0, z1, scan), ...]}, the solids
    along the column at (x0 + (i + 0.5) COLUMN_M, y0 + (j + 0.5) COLUMN_M)."""
    columns: dict[tuple[int, int], list] = {}
    for mesh in lane.meshes:
        if not mesh.collision:
            continue
        crossings: dict[tuple[int, int], list[float]] = {}
        for a, b, c in mesh.vertices[mesh.faces] - origin:
            det = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(det) < 1e-12:
                continue                                  # a vertical face: the faces above and below bound the solid
            xs, ys = (a[0], b[0], c[0]), (a[1], b[1], c[1])
            i0 = max(0, math.ceil((min(xs) - x0) / COLUMN_M - 0.5))
            i1 = min(nx - 1, math.floor((max(xs) - x0) / COLUMN_M - 0.5))
            j0 = max(0, math.ceil((min(ys) - y0) / COLUMN_M - 0.5))
            j1 = min(ny - 1, math.floor((max(ys) - y0) / COLUMN_M - 0.5))
            if i1 < i0 or j1 < j0:
                continue
            I, J = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), indexing="ij")
            X, Y = x0 + (I + 0.5) * COLUMN_M, y0 + (J + 0.5) * COLUMN_M
            l0 = ((b[1] - c[1]) * (X - c[0]) + (c[0] - b[0]) * (Y - c[1])) / det
            l1 = ((c[1] - a[1]) * (X - c[0]) + (a[0] - c[0]) * (Y - c[1])) / det
            l2 = 1.0 - l0 - l1
            inside = (l0 >= -1e-9) & (l1 >= -1e-9) & (l2 >= -1e-9)
            Z = l0 * a[2] + l1 * b[2] + l2 * c[2]
            for i, j, z in zip(I[inside].tolist(), J[inside].tolist(), Z[inside].tolist()):
                crossings.setdefault((i, j), []).append(z)
        for key, zs in crossings.items():
            zs = sorted(zs)
            unique = [z for k, z in enumerate(zs) if k == 0 or z - zs[k - 1] > 1e-6]   # an edge two faces share
            if len(unique) % 2:
                pairs = [(z, z) for z in unique]          # not closed: each crossing a sheet
            else:
                pairs = list(zip(unique[0::2], unique[1::2]))
            columns.setdefault(key, []).extend((z0, z1, mesh.scan) for z0, z1 in pairs)
    return columns


def _levels(solids) -> list[float]:
    """Where a column can be stood on: the hall floor and the walkable tops, each with HEADROOM_M clear over it."""
    tops = [0.0] + [z1 for _, z1, scan in solids if scan]
    out = []
    for top in sorted(set(round(t, 3) for t in tops)):
        if top < 0.0:
            continue
        clear = all(z1 <= top + 0.02 or z0 >= top + HEADROOM_M for z0, z1, _ in solids)
        if clear:
            out.append(top)
    return out


def _merge(intervals, gap: float = 0.02) -> list[tuple[float, float]]:
    out: list[list[float]] = []
    for z0, z1 in sorted(intervals):
        if out and z0 <= out[-1][1] + gap:
            out[-1][1] = max(out[-1][1], z1)
        else:
            out.append([z0, z1])
    return [(a, b) for a, b in out]


def terrain(lane) -> dict:
    """The lane's terrain (module docstring): its grid and, per cell, its levels and solids."""
    origin = np.asarray(lane.origin, dtype=float)
    xy = np.concatenate([m.vertices[:, :2] for m in lane.meshes]) - origin[:2]
    x0 = math.floor((xy[:, 0].min() - MARGIN_M) / CELL_M) * CELL_M
    y0 = math.floor((xy[:, 1].min() - MARGIN_M) / CELL_M) * CELL_M
    width = math.ceil((xy[:, 0].max() + MARGIN_M - x0) / CELL_M)
    height = math.ceil((xy[:, 1].max() + MARGIN_M - y0) / CELL_M)
    sub = int(round(CELL_M / COLUMN_M))
    columns = _columns(lane, origin, x0, y0, width * sub, height * sub)
    levels, solids = [], []
    for j in range(height):
        for i in range(width):
            cols = [columns.get((i * sub + di, j * sub + dj), []) for di in range(sub) for dj in range(sub)]
            col_levels = [_levels(c) for c in cols]
            cell = []
            for level in sorted({z for lv in col_levels for z in lv}, reverse=True):
                if all(any(abs(z - level) <= MAX_STEP_M for z in lv) for lv in col_levels) and \
                        not any(abs(level - kept) < 0.03 for kept in cell):
                    cell.append(level)
            levels.append(sorted(cell))
            solids.append(_merge([(z0, z1) for c in cols for z0, z1, _ in c]))
    return {"x0": x0, "y0": y0, "width": width, "height": height, "levels": levels, "solids": solids}


def _pack(lists, per_item: int) -> dict:
    """Variable-length lists of heights as two base64 arrays: a count per cell (uint8) and the heights in cm (int16)."""
    counts = np.array([len(item) // per_item if per_item > 1 else len(item) for item in lists], dtype="u1")
    flat = [v for item in lists for v in item]
    z = np.round(np.asarray(flat, dtype=float) * 100).astype("<i2")
    return {"counts": base64.b64encode(counts.tobytes()).decode(), "z_cm": base64.b64encode(z.tobytes()).decode()}


def terrain_json(grid: dict, frame: str) -> dict:
    return {
        "schema": TERRAIN_SCHEMA, "frame": frame, "resolution": CELL_M,
        "x0": round(grid["x0"], 4), "y0": round(grid["y0"], 4), "width": grid["width"], "height": grid["height"],
        "base_height": BASE_HEIGHT_M, "max_step": MAX_STEP_M, "headroom": HEADROOM_M, "wall_band": list(WALL_BAND_M),
        "outside_level": 0.0,
        "levels": _pack(grid["levels"], 1),
        "solids": _pack([[z for pair in cell for z in pair] for cell in grid["solids"]], 2),
    }


def _cell(grid: dict, x: float, y: float):
    i, j = math.floor((x - grid["x0"]) / CELL_M), math.floor((y - grid["y0"]) / CELL_M)
    if 0 <= i < grid["width"] and 0 <= j < grid["height"]:
        return j * grid["width"] + i
    return None


def alignment_walls(grid: dict, start: dict) -> list[list[float]]:
    """What /map2d shows from the route's start: the cells with a solid in WALL_BAND_M over the surface the robot
    starts on (its highest level there), floors included."""
    index = _cell(grid, start["x"], start["y"])
    here = grid["levels"][index] if index is not None and grid["levels"][index] else [0.0]
    low, high = here[-1] + WALL_BAND_M[0], here[-1] + WALL_BAND_M[1]
    out = []
    for k, cell in enumerate(grid["solids"]):
        if any(z0 <= high and z1 >= low for z0, z1 in cell):
            j, i = divmod(k, grid["width"])
            out.append([round(grid["x0"] + (i + 0.5) * CELL_M, 3), round(grid["y0"] + (j + 0.5) * CELL_M, 3)])
    return out


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
             "resolution": CELL_M, "arenas": []}
    grids = {}
    for lane in lanes:
        groups = model_groups(lane)
        model = glb(groups)
        (arenas_dir / f"{lane.key}.glb").write_bytes(model)
        grid = grids[lane.key] = terrain(lane)
        (arenas_dir / f"{lane.key}.terrain.json").write_text(
            json.dumps(terrain_json(grid, f"arena {lane.key}"), separators=(",", ":")) + "\n")
        everything = np.concatenate([v for v, _ in groups.values()])
        origin = np.asarray(lane.origin, dtype=float)
        floor = lane.spawn[2] - SPAWN_DROP_M - origin[2]
        index["arenas"].append({
            "id": lane.key, "title": lane.title, "group": lane.arena, "setting": lane.setting,
            "subtitle": getattr(lane, "subtitle", ""),
            "model": f"arenas/{lane.key}.glb", "terrain": f"arenas/{lane.key}.terrain.json",
            "version": hashlib.sha1(model).hexdigest()[:10],
            "floor_z": round(floor, 3),
            "start": {"x": round(lane.spawn[0] - origin[0], 3), "y": round(lane.spawn[1] - origin[1], 3),
                      "yaw_deg": round(yaw_deg(lane.spawn_rotation), 2)},
            "bounds": [round(float(v), 3) for v in (*everything[:, :2].min(axis=0), *everything[:, :2].max(axis=0))],
            "markers": markers(lane),
            "walkable_cells": sum(bool(levels) for levels in grid["levels"]),
        })
    (arenas_dir / "index.json").write_text(json.dumps(index, indent=1) + "\n")
    square = next(a for a in index["arenas"] if a["id"] == "square")
    routes_dir = maps_dir / "routes"
    routes_dir.mkdir(parents=True, exist_ok=True)
    (routes_dir / "krail_square.json").write_text(json.dumps(route_file(
        "K-Rail Square course", square, SQUARE_ROUTE, grids["square"]), indent=1) + "\n")
    return index


def route_file(name: str, arena: dict, waypoints, grid: dict) -> dict:
    """A route file (VIP-Rescue's route/route_file.hpp reads it) on an arena, with the walls the robot aligns to from
    its start (`alignment_walls`). The Routes tab writes the same."""
    return {
        "schema": ROUTE_SCHEMA, "name": name,
        "reference": {"kind": "arena", "id": arena["id"], "title": arena["title"]},
        "start": arena["start"],
        "waypoints": [[float(x), float(y)] for x, y in waypoints],
        "speed": 0.5,
        "walls": {"resolution": CELL_M, "seen_from": "start", "cells": alignment_walls(grid, arena["start"])},
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
        size = (args.out / arena["terrain"]).stat().st_size
        print(f"[web_export]   {arena['id']:22s} {arena['walkable_cells']:6d} walkable cells ({size / 1e3:.0f} kB), "
              f"start ({arena['start']['x']}, {arena['start']['y']}, {arena['start']['yaw_deg']} deg)")


if __name__ == "__main__":
    main()
