"""Training ground for the K-Rail Square: raised decks to step onto and off, with K-rails on them and between them.

THE TARGET. The competition's K-Rail Square (`../../competition/geometry.py`, `square_lane`) is a 2.4 m framed deck,
12.2 cm above the hall floor (10 cm framing, 11 mm OSB, 11 mm backing), with four 10 cm K-rails (10 cm wide, 45 deg
mitred ends) making an X across it and a crate on the crossing. The robot starts on the hall floor outside, steps up
through an open half-edge, crosses two rails to reach the far triangle, and steps down off the far edge. The
Diagonal K-Rails lanes add the same rails on 1.2 m panels, 20 cm tall in the Additional Obstacles copy.

WHAT A TILE IS. `krail_deck_terrain` scatters decks (rectangles 1.4-3.0 m a side) around a clear spawn square, most
with an X of rails corner to corner as in the square, the rest with one or two rails at random angles, and puts
more rails on the ground between them. Whichever way a robot walks out of the spawn square it meets a step up, a
rail or both within a couple of metres, and a step down on the far side of each deck. `difficulty` sets the deck
height and the rail height together:

    level (of 10)      0      2      4      6      8      9
    deck height, m   0.03   0.07   0.12   0.16   0.20   0.22     (square: 0.122 -> level ~4)
    rail height, m   0.03   0.07   0.11   0.15   0.20   0.23     (square: 0.10 -> level ~3.5; layered: 0.20 -> ~8)

Each deck and rail also draws its own height within +-15 % of those, so a tile never teaches one exact step.

The rails are 10 cm boxes, not the guide's mitred prisms: the mitres only shape the ends, and a rail's end is where
a robot steps round it rather than over it. Collision and the height scan both see every mesh here.

No Isaac Sim imports (trimesh and numpy only), so the generator can be run and checked outside Kit.
"""
from __future__ import annotations

import math
from dataclasses import MISSING

import numpy as np
import trimesh

from isaaclab.terrains import SubTerrainBaseCfg
from isaaclab.utils import configclass

RAIL_WIDTH_M = 0.10  # competition/geometry.py: `diagonal(..., width=0.1, ...)`


def _lerp(bounds, t: float) -> float:
    return bounds[0] + t * (bounds[1] - bounds[0])


def _box(center, size, yaw: float = 0.0) -> trimesh.Trimesh:
    transform = trimesh.transformations.rotation_matrix(yaw, (0.0, 0.0, 1.0))
    transform[:3, 3] = center
    return trimesh.creation.box(extents=size, transform=transform)


def _corners(cx, cy, length, width, yaw) -> np.ndarray:
    """The four plan-view corners of a length x width rectangle centred at (cx, cy), rotated by yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    half = np.array([[length, width], [length, -width], [-length, -width], [-length, width]]) / 2
    return np.stack([cx + half[:, 0] * c - half[:, 1] * s, cy + half[:, 0] * s + half[:, 1] * c], axis=1)


def _separated(a: np.ndarray, b: np.ndarray, gap: float) -> bool:
    """True when two convex plan polygons (corners in order) are at least `gap` apart along some separating axis."""
    for poly in (a, b):
        edges = np.roll(poly, -1, axis=0) - poly
        for edge in edges:
            axis = np.array([-edge[1], edge[0]])
            axis /= np.linalg.norm(axis)
            pa, pb = a @ axis, b @ axis
            if pa.max() + gap <= pb.min() or pb.max() + gap <= pa.min():
                return True
    return False


def _rect(cx, cy, sx, sy) -> np.ndarray:
    return _corners(cx, cy, sx, sy, 0.0)


def krail_deck_terrain(difficulty: float, cfg: "KRailDeckTerrainCfg") -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Decks with K-rails on them and K-rails between them, around a clear spawn square. See the module docstring."""
    size_x, size_y = cfg.size
    centre = np.array([size_x / 2, size_y / 2])
    deck_h = _lerp(cfg.deck_height_range, difficulty)
    rail_h = _lerp(cfg.rail_height_range, difficulty)
    jitter = lambda: float(np.random.uniform(1.0 - cfg.height_jitter, 1.0 + cfg.height_jitter))  # noqa: E731

    meshes = [_box((size_x / 2, size_y / 2, -0.5), (size_x, size_y, 1.0))]
    spawn = _rect(centre[0], centre[1], 2 * cfg.spawn_clear_m, 2 * cfg.spawn_clear_m)
    margin = cfg.edge_margin_m

    # --- decks ---------------------------------------------------------------------------------------------------
    decks = []  # (cx, cy, sx, sy, height)
    for _ in range(cfg.deck_attempts):
        if len(decks) >= cfg.num_decks:
            break
        sx, sy = np.random.uniform(*cfg.deck_size_range, size=2)
        cx = np.random.uniform(margin + sx / 2, size_x - margin - sx / 2)
        cy = np.random.uniform(margin + sy / 2, size_y - margin - sy / 2)
        poly = _rect(cx, cy, sx, sy)
        if not _separated(poly, spawn, 0.0):
            continue
        if not all(_separated(poly, _rect(d[0], d[1], d[2], d[3]), cfg.deck_gap_m) for d in decks):
            continue
        decks.append((cx, cy, sx, sy, deck_h * jitter()))
    for cx, cy, sx, sy, h in decks:
        meshes.append(_box((cx, cy, h / 2), (sx, sy, h)))
        # Rails on the deck: an X corner to corner as in the square, or one or two at random angles.
        if np.random.uniform() < cfg.deck_x_probability:
            diag = math.hypot(sx, sy) - cfg.rail_end_inset_m
            for yaw in (math.atan2(sy, sx), -math.atan2(sy, sx)):
                rh = rail_h * jitter()
                meshes.append(_box((cx, cy, h + rh / 2), (diag, RAIL_WIDTH_M, rh), yaw))
        else:
            placed = 0
            for _ in range(40):
                if placed >= np.random.randint(1, 3):
                    break
                yaw = np.random.uniform(0.0, math.pi)
                length = np.random.uniform(*cfg.rail_length_range)
                rx = np.random.uniform(cx - sx / 2, cx + sx / 2)
                ry = np.random.uniform(cy - sy / 2, cy + sy / 2)
                corners = _corners(rx, ry, length, RAIL_WIDTH_M, yaw)
                inside = (np.abs(corners[:, 0] - cx) <= sx / 2 - 0.05).all() and \
                         (np.abs(corners[:, 1] - cy) <= sy / 2 - 0.05).all()
                if not inside:
                    continue
                rh = rail_h * jitter()
                meshes.append(_box((rx, ry, h + rh / 2), (length, RAIL_WIDTH_M, rh), yaw))
                placed += 1

    # --- rails on the ground ----------------------------------------------------------------------------------------
    ground = []
    for _ in range(cfg.rail_attempts):
        if len(ground) >= cfg.num_ground_rails:
            break
        yaw = np.random.uniform(0.0, math.pi)
        length = np.random.uniform(*cfg.rail_length_range)
        rx = np.random.uniform(margin, size_x - margin)
        ry = np.random.uniform(margin, size_y - margin)
        poly = _corners(rx, ry, length, RAIL_WIDTH_M, yaw)
        if (poly < margin).any() or (poly[:, 0] > size_x - margin).any() or (poly[:, 1] > size_y - margin).any():
            continue
        if not _separated(poly, spawn, 0.0):
            continue
        if not all(_separated(poly, _rect(d[0], d[1], d[2], d[3]), cfg.rail_deck_gap_m) for d in decks):
            continue
        if not all(_separated(poly, other, cfg.rail_gap_m) for other in ground):
            continue
        ground.append(poly)
        rh = rail_h * jitter()
        meshes.append(_box((rx, ry, rh / 2), (length, RAIL_WIDTH_M, rh), yaw))

    origin = np.array([centre[0], centre[1], 0.0])
    return meshes, origin


@configclass
class KRailDeckTerrainCfg(SubTerrainBaseCfg):
    """`krail_deck_terrain`'s settings. With `num_decks=0` it is a field of rails on the flat."""

    function = krail_deck_terrain

    deck_height_range: tuple[float, float] = (0.03, 0.24)
    """Deck height at difficulty 0 and 1, m. The square's deck is 0.122."""
    rail_height_range: tuple[float, float] = (0.03, 0.25)
    """K-rail height at difficulty 0 and 1, m. The square's are 0.10; the Additional Obstacles lane's 0.20."""
    height_jitter: float = 0.15
    """Each deck and rail draws its height within this fraction of the level's."""
    num_decks: int = 4
    deck_attempts: int = 300
    deck_size_range: tuple[float, float] = (1.4, 3.0)
    deck_gap_m: float = 0.6
    """Ground between two decks, so each has a step down and a step up rather than merging into one."""
    deck_x_probability: float = 0.6
    rail_end_inset_m: float = 0.15
    num_ground_rails: int = 5
    rail_attempts: int = 300
    rail_length_range: tuple[float, float] = (0.9, 1.8)
    rail_gap_m: float = 0.35
    rail_deck_gap_m: float = 0.35
    spawn_clear_m: float = 1.0
    """Half-width of the clear square the robot spawns in, at the tile's centre."""
    edge_margin_m: float = 0.15
