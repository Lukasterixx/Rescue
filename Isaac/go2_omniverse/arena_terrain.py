# arena_terrain.py
"""Isaac Lab glue for the obstacle arena described in `arena_layout.py`: the
sub-terrain config and generator function that hand the arena to Isaac Lab's
TerrainGenerator, so it becomes the walkable, height-scanned terrain exactly
like the maze did.
"""
from __future__ import annotations

from dataclasses import MISSING

import numpy as np
import trimesh
from isaaclab.utils import configclass

import arena_layout
from terrain_generator_cfg import SubTerrainBaseCfg


@configclass
class ArenaTerrainCfg(SubTerrainBaseCfg):
    """The 2.4 m obstacle arena, centred on a flat floor slab the size of the sub-terrain."""

    deck_thickness: float = arena_layout.DECK_THICKNESS
    obstacle_height: float = arena_layout.BEAM_HEIGHT
    rail_height: float = arena_layout.TASK_ELEVATION
    function = MISSING


def generate_arena_terrain(difficulty: float, cfg: ArenaTerrainCfg) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Build the arena in the middle of the sub-terrain and return that middle as the origin.

    The generator re-centres every sub-terrain's (0, 0)-(size) box on the world origin and
    shifts the terrain origin by the same amount, so returning the centre here puts both the
    arena AND the env origin at world (0, 0, 0). Everything in arena_layout, including the
    robot's start poses, is then a plain world coordinate. (The maze returns zeros instead,
    which is why its spawn is quoted in sub-terrain coordinates as (6.2, 6.2).)
    """
    cx, cy = cfg.size[0] / 2.0, cfg.size[1] / 2.0
    meshes = arena_layout.build_arena_meshes(
        floor_size=cfg.size,
        center=(cx, cy),
        deck_thickness=cfg.deck_thickness,
        obstacle_height=cfg.obstacle_height,
        rail_height=cfg.rail_height,
    )
    print(arena_layout.describe(cfg.deck_thickness, cfg.obstacle_height, cfg.rail_height))
    return [trimesh.util.concatenate(meshes)], np.array([cx, cy, 0.0])


ArenaTerrainCfg.function = generate_arena_terrain
