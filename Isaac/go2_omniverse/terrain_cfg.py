# Copyright (c) 2024, RoboVerse community
# ... (License header) ...

from terrain_generator_cfg import TerrainGeneratorCfg
import isaaclab.terrains as terrain_gen

# --- IMPORT CONFIG AND FUNCTION ---
from maze_terrain import MazeTerrainCfg, generate_maze_terrain
from arena_terrain import ArenaTerrainCfg, generate_arena_terrain

# THE OBSTACLE ARENA (2026-09-18), the default world: the 2.4 m x 2.4 m course from the design
# drawing, every dimension in arena_layout.py. The sub-terrain is a flat floor slab with the
# arena in its middle, so `size` is how much room the robot has around the structure.
# custom_rl_env.py picks this unless the sim is launched with `--custom_env maze`.
ARENA_TERRAIN_CFG = TerrainGeneratorCfg(
    size=(10.0, 10.0),
    border_width=0.0,
    num_rows=1,
    num_cols=1,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "arena": ArenaTerrainCfg(
            function=generate_arena_terrain,
            proportion=1.0,
            size=(10.0, 10.0),  # overwritten by the generator's size above
        ),
    },
)

# THE MAZE, the previous default, kept behind `--custom_env maze`.
ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
    size=(12.0, 12.0),
    border_width=0.0,
    num_rows=1,
    num_cols=1,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "my_maze": MazeTerrainCfg(
            function=generate_maze_terrain, 
            proportion=1.0,
            wall_height=1.0,
            wall_thickness=0.05,  # Thinner walls
            cell_width=1.2,      # Slightly wider halls
            step_height=0.1,
            step_depth=0.2,
            # FLAT FLOOR, NO STAIRS (2026-09-14). No policy trained for this robot climbs the
            # maze's 10 cm x 20 cm stairs reliably (training/README.md, "The seeded benchmark"),
            # so the maze is built on one level and the walking policy is trained on flat
            # ground. Set back to 0.15 to bring the stairs back.
            elevation_chance=0.0,
            size=(15.0, 15.0)    # 15x15m maze
        ),
    },
)
