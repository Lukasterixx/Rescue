# Copyright (c) 2024, RoboVerse community
# ... (License header) ...

from terrain_generator_cfg import TerrainGeneratorCfg
import isaaclab.terrains as terrain_gen

# --- IMPORT CONFIG AND FUNCTION ---
from maze_terrain import MazeTerrainCfg, generate_maze_terrain

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
