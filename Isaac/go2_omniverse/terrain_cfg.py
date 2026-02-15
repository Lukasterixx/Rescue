# Copyright (c) 2024, RoboVerse community
# ... (License header) ...

from terrain_generator_cfg import TerrainGeneratorCfg
import omni.isaac.orbit.terrains as terrain_gen

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
            size=(15.0, 15.0)    # 15x15m maze
        ),
    },
)
