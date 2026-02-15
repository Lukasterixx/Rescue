# maze_terrain.py
from __future__ import annotations

import numpy as np
import trimesh
import random
from dataclasses import MISSING
from omni.isaac.orbit.utils import configclass
from terrain_generator_cfg import SubTerrainBaseCfg

@configclass
class MazeTerrainCfg(SubTerrainBaseCfg):
    """Configuration for a Maze terrain."""
    wall_height: float = 1.2
    wall_thickness: float = 0.1
    cell_width: float = 1.2
    step_height: float = 0.1
    step_depth: float = 0.1  # <--- NEW PARAMETER (Default 0.1)
    function = MISSING

def generate_maze_terrain(difficulty: float, cfg: MazeTerrainCfg) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """
    Generates a maze in positive coordinates (0,0) to (W,L).
    """
    # 1. SETUP GRID
    width_m, length_m = cfg.size
    rows = int(width_m / cfg.cell_width)
    cols = int(length_m / cfg.cell_width)
    
    if rows % 2 == 0: rows += 1
    if cols % 2 == 0: cols += 1

    open_cells = {}
    passages = set()
    frontier = set()

    # 2. MAZE ALGORITHM
    start_r, start_c = rows // 2, cols // 2
    start_z = 0.0
    
    # Safe Zone
    for r in range(start_r - 1, start_r + 2):
        for c in range(start_c - 1, start_c + 2):
            if 0 <= r < rows and 0 <= c < cols:
                open_cells[(r, c)] = start_z
                if r > start_r - 1: passages.add(tuple(sorted(((r, c), (r-1, c)))))
                if c > start_c - 1: passages.add(tuple(sorted(((r, c), (r, c-1)))))

    # Frontier
    def add_frontier(r, c):
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if (nr, nc) not in open_cells and (nr, nc) not in frontier:
                    frontier.add((nr, nc))

    for (r, c) in open_cells.keys():
        add_frontier(r, c)

    # Grow Maze
    while frontier:
        idx = random.randrange(len(frontier))
        cr, cc = list(frontier)[idx]
        frontier.remove((cr, cc))
        
        open_neighbors = []
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nr, nc = cr + dr, cc + dc
            if (nr, nc) in open_cells:
                open_neighbors.append((nr, nc))
        
        if open_neighbors:
            parent = random.choice(open_neighbors)
            parent_z = open_cells[parent]
            new_z = parent_z
            
            # Elevation Chance
            if random.random() < 0.15:
                steps_up = random.randint(2, 4) 
                elevation_gain = steps_up * cfg.step_height
                new_z = parent_z + elevation_gain
            
            open_cells[(cr, cc)] = new_z
            passages.add(tuple(sorted(((cr, cc), parent))))
            add_frontier(cr, cc)
            
            if random.random() < 0.05: # Loop closing
                for n in open_neighbors:
                    if n != parent and abs(open_cells[n] - new_z) < 0.01:
                         passages.add(tuple(sorted(((cr, cc), n))))

    # 3. GENERATE GEOMETRY
    meshes = []
    
    C_FLOOR  = [120, 120, 120, 255] 
    C_WALL   = [50, 50, 70, 255]
    C_STAIR  = [180, 80, 80, 255]
    C_PILLAR = [80, 80, 80, 255]

    def add_box(center, size, color):
        if size[0] <= 0.001 or size[1] <= 0.001: return 
        b = trimesh.creation.box(extents=size)
        b.apply_translation(center)
        b.visual.vertex_colors = color
        meshes.append(b)

    for r in range(rows):
        for c in range(cols):
            cx = r * cfg.cell_width
            cy = c * cfg.cell_width
            
            if (r, c) in open_cells:
                z = open_cells[(r, c)]
                
                # Default Floor Bounds (relative to center)
                bounds = {
                    'E': cfg.cell_width / 2, # +x
                    'W': cfg.cell_width / 2, # -x
                    'N': cfg.cell_width / 2, # +y
                    'S': cfg.cell_width / 2  # -y
                }

                # Check neighbors to shrink floor
                neighbors = {
                    'E': (r+1, c), 'W': (r-1, c),
                    'N': (r, c+1), 'S': (r, c-1)
                }

                for direction, (nr, nc) in neighbors.items():
                    if (nr, nc) in open_cells:
                        if tuple(sorted(((r, c), (nr, nc)))) in passages:
                            nz = open_cells[(nr, nc)]
                            diff = abs(z - nz)
                            
                            # If height diff exists, shrink floor to fit stairs
                            if diff > 0.05:
                                num_steps = int(round(diff / cfg.step_height))
                                total_run = num_steps * cfg.step_depth # <--- Uses custom depth
                                
                                # Center the staircase between the two tiles
                                new_bound = (cfg.cell_width / 2.0) - (total_run / 2.0)
                                if new_bound < 0: new_bound = 0.0
                                
                                bounds[direction] = new_bound

                # --- DRAW FLOOR ---
                size_x = bounds['W'] + bounds['E']
                size_y = bounds['S'] + bounds['N']
                center_offset_x = (bounds['E'] - bounds['W']) / 2.0
                center_offset_y = (bounds['N'] - bounds['S']) / 2.0
                
                floor_thick = 0.1
                add_box(
                    [cx + center_offset_x, cy + center_offset_y, z - (floor_thick/2)], 
                    [size_x, size_y, floor_thick], 
                    C_FLOOR
                )
                
                # Pillar
                if z > 0.05:
                    pillar_h = z - 0.1
                    if pillar_h > 0:
                        p_z = (z - 0.1) - (pillar_h / 2)
                        add_box([cx, cy, p_z], [cfg.cell_width*0.5, cfg.cell_width*0.5, pillar_h], C_PILLAR)

                # --- DRAW WALLS & STAIRS ---
                for direction, (nr, nc) in neighbors.items():
                    has_neighbor = (nr, nc) in open_cells
                    is_connected = has_neighbor and tuple(sorted(((r, c), (nr, nc)))) in passages
                    
                    if not is_connected:
                        # Walls
                        offset = cfg.cell_width / 2
                        wx, wy = cx, cy
                        w_dim = [0,0,0]
                        
                        if direction == 'E': wx += offset; w_dim = [cfg.wall_thickness, cfg.cell_width, cfg.wall_height]
                        elif direction == 'W': wx -= offset; w_dim = [cfg.wall_thickness, cfg.cell_width, cfg.wall_height]
                        elif direction == 'N': wy += offset; w_dim = [cfg.cell_width, cfg.wall_thickness, cfg.wall_height]
                        elif direction == 'S': wy -= offset; w_dim = [cfg.cell_width, cfg.wall_thickness, cfg.wall_height]
                        
                        wz = z + (cfg.wall_height / 2)
                        add_box([wx, wy, wz], w_dim, C_WALL)

                    elif is_connected:
                        neighbor_z = open_cells[(nr, nc)]
                        diff = z - neighbor_z
                        
                        # Only generate stairs if WE are the higher cell
                        if diff > 0.05:
                            num_steps = int(round(diff / cfg.step_height))
                            
                            start_dist = bounds[direction] 
                            
                            for s in range(num_steps):
                                # 1. Vertical Logic
                                step_top = z - ((s + 1) * cfg.step_height)
                                
                                # Solid base
                                solid_h = step_top - neighbor_z
                                if solid_h < 0.01: solid_h = 0.01
                                step_block_z = neighbor_z + (solid_h / 2.0)

                                # 2. Horizontal Logic (using custom step_depth)
                                current_dist = start_dist + (s * cfg.step_depth) + (cfg.step_depth / 2.0)
                                
                                sx, sy = cx, cy
                                s_dim = [0, 0, 0]
                                
                                if direction == 'E': 
                                    sx += current_dist
                                    s_dim = [cfg.step_depth, cfg.cell_width, solid_h]
                                elif direction == 'W': 
                                    sx -= current_dist
                                    s_dim = [cfg.step_depth, cfg.cell_width, solid_h]
                                elif direction == 'N': 
                                    sy += current_dist
                                    s_dim = [cfg.cell_width, cfg.step_depth, solid_h]
                                elif direction == 'S': 
                                    sy -= current_dist
                                    s_dim = [cfg.cell_width, cfg.step_depth, solid_h]
                                
                                add_box([sx, sy, step_block_z], s_dim, C_STAIR)

    if not meshes: return [], np.zeros(3)

    combined_mesh = trimesh.util.concatenate(meshes)
    return [combined_mesh], np.zeros(3)

MazeTerrainCfg.function = generate_maze_terrain