import random
import math
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf, UsdShade

class MazeManager:
    def __init__(self, stage, start_pos=(0, 0, 0), grid_radius=15):
        self.stage = stage
        self.grid_radius = grid_radius
        self.tile_size = 1.0   
        self.wall_height = 1.0 
        self.wall_thickness = 0.05 
        
        # --- Maze Data ---
        # Maps (x, y) -> z_height (float)
        self.open_cells = {} 
        self.passages = set() 
        self.frontier = set()

        # 1. Create a Safe Zone (3x3 Room at spawn)
        start_z = start_pos[2]
        # Snap start Z to nearest 0.1 to align with stairs
        start_z = round(start_z * 10) / 10.0
        
        for x in range(-1, 2):
            for y in range(-1, 2):
                self.open_cells[(x, y)] = start_z
                # Connect internal neighbors
                if x < 1: self._add_passage((x, y), (x + 1, y))
                if y < 1: self._add_passage((x, y), (x, y + 1))

        # 2. Initialize Frontier
        for cell in list(self.open_cells.keys()):
            self._add_neighbors_to_frontier(cell[0], cell[1])

        # --- Object Pooling ---
        # Must match the path used in custom_rl_env.py -> spawn_maze_pools
        self.root_path = "/World/warehouse/Maze"
        
        # Pool Sizes (Must match custom_rl_env.py)
        self.wall_pool_size = 800
        self.floor_pool_size = 800
        self.stair_pool_size = 200

        # Pools and Active Maps
        self.wall_pool = []
        self.active_walls_map = {} 
        
        self.floor_pool = []
        self.active_floors_map = {}

        self.stair_pool = []
        self.active_stairs_map = {}
        
        self._init_usd_pool()

    def _init_usd_pool(self):
        """
        Locates the prims already spawned by custom_rl_env.py and 
        ensures they have the necessary XformOps for manipulation.
        """
        print(f"[Maze] Linking to pre-spawned pools at {self.root_path}...")
        
        # Ensure root exists (it should, from env creation)
        if not self.stage.GetPrimAtPath(self.root_path):
            print(f"[Maze][WARN] Root path {self.root_path} not found! Did spawn_maze_pools run?")

        # --- 1. Wall Pool ---
        for i in range(self.wall_pool_size):
            path = f"{self.root_path}/Wall_{i}"
            if self._ensure_prim_ops(path):
                self.wall_pool.append(path)
                self._reset_prim(path)

        # --- 2. Floor Pool ---
        for i in range(self.floor_pool_size):
            path = f"{self.root_path}/Floor_{i}"
            if self._ensure_prim_ops(path):
                self.floor_pool.append(path)
                self._reset_prim(path)

        # --- 3. Stair Pool ---
        for i in range(self.stair_pool_size):
            path = f"{self.root_path}/Stair_{i}"
            if self._ensure_prim_ops(path):
                self.stair_pool.append(path)
                self._reset_prim(path)

        print(f"[Maze] Pool Init Complete. Walls: {len(self.wall_pool)}, Floors: {len(self.floor_pool)}, Stairs: {len(self.stair_pool)}")

    def _ensure_prim_ops(self, prim_path):
        """
        Checks if prim exists. If so, ensures it has Translate, RotateXYZ, and Scale ops.
        custom_rl_env.py only adds Translate. We add the others here so placement logic works.
        """
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return False
            
        xform = UsdGeom.Xformable(prim)
        
        # We expect 3 ops: Translate, Rotate, Scale.
        # custom_rl_env adds Translate. We need to append the others if missing.
        
        # Check if we already added them (idempotency)
        has_rotate = False
        has_scale = False
        
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ: has_rotate = True
            if op.GetOpType() == UsdGeom.XformOp.TypeScale: has_scale = True
            
        if not has_rotate:
            xform.AddRotateXYZOp()
        if not has_scale:
            xform.AddScaleOp()
            
        return True

    def _reset_prim(self, prim_path):
        """Moves an object to holding area."""
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid(): return
        UsdGeom.Imageable(prim).MakeInvisible()
        
        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        
        # We can safely assume these exist now due to _ensure_prim_ops
        if len(ops) >= 3:
            ops[0].Set(Gf.Vec3d(0, 0, -100.0)) # Translate
            ops[1].Set(Gf.Vec3d(0, 0, 0))      # Rotate
            ops[2].Set(Gf.Vec3d(1.0, 1.0, 1.0))# Scale

    def _add_neighbors_to_frontier(self, x, y):
        for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]:
            nx, ny = x + dx, y + dy
            if (nx, ny) not in self.open_cells and (nx, ny) not in self.frontier:
                self.frontier.add((nx, ny))

    def _add_passage(self, cell_a, cell_b):
        connection = tuple(sorted((cell_a, cell_b)))
        self.passages.add(connection)

    def _has_passage(self, cell_a, cell_b):
        return tuple(sorted((cell_a, cell_b))) in self.passages

    def _grow_maze(self, target_x, target_y, force_complete=False):
        steps = 0
        max_steps = 999999 if force_complete else 30 
        
        while steps < max_steps:
            local_frontier = [
                (fx, fy) for fx, fy in self.frontier 
                if abs(fx - target_x) <= self.grid_radius + 2 
                and abs(fy - target_y) <= self.grid_radius + 2
            ]

            if not local_frontier:
                break 

            idx = random.randrange(len(local_frontier))
            cx, cy = local_frontier[idx]
            self.frontier.remove((cx, cy))
            
            open_neighbors = []
            for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) in self.open_cells:
                    open_neighbors.append((nx, ny))

            if open_neighbors:
                parent = random.choice(open_neighbors)
                parent_z = self.open_cells[parent]
                new_z = parent_z
                
                # --- ELEVATION LOGIC (10% Chance) ---
                if random.random() < 0.10:
                    steps_up = random.randint(2, 5) # 0.2m to 0.5m
                    elevation_gain = steps_up * 0.1
                    new_z = parent_z + elevation_gain
                # ------------------------------------

                self.open_cells[(cx, cy)] = new_z
                self._add_passage((cx, cy), parent)
                
                self._add_neighbors_to_frontier(cx, cy)
                
                # Small chance to close loops (same height only)
                if random.random() < 0.05: 
                    for n in open_neighbors:
                         if n != parent and abs(self.open_cells[n] - new_z) < 0.01:
                             if random.random() < 0.5: self._add_passage((cx, cy), n)

            steps += 1

    def update(self, robot_pos_3d, force_complete=False):
        rx = int(round(robot_pos_3d[0] / self.tile_size))
        ry = int(round(robot_pos_3d[1] / self.tile_size))

        self._grow_maze(rx, ry, force_complete=force_complete)
        
        needed_walls = set()
        needed_floors = set()
        needed_stairs = set()

        r = self.grid_radius
        for x in range(rx - r, rx + r + 1):
            for y in range(ry - r, ry + r + 1):
                if (x, y) in self.open_cells:
                    current_z = self.open_cells[(x, y)]
                    
                    # A. Floor
                    needed_floors.add((x, y))

                    # B. Neighbors (Walls & Stairs)
                    for dx, dy, direction in [(1,0,'E'), (-1,0,'W'), (0,1,'N'), (0,-1,'S')]:
                        nx, ny = x + dx, y + dy
                        
                        # 1. Wall Logic: If neighbor is blocked or no passage exists
                        if (nx, ny) not in self.open_cells or not self._has_passage((x, y), (nx, ny)):
                            needed_walls.add((x, y, direction))
                        
                        # 2. Stair Logic: Connect to connected neighbors
                        # We only trigger stair generation from the UPPER level looking down.
                        elif (nx, ny) in self.open_cells and self._has_passage((x, y), (nx, ny)):
                            neighbor_z = self.open_cells[(nx, ny)]
                            diff = current_z - neighbor_z
                            
                            # If I am higher than neighbor by at least 1 step
                            if diff > 0.05:
                                needed_stairs.add((x, y, direction, round(diff, 1)))

        self._gc_pool(self.active_walls_map, needed_walls, self.wall_pool)
        self._gc_pool(self.active_floors_map, needed_floors, self.floor_pool)
        self._gc_pool(self.active_stairs_map, needed_stairs, self.stair_pool)

        self._allocate_pool(self.active_walls_map, needed_walls, self.wall_pool, self._place_wall_instance)
        self._allocate_pool(self.active_floors_map, needed_floors, self.floor_pool, self._place_floor_instance)
        self._allocate_pool(self.active_stairs_map, needed_stairs, self.stair_pool, self._place_stair_instance)

    def _gc_pool(self, active_map, needed_set, pool_list):
        for key in list(active_map.keys()):
            if key not in needed_set:
                prim_path = active_map.pop(key)
                self._reset_prim(prim_path)
                pool_list.append(prim_path)

    def _allocate_pool(self, active_map, needed_set, pool_list, place_func):
        for key in needed_set:
            if key in active_map: continue
            if not pool_list: break
            
            prim_path = pool_list.pop()
            active_map[key] = prim_path
            place_func(prim_path, key)

    def _place_wall_instance(self, prim_path, key):
        x, y, direction = key
        z_top = self.open_cells[(x, y)]
        
        wall_h = z_top + self.wall_height
        wall_z_center = wall_h / 2.0

        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid(): return
        UsdGeom.Imageable(prim).MakeVisible()
        
        # custom_rl_env.py only creates "Solid" child, no Windows.
        # Ensure Solid is visible (just in case)
        solid_prim = self.stage.GetPrimAtPath(f"{prim_path}/Solid")
        if solid_prim.IsValid():
            UsdGeom.Imageable(solid_prim).MakeVisible()

        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        
        cx, cy = x * self.tile_size, y * self.tile_size
        thick = self.wall_thickness
        
        pos = Gf.Vec3d(0,0,0)
        scale = Gf.Vec3d(1,1,1)
        rot = Gf.Vec3d(0,0,0)

        if direction == 'E':
            pos = Gf.Vec3d(cx + 0.5, cy, wall_z_center)
            scale = Gf.Vec3d(thick, 1.0, wall_h)
        elif direction == 'W':
            pos = Gf.Vec3d(cx - 0.5, cy, wall_z_center)
            scale = Gf.Vec3d(thick, 1.0, wall_h)
        elif direction == 'N':
            pos = Gf.Vec3d(cx, cy + 0.5, wall_z_center)
            scale = Gf.Vec3d(1.0, thick, wall_h)
        elif direction == 'S':
            pos = Gf.Vec3d(cx, cy - 0.5, wall_z_center)
            scale = Gf.Vec3d(1.0, thick, wall_h)

        # Ops are [Translate, Rotate, Scale] (ensured by _ensure_prim_ops)
        ops[0].Set(pos)
        ops[1].Set(rot)
        ops[2].Set(scale)

    def _place_floor_instance(self, prim_path, key):
        x, y = key
        z = self.open_cells[(x, y)]
        
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid(): return

        # Hide floors that are too low (near zero) if desired, or keep them.
        # Keeping logic to hide very low floors to prevent z-fighting with ground plane
        if z < 0.05:
            UsdGeom.Imageable(prim).MakeInvisible()
            return
        
        UsdGeom.Imageable(prim).MakeVisible()
        
        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        
        # Scale: In custom_rl_env, Floor is a 1x1x1 Cube. We scale Z to 0.1
        ops[0].Set(Gf.Vec3d(x * self.tile_size, y * self.tile_size, z - 0.05))
        ops[1].Set(Gf.Vec3d(0,0,0))
        ops[2].Set(Gf.Vec3d(self.tile_size, self.tile_size, 0.1))

    def _place_stair_instance(self, prim_path, key):
        ux, uy, direction_to_lower, height_diff = key
        upper_z = self.open_cells[(ux, uy)]
        lower_z = upper_z - height_diff

        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid(): return
        UsdGeom.Imageable(prim).MakeVisible()

        num_steps_needed = int(round(height_diff / 0.1))
        num_steps_needed = max(1, min(5, num_steps_needed))
        
        # custom_rl_env.py created Step_0 through Step_4
        for i in range(5):
            step_prim = self.stage.GetPrimAtPath(f"{prim_path}/Step_{i}")
            if i < num_steps_needed:
                UsdGeom.Imageable(step_prim).MakeVisible()
            else:
                UsdGeom.Imageable(step_prim).MakeInvisible()

        cx_upper = ux * self.tile_size
        cy_upper = uy * self.tile_size
        
        pos = Gf.Vec3d(0,0,0)
        rot_z = 0
        
        if direction_to_lower == 'E': 
            rot_z = 180
            pos = Gf.Vec3d(cx_upper + 0.5, cy_upper, lower_z)
        elif direction_to_lower == 'W':
            rot_z = 0
            pos = Gf.Vec3d(cx_upper - 0.5, cy_upper, lower_z)
        elif direction_to_lower == 'N':
            rot_z = -90
            pos = Gf.Vec3d(cx_upper, cy_upper + 0.5, lower_z)
        elif direction_to_lower == 'S':
            rot_z = 90
            pos = Gf.Vec3d(cx_upper, cy_upper - 0.5, lower_z)

        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        
        ops[0].Set(pos)
        ops[1].Set(Gf.Vec3d(0, 0, rot_z))
        ops[2].Set(Gf.Vec3d(1,1,1))