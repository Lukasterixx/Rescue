import random
from pxr import Usd, UsdGeom, Gf, UsdPhysics

class MazeManager:
    def __init__(self, stage, start_pos=(0, 0, 0), grid_radius=15):
        self.stage = stage
        self.grid_radius = grid_radius
        self.tile_size = 1.0   
        self.wall_height = 1.0 
        self.wall_thickness = 0.05 
        
        # --- Maze Data ---
        self.open_cells = set() 
        self.passages = set() 
        self.frontier = set()

        # 1. Create a Safe Zone (3x3 Room at spawn)
        for x in range(-1, 2):
            for y in range(-1, 2):
                self.open_cells.add((x, y))
                # Connect internal neighbors
                if x < 1: self._add_passage((x, y), (x + 1, y))
                if y < 1: self._add_passage((x, y), (x, y + 1))

        # 2. Initialize Frontier
        for cell in list(self.open_cells):
            self._add_neighbors_to_frontier(cell[0], cell[1])

        # --- Object Pooling ---
        self.root_path = "/World/warehouse/Maze"
        self.pool_size = 800  
        self.wall_pool = []
        self.active_walls_map = {} 
        
        self._init_usd_pool()

    def _init_usd_pool(self):
        """Creates a pool of walls with compliant SRT order and Window variants."""
        if not self.stage.GetPrimAtPath("/World/warehouse"):
            UsdGeom.Xform.Define(self.stage, "/World/warehouse")

        if not self.stage.GetPrimAtPath(self.root_path):
            UsdGeom.Xform.Define(self.stage, self.root_path)
        
        print(f"[Maze] Initializing pool of {self.pool_size} walls (Solid + Window variants)...")
        
        for i in range(self.pool_size):
            wall_path = f"{self.root_path}/Wall_{i}"
            self.wall_pool.append(wall_path)
            
            # 1. Create Main Container Xform
            wall_xform = UsdGeom.Xform.Define(self.stage, wall_path)
            
            # Apply Default Transforms (Identity)
            xform_api = UsdGeom.Xformable(wall_xform)
            xform_api.AddTranslateOp()     
            xform_api.AddRotateXYZOp()     
            xform_api.AddScaleOp()         

            # 2. Create SOLID Wall Variant (Standard Cube)
            solid_path = f"{wall_path}/Solid"
            solid_cube = UsdGeom.Cube.Define(self.stage, solid_path)
            solid_cube.GetSizeAttr().Set(1.0)
            UsdPhysics.CollisionAPI.Apply(solid_cube.GetPrim())

            # 3. Create WINDOW Wall Variant (Group of 4 Cubes)
            win_path = f"{wall_path}/Window"
            UsdGeom.Xform.Define(self.stage, win_path)
            
            # Geometry Math: Wall is 1x1 (-0.5 to 0.5). Window is 0.5x0.5 centered.
            # We need 4 pieces: Bottom, Top, Left, Right.
            
            # Bottom Piece (Y: -0.5 to 0.5, Z: -0.5 to -0.25) -> Center Z: -0.375, Scale Z: 0.25
            self._create_sub_cube(f"{win_path}/Bottom", 0, 0, -0.375, 1.0, 1.0, 0.25)
            # Top Piece (Y: -0.5 to 0.5, Z: 0.25 to 0.5) -> Center Z: 0.375, Scale Z: 0.25
            self._create_sub_cube(f"{win_path}/Top", 0, 0, 0.375, 1.0, 1.0, 0.25)
            # Left Piece (Y: -0.5 to -0.25, Z: -0.25 to 0.25) -> Center Y: -0.375, Scale Y: 0.25, Scale Z: 0.5
            self._create_sub_cube(f"{win_path}/Left", 0, -0.375, 0, 1.0, 0.25, 0.5)
            # Right Piece (Y: 0.25 to 0.5, Z: -0.25 to 0.25) -> Center Y: 0.375, Scale Y: 0.25, Scale Z: 0.5
            self._create_sub_cube(f"{win_path}/Right", 0, 0.375, 0, 1.0, 0.25, 0.5)

            # Reset to holding area
            self._reset_wall(wall_path)

    def _create_sub_cube(self, path, x, y, z, sx, sy, sz):
        """Helper to create a collision-enabled cube part for the window frame."""
        cube = UsdGeom.Cube.Define(self.stage, path)
        cube.GetSizeAttr().Set(1.0)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        
        xform = UsdGeom.Xformable(cube)
        xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        xform.AddScaleOp().Set(Gf.Vec3d(sx, sy, sz))

    def _reset_wall(self, prim_path):
        """Moves a wall to the holding area and hides it."""
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid(): return
        UsdGeom.Imageable(prim).MakeInvisible()
        
        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        ops[0].Set(Gf.Vec3d(0, 0, -100.0)) # Hide underground
        ops[1].Set(Gf.Vec3d(0, 0, 0))      
        ops[2].Set(Gf.Vec3d(1.0, 1.0, 1.0))

    def _add_neighbors_to_frontier(self, x, y):
        for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]:
            nx, ny = x + dx, y + dy
            if (nx, ny) not in self.open_cells and (nx, ny) not in self.frontier:
                self.frontier.add((nx, ny))

    def _add_passage(self, cell_a, cell_b):
        """Registers a valid path between two cells."""
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
                self.open_cells.add((cx, cy))
                self._add_neighbors_to_frontier(cx, cy)

                if random.random() < 0.05: 
                    for n in open_neighbors:
                         if random.random() < 0.5: self._add_passage((cx, cy), n)
                else:
                    parent = random.choice(open_neighbors)
                    self._add_passage((cx, cy), parent)
            
            steps += 1

    def update(self, robot_pos_3d, force_complete=False):
        rx = int(round(robot_pos_3d[0] / self.tile_size))
        ry = int(round(robot_pos_3d[1] / self.tile_size))

        # 1. Grow Maze
        self._grow_maze(rx, ry, force_complete=force_complete)
        
        # 2. Determine Walls Needed
        needed_walls = set()
        
        r = self.grid_radius
        for x in range(rx - r, rx + r + 1):
            for y in range(ry - r, ry + r + 1):
                if (x, y) in self.open_cells:
                    for dx, dy, direction in [(1,0,'E'), (-1,0,'W'), (0,1,'N'), (0,-1,'S')]:
                        neighbor = (x + dx, y + dy)
                        if neighbor not in self.open_cells or not self._has_passage((x, y), neighbor):
                            needed_walls.add((x, y, direction))

        # 3. Garbage Collection
        active_keys = list(self.active_walls_map.keys())
        for key in active_keys:
            if key not in needed_walls:
                prim_path = self.active_walls_map.pop(key)
                self._reset_wall(prim_path)

        # 4. Allocation
        available_prims = [p for p in self.wall_pool if p not in self.active_walls_map.values()]
        
        for key in needed_walls:
            if key in self.active_walls_map:
                continue 
            
            if not available_prims:
                break 
            
            prim_path = available_prims.pop()
            self.active_walls_map[key] = prim_path
            
            x, y, direction = key
            self._place_wall_instance(prim_path, x, y, direction)

    def _place_wall_instance(self, prim_path, x, y, direction):
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid(): return
        
        # Make Main Xform Visible
        UsdGeom.Imageable(prim).MakeVisible()
        
        # --- Randomly Select Wall Type (Solid vs Window) ---
        # Get Child Prims
        solid_prim = self.stage.GetPrimAtPath(f"{prim_path}/Solid")
        window_prim = self.stage.GetPrimAtPath(f"{prim_path}/Window")
        
        # 20% Chance for Window, 80% Chance for Solid
        is_window = random.random() < 0.20
        
        if is_window:
            UsdGeom.Imageable(solid_prim).MakeInvisible()
            UsdGeom.Imageable(window_prim).MakeVisible()
        else:
            UsdGeom.Imageable(solid_prim).MakeVisible()
            UsdGeom.Imageable(window_prim).MakeInvisible()
        # ---------------------------------------------------

        # Position Logic (Unchanged, operates on Parent Xform)
        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        
        pos = Gf.Vec3d(0,0,0)
        scale = Gf.Vec3d(1,1,1)
        rot = Gf.Vec3d(0,0,0)
        
        if direction == 'E':
            pos = Gf.Vec3d((x + 0.5) * self.tile_size, y * self.tile_size, self.wall_height / 2)
            scale = Gf.Vec3d(self.wall_thickness, self.tile_size, self.wall_height)
        elif direction == 'W':
            pos = Gf.Vec3d((x - 0.5) * self.tile_size, y * self.tile_size, self.wall_height / 2)
            scale = Gf.Vec3d(self.wall_thickness, self.tile_size, self.wall_height)
        elif direction == 'N':
            pos = Gf.Vec3d(x * self.tile_size, (y + 0.5) * self.tile_size, self.wall_height / 2)
            scale = Gf.Vec3d(self.tile_size, self.wall_thickness, self.wall_height)
        elif direction == 'S':
            pos = Gf.Vec3d(x * self.tile_size, (y - 0.5) * self.tile_size, self.wall_height / 2)
            scale = Gf.Vec3d(self.tile_size, self.wall_thickness, self.wall_height)

        ops[0].Set(pos)
        ops[1].Set(rot)
        ops[2].Set(scale)