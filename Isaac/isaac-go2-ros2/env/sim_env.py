from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.lab.terrains import TerrainImporterCfg, TerrainImporter
from omni.isaac.lab.terrains import TerrainGeneratorCfg
from env.terrain_cfg import HfUniformDiscreteObstaclesTerrainCfg
import omni.replicator.core as rep
import os

def create_custom_world():
    # Optional: add a semantic label to the ground
    ground_prim = rep.get.prims("/World/ground")
    with ground_prim:
        rep.modify.semantics([("class", "floor")])

    # Then reference your USD under /World/Custom
    assets_root_path = get_assets_root_path()  # typically .../Kit/Extensions/omni.isaac.core/exts/…
    # You can also bypass that and use an absolute path below
    prim = get_prim_at_path("/World/Custom")
    prim = define_prim("/World/Custom", "Xform")

    # Either reference from the Isaac asset tree:
    # usd_path = assets_root_path + "/Isaac/Environments/MyCustomFolder/custom_arena.usd"

    # …or simply give an absolute path:
    usd_path = "/home/lukas/my_worlds/custom_arena.usd"

    prim.GetReferences().AddReference(usd_path)

def add_semantic_label():
    ground_plane = rep.get.prims("/World/ground")
    with ground_plane:
    # Add a semantic label
        rep.modify.semantics([("class", "floor")])

def create_obstacle_sparse_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=100 ,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain) 

def create_obstacle_medium_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=200 ,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain) 


def create_obstacle_dense_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=400,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain) 


def create_farm_env():
    add_semantic_label()

    # 1) Get the directory where this sim_env.py lives
    this_folder = os.path.dirname(__file__)  # e.g. "/home/lukas/.../isaac-go2-ros2/env"

    # 2) Build the full path to FarmNoMDL.usd
    farm_usd_path = os.path.join(this_folder, "FarmNoMDL.usd")
    print(f"[sim_env] loading Farm USD from: {farm_usd_path}")

    # 3) Create a prim under /World for it (use a path that makes sense; e.g. "/World/Farm")
    prim = get_prim_at_path("/World/Farm")
    prim = define_prim("/World/Farm", "Xform")

    # 4) Reference that USD
    prim.GetReferences().AddReference(farm_usd_path)

def create_warehouse_env():
    add_semantic_label()
    assets_root_path = get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

def create_warehouse_forklifts_env():
    add_semantic_label()
    assets_root_path = get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
    prim.GetReferences().AddReference(asset_path)

def create_warehouse_shelves_env():
    add_semantic_label()
    assets_root_path = get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
    prim.GetReferences().AddReference(asset_path)

def create_full_warehouse_env():
    add_semantic_label()
    assets_root_path = get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

def create_hospital_env():
    add_semantic_label()
    assets_root_path = get_assets_root_path()
    prim = get_prim_at_path("/World/Hospital")
    prim = define_prim("/World/Hospital", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Hospital/hospital.usd"
    prim.GetReferences().AddReference(asset_path)

def create_office_env():
    add_semantic_label()
    assets_root_path = get_assets_root_path()
    prim = get_prim_at_path("/World/Office")
    prim = define_prim("/World/Office", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Office/office.usd"
    prim.GetReferences().AddReference(asset_path)
