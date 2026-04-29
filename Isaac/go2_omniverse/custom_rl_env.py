# Copyright (c) 2024, RoboVerse community
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# ... (License header) ...

import math
import torch
import sys
from dataclasses import MISSING
from typing import Literal

# --- IMPORT OMNI.USD ---
import omni.usd

# ADDED IMPORTS FOR USD GENERATION
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf, UsdShade

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.utils import configclass
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG 

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp

from terrain_cfg import ROUGH_TERRAINS_CFG
from robots.g1.config import G1_CFG

# --- HELPER TO DETECT FLAT CONFIG ---
def is_flat_terrain():
    return "--terrain" in sys.argv and "flat" in sys.argv

# --- RESTORED MISSING HELPERS ---
base_command = {}

def constant_commands(env: ManagerBasedRLEnvCfg) -> torch.Tensor:
    global base_command
    tensor_lst = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32, device=env.device).repeat(env.num_envs, 1)
    for i in range(env.num_envs):
        tensor_lst[i] = torch.tensor(base_command.get(str(i), [0.0, 0.0, 0.0]), dtype=torch.float32, device=env.device)
    return tensor_lst
# --------------------------------

@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Configuration for the terrain scene with a legged robot."""

    # 1. TERRAIN SELECTION LOGIC
    if is_flat_terrain():
        terrain = TerrainImporterCfg(
            prim_path="/World/warehouse/ground",
            terrain_type="plane",
            debug_vis=False,
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.0,
                dynamic_friction=1.0,
            ),
        )
    else:
        # Uses the generator (Maze)
        terrain = TerrainImporterCfg(
            prim_path="/World/warehouse/ground",
            terrain_type="generator",
            terrain_generator=ROUGH_TERRAINS_CFG, 
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.0,
                dynamic_friction=1.0,
            ),
            debug_vis=False,
        )

    robot: ArticulationCfg = MISSING

    # Height Scanner
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        drift_range=(0.0, 0.0),
        mesh_prim_paths=["/World/warehouse/ground"],
        max_distance=100.0,
    )

    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)
    
    # Grey Studio style lighting:
    # neutral grey ambient dome + soft broad key light.
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=1800.0,
            color=(0.55, 0.55, 0.55),
            visible_in_primary_ray=True,
        ),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(
            intensity=600.0,
            color=(1.0, 1.0, 1.0),
            angle=35.0,
        ),
    )


@configclass
class ViewerCfg:
    # 1. ATTACH CAMERA TO ROBOT
    # Change this to "world" to stop Orbit's default translation-only tracking
    origin_type: Literal["world", "env", "asset_root"] = "world"
    asset_name: str | None = "robot" 
    env_index: int = 0
    
    # 2. POSITION: Behind (-X) and Above (+Z) looking at robot
    eye: tuple[float, float, float] = (-6.0, 0.0, 5) 
    lookat: tuple[float, float, float] = (0.0, 0.0, 0.0) 
    
    cam_prim_path: str = "/OmniverseKit_Persp"
    resolution: tuple[int, int] = (1920, 1080)


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        velocity_commands = ObsTerm(func=constant_commands)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        actions = ObsTerm(func=mdp.last_action)
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True)


@configclass
class CommandsCfg:
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(0.0, 0.0),
        rel_standing_envs=0.02,
        rel_heading_envs=1.0,
        heading_command=True,
        heading_control_stiffness=0.5,
        debug_vis=False,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 0.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0), heading=(0, 0)
        ),
    )


@configclass
class RewardsCfg:
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp, weight=1.0, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp, weight=0.5, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    dof_torques_l2 = RewTerm(func=mdp.joint_torques_l2, weight=-1.0e-5)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=0.125,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*FOOT"),
            "command_name": "base_velocity",
            "threshold": 0.5,
        },
    )
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*THIGH"), "threshold": 1.0},
    )
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=0.0)
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=0.0)


@configclass
class TerminationsCfg:
    time_out = None
    base_contact = None
   


@configclass
class EventCfg:
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.8, 0.8),
            "dynamic_friction_range": (0.6, 0.6),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (1.0, 1.0),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class LocomotionVelocityRoughEnvCfg(ManagerBasedRLEnvCfg):
    scene: MySceneCfg = MySceneCfg(num_envs=4096, env_spacing=2.5)
    viewer: ViewerCfg = ViewerCfg()
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        self.decimation = 4
        self.sim.render_interval = self.decimation
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material

        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt
        
        if getattr(self.curriculum, "terrain_levels", None) is not None:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = True
        else:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = False

@configclass
class UnitreeGo2CustomEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.robot = UNITREE_GO2_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot",
            init_state=ArticulationCfg.InitialStateCfg(
                # Maze safe-zone centre: start_r/start_c * cell_width ~= 6 * 1.2 = 7.2
                pos=(6.2, 6.2, 0.42),
                rot=(1.0, 0.0, 0.0, 0.0),
                joint_pos={
                    ".*L_hip_joint": 0.1,
                    ".*R_hip_joint": -0.1,
                    "F[L,R]_thigh_joint": 0.8,
                    "R[L,R]_thigh_joint": 1.0,
                    ".*_calf_joint": -1.5,
                },
                joint_vel={".*": 0.0},
            ),
        )
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/base"
        self.actions.joint_pos.scale = 0.25
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = ".*_foot"
        self.rewards.feet_air_time.weight = 0.01
        self.rewards.undesired_contacts = None
        self.rewards.dof_torques_l2.weight = -0.0002
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 0.75
        self.rewards.dof_acc_l2.weight = -2.5e-7
        self.terminations.base_contact = None

        # Live simulation: do not periodically reset the robot.
        self.terminations.time_out = None

@configclass
class G1RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        G1_MINIMAL_CFG = G1_CFG.copy()
        G1_MINIMAL_CFG.spawn.usd_path = "./robots/g1/g1.usd"
        self.scene.robot = G1_MINIMAL_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/torso_link"
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = ".*_ankle_roll_link"
        self.rewards.undesired_contacts = None
        self.terminations.base_contact.params["sensor_cfg"].body_names = ["torso_link"]
