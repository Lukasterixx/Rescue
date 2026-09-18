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

from terrain_cfg import ROUGH_TERRAINS_CFG, ARENA_TERRAIN_CFG
import arena_layout
from go2_actuators import apply_go2_actuator
from robots.g1.config import G1_CFG

# --- HELPER TO DETECT FLAT CONFIG ---
def is_flat_terrain():
    return "--terrain" in sys.argv and "flat" in sys.argv


def _cli_value(flag: str, default: str) -> str:
    """`--flag value` or `--flag=value` from sys.argv, else `default`.

    The scene config below is built at class-definition time, before the arguments parsed in
    omniverse_sim.py can reach it, so it reads the command line directly like is_flat_terrain().
    """
    for i, arg in enumerate(sys.argv):
        if arg == flag and i + 1 < len(sys.argv):
            return sys.argv[i + 1]
        if arg.startswith(flag + "="):
            return arg.split("=", 1)[1]
    return default


def custom_env_name() -> str:
    """What `--custom_env` asked for. "arena" (the default) and "maze" are procedural terrains
    built here; any other value gets the arena terrain plus whatever static USD
    omniverse_sim.setup_custom_env() loads for it."""
    return _cli_value("--custom_env", "arena")


def arena_start_name() -> str:
    """Which of the arena drawing's START|END boxes the robot spawns in (`--arena_start`)."""
    return _cli_value("--arena_start", arena_layout.DEFAULT_START)

# The 12 joints the walking checkpoint knows about.
#
# Only relevant under `--arm_mount weld`: welding folds the D1's 8 joints into
# the Go2's articulation, so the robot has 20 where the policy expects 12, and
# every joint-space observation and action has to be scoped back to these.
# `omniverse_sim._scope_env_cfg_to_legs()` applies it. Under `--arm_mount
# teleport` the arm is its own articulation and nothing here needs to change.
#
# Scoping is safe because `SceneEntityCfg` resolves joint names through
# `find_joints(..., preserve_order=False)`, which returns indices in ascending
# articulation order. Subsetting cannot reorder the legs relative to each other,
# so the 12 values reach the policy in exactly the order it expects -- even
# though PhysX interleaves the arm's Joint1 between the thigh and calf joints.
LEG_JOINTS = [".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"]

# --- RESTORED MISSING HELPERS ---
base_command = {}

def constant_commands(env: ManagerBasedRLEnvCfg) -> torch.Tensor:
    global base_command
    tensor_lst = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32, device=env.device).repeat(env.num_envs, 1)
    for i in range(env.num_envs):
        tensor_lst[i] = torch.tensor(base_command.get(str(i), [0.0, 0.0, 0.0]), dtype=torch.float32, device=env.device)
    return tensor_lst
# --------------------------------

# Height scan with WALLS REMOVED, not clamped.
#
# THE PROBLEM IT SOLVES, measured. Driven in the maze the robot refused forward commands and
# reversed instead. `training/measure_scan.py` reproduced it on flat ground by overwriting
# only the height scan: with a wall 0.5 m ahead, a held +1.0 m/s forward command produced
# -0.53 m/s, i.e. the robot drove backwards, and side walls alone in a corridor took forward
# motion from +0.53 m/s to zero.
#
# WHY. `height_scan` reports ground height relative to a nominal, more negative meaning
# higher, and the observation clips at -1.0. A 1 m wall reads -1.165 and clips. The policy
# trained on risers up to 0.16 m, which read about -0.33, so its only learned meaning for a
# large negative ahead is "a step up" -- and one this large is a step it cannot climb, so it
# backs away. Correct behaviour on the terrain it knows; wrong in a corridor.
#
# WHY CLAMPING DOES NOT FIX IT, also measured. A clamp only makes the wall a SHORTER step, and
# the sweep in measure_scan.py found no value that helps -- reversal at every one of -1.00,
# -0.85, -0.70, -0.55 and -0.40, WORST at -0.70 (-1.19 m/s) where the wall reads most like a
# steep but plausible stair. The arithmetic says why: a clamp of v implies a step of
# (0.335 - 0.5 - v) m, so even -0.40 still says 0.24 m, well above the 0.16 m the policy has
# ever climbed. The value that would read as climbable is about -0.33, and the maze's own
# 10 cm risers read -0.57, so that clamp blinds the robot to the steps it is there to climb.
#
# SO THE WALL IS REMOVED INSTEAD OF SHORTENED. Rays that read past `wall_threshold` are not a
# surface this robot can ever step onto, so they are replaced with the median of the rays that
# ARE walkable -- the local floor. The policy then sees the floor continuing, which is the
# honest input for a locomotion controller whose job stops at the gait. Avoiding the wall is
# nav2's job, through the costmap, and it has the lidar to do it with.
#
# THE THRESHOLD HAS ROOM. In the maze the steepest thing is a 10 cm riser on a 20 cm tread,
# reaching about -0.57 at the front of the scan (flights are 2-4 steps, so 0.4 m of rise is
# the worst case). A wall reads -1.165 before clipping. -0.75 sits between them with 0.18 to
# spare on the stair side, so no step in this maze is ever mistaken for a wall.
#
# WHAT IT COSTS. The policy becomes blind to anything it could not climb anyway, including
# obstacles that are not walls. It will walk into such a thing if commanded to. That is
# accepted deliberately: the alternative is the robot deciding for itself where it may go,
# which is the nav stack's decision and not the gait's.
#
# THE PROPER FIX IS IN TRAINING, not here. A policy that had seen walls would learn that a
# clipped column is simply not ground and needs no filter. That costs a training run; this is
# the change that makes the current checkpoint usable in the maze tonight. See
# training/README.md.
WALL_SCAN_THRESHOLD = -0.75


def height_scan_walls_removed(env, sensor_cfg, wall_threshold: float = WALL_SCAN_THRESHOLD,
                              offset: float = 0.5) -> torch.Tensor:
    """`mdp.height_scan`, with rays that hit a wall replaced by the local floor."""
    sensor = env.scene[sensor_cfg.name]
    h = sensor.data.pos_w[:, 2].unsqueeze(1) - sensor.data.ray_hits_w[..., 2] - offset

    # A ray that escapes the mesh returns inf. Treat it as a wall rather than letting one
    # non-finite value poison the median and, through it, every column of the observation.
    finite = torch.isfinite(h)
    wall = (~finite) | (h < wall_threshold)

    walkable = torch.where(wall, torch.full_like(h, float("nan")), h)
    floor = torch.nanmedian(walkable, dim=1, keepdim=True).values
    # Every ray a wall (nose into a corner): fall back to the LEAST negative reading, i.e.
    # the lowest ground the scan can see, which is the best floor estimate available.
    fallback = torch.where(finite, h, torch.full_like(h, -float("inf"))).max(dim=1, keepdim=True).values
    floor = torch.where(torch.isnan(floor), fallback, floor)
    floor = torch.nan_to_num(floor, nan=0.0, posinf=0.0, neginf=0.0)

    return torch.where(wall, floor.expand_as(h), torch.nan_to_num(h, nan=0.0, posinf=0.0, neginf=0.0))



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
        # Procedural terrain: the obstacle arena by default, the maze behind --custom_env maze.
        terrain = TerrainImporterCfg(
            prim_path="/World/warehouse/ground",
            terrain_type="generator",
            terrain_generator=ROUGH_TERRAINS_CFG if custom_env_name() == "maze" else ARENA_TERRAIN_CFG,
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
            # NOT `mdp.height_scan` -- see `height_scan_walls_removed` above. The maze has
            # 1 m walls and the policy was trained on a terrain whose tallest feature was a
            # 16 cm riser, so an unfiltered wall reads as an unclimbable step and the robot
            # reverses away from it. Measured, with the numbers, in that function's comment.
            func=height_scan_walls_removed,
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
        if custom_env_name() == "maze":
            # Maze safe-zone centre (start_r/start_c * cell_width ~= 6 * 1.2), in the sub-terrain's
            # own (0..size) frame: the generator puts the env origin at -size/2, so this lands at
            # world (0.2, 0.2).
            spawn_pos, spawn_rot = (6.2, 6.2, 0.42), (1.0, 0.0, 0.0, 0.0)
        else:
            # One of the drawing's START|END boxes, outside the arena. The arena generator returns
            # its centre as the terrain origin, so these are world coordinates.
            spawn_pos, spawn_rot = arena_layout.start_pose(arena_start_name())
        self.scene.robot = UNITREE_GO2_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot",
            init_state=ArticulationCfg.InitialStateCfg(
                pos=spawn_pos,
                rot=spawn_rot,
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

        # THE LEGS RUN UNITREE'S MEASURED MOTOR, not Isaac Lab's ideal one -- and this line
        # has to stay in step with the training task. `training/go2_rescue/` applies the same
        # swap from the same module (go2_actuators.py), because a policy trained against one
        # torque-speed curve and played back against another is being run on a robot it never
        # saw. The stock DCMotor offers a flat 23.5 N*m of BRAKING torque at any joint speed;
        # the real motor gives 4.3 at 27 rad/s, which is precisely the authority a policy
        # leans on when it catches the body on a step edge.
        #
        # Policies trained BEFORE this line existed (the 2026 rough policy in agent_cfg.py)
        # were trained against the ideal motor. They still load and still walk -- the motor is
        # not part of the observation -- but they are now being played on a slightly different
        # robot than they trained on. Remove this line to put such a policy back on its own
        # motor; leave it for anything trained by `training/go2_rescue/`.
        apply_go2_actuator(self.scene.robot)

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
