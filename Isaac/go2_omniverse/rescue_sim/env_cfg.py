"""The rescue sim's environment: the walking Go2 with the D1 welded on, in the competition hall, a cup and a wrist
RealSense.

Two halves, kept apart on purpose:

* **The legs are Rescue's.** The walking policy (`agent_cfg.py`) was trained and measured against this
  observation layout (235 wide, height scan included), Unitree's measured motor curve (`go2_actuators.py`) and a
  0.25 action scale. None of that changes here. Its joint terms are scoped to the 12 leg joints, because the weld
  makes the robot 20 joints wide.
* **The arm and the camera are D1Training's** (`demos/cup/pick_demo/scene.py` and `flat_env_cfg.make_robot_cfg`,
  commit 5e19028), so a policy trained there meets the same arm here: the D1 welded at (0, 0, 0.08) with the
  published 3.152 kg (`arm_weld.py`, D1Training's `weld.py`), force drives at 4000/400 with the published torque
  limits and the measured speed ceilings (`arm_actuator="d1_servo"`), the fitted firmware planner between setpoint
  and drive (`d1_arm.D1Firmware`, run by the `D1ArmAction` term below), 8/4 solver iterations and self-collisions
  on. The wrist camera and its visual case are D1Training's too (`realsense.py`, `camera_asset.py`).

The policy never acts on the arm: `D1ArmAction` takes no actions, it only feeds the drives from the firmware, so the
policy's action vector stays the 12 it was trained with.
"""
from __future__ import annotations

from dataclasses import MISSING
import math

import torch

import isaaclab.sim as sim_utils
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg, ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

from go2_actuators import apply_go2_actuator

from . import d1_model
from .realsense import FAR_CLIP_M, NEAR_CLIP_M

# The 12 joints the walking checkpoint knows about. Scoping is safe because SceneEntityCfg resolves joint names in
# ascending articulation order, so the legs reach the policy in the order it trained with even though PhysX
# interleaves the arm's joints among them.
LEG_JOINTS = [".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"]
STANDING_LEG_POSE = {
    ".*L_hip_joint": 0.1, ".*R_hip_joint": -0.1,
    "F[L,R]_thigh_joint": 0.8, "R[L,R]_thigh_joint": 1.0, ".*_calf_joint": -1.5,
}
WELD_PATH = "{ENV_REGEX_NS}/Robot/D1"
LINK6_PATH = WELD_PATH + "/Link6"

# Velocity commands come from ROS (robot0/cmd_vel) and the keyboard, not the env's own sampler.
base_command: dict[str, list[float]] = {}


def constant_commands(env) -> torch.Tensor:
    out = torch.zeros(env.num_envs, 3, dtype=torch.float32, device=env.device)
    for i in range(env.num_envs):
        out[i] = torch.tensor(base_command.get(str(i), [0.0, 0.0, 0.0]), dtype=torch.float32, device=env.device)
    return out


# Height scan with walls removed rather than clipped (Rescue's custom_rl_env.py has the measurements): the policy
# trained on risers up to 16 cm reads a clipped wall as a step it cannot climb and backs away from it, and no clamp
# value fixes that. Rays past the threshold are replaced with the median of the walkable ones. Avoiding walls is
# nav2's job.
WALL_SCAN_THRESHOLD = -0.75


def height_scan_walls_removed(env, sensor_cfg, wall_threshold: float = WALL_SCAN_THRESHOLD, offset: float = 0.5):
    sensor = env.scene[sensor_cfg.name]
    h = sensor.data.pos_w[:, 2].unsqueeze(1) - sensor.data.ray_hits_w[..., 2] - offset
    finite = torch.isfinite(h)
    wall = (~finite) | (h < wall_threshold)
    walkable = torch.where(wall, torch.full_like(h, float("nan")), h)
    floor = torch.nanmedian(walkable, dim=1, keepdim=True).values
    fallback = torch.where(finite, h, torch.full_like(h, -float("inf"))).max(dim=1, keepdim=True).values
    floor = torch.where(torch.isnan(floor), fallback, floor)
    floor = torch.nan_to_num(floor, nan=0.0, posinf=0.0, neginf=0.0)
    return torch.where(wall, floor.expand_as(h), torch.nan_to_num(h, nan=0.0, posinf=0.0, neginf=0.0))


# ------------------------------------------------------------------------------------------ the arm's drives
class D1ArmAction(ActionTerm):
    """Feeds the D1's drives from the simulated firmware every physics step. Takes no policy actions.

    `SimulatedD1` (in `d1_drive.py`) is attached after the environment exists; until then the drives hold their
    initial targets, which are the rest pose.
    """

    cfg: "D1ArmActionCfg"

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._raw = torch.zeros(self.num_envs, 0, device=self.device)
        self.arm = None

    @property
    def action_dim(self) -> int:
        return 0

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._raw

    def process_actions(self, actions: torch.Tensor):
        if self.arm is not None:
            self.arm.begin_step()

    def apply_actions(self):
        if self.arm is not None:
            self.arm.apply(self._env.physics_dt)

    def reset(self, env_ids=None) -> None:
        pass


@configclass
class D1ArmActionCfg(ActionTermCfg):
    class_type: type = D1ArmAction


# ------------------------------------------------------------------------------------------ the scene
@configclass
class RescueSceneCfg(InteractiveSceneCfg):
    terrain: TerrainImporterCfg = MISSING
    robot: ArticulationCfg = MISSING
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        drift_range=(0.0, 0.0),
        mesh_prim_paths=MISSING,
        max_distance=100.0,
    )
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)
    # Rescue's grey studio lighting, which is also D1Training's pick scene's.
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(intensity=1800.0, color=(0.55, 0.55, 0.55), visible_in_primary_ray=True),
    )
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(intensity=600.0, color=(1.0, 1.0, 1.0), angle=35.0),
    )
    cup: RigidObjectCfg = MISSING
    wrist_cam: CameraCfg = MISSING
    rs_body: AssetBaseCfg = None


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        # Term for term the stock rough Go2's, which the walking policy was trained on: 3+3+3+3+12+12+12+187.
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(func=constant_commands)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)})
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)})
        actions = ObsTerm(func=mdp.last_action)
        height_scan = ObsTerm(func=height_scan_walls_removed, params={"sensor_cfg": SceneEntityCfg("height_scanner")},
                              clip=(-1.0, 1.0))

        def __post_init__(self):
            # No corruption while it is driven by hand or by nav2.
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=LEG_JOINTS, scale=0.25,
                                           use_default_offset=True)
    d1_arm = D1ArmActionCfg(asset_name="robot")


@configclass
class CommandsCfg:
    # Frozen: constant_commands() is the source. Kept because the reward terms name it.
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot", resampling_time_range=(0.0, 0.0), rel_standing_envs=0.02, rel_heading_envs=1.0,
        heading_command=True, heading_control_stiffness=0.5, debug_vis=False,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(lin_vel_x=(0.0, 0.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0),
                                                    heading=(0, 0)),
    )


@configclass
class RewardsCfg:
    # Inert: nothing trains here. The manager needs one term.
    track_lin_vel_xy_exp = RewTerm(func=mdp.track_lin_vel_xy_exp, weight=1.5,
                                   params={"command_name": "base_velocity", "std": math.sqrt(0.25)})


@configclass
class TerminationsCfg:
    # A live sim: nothing resets the robot out from under whoever is driving it.
    time_out = None


@configclass
class EventCfg:
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names=".*"), "static_friction_range": (0.8, 0.8),
                "dynamic_friction_range": (0.6, 0.6), "restitution_range": (0.0, 0.0), "num_buckets": 64},
    )


@configclass
class RescueEnvCfg(ManagerBasedRLEnvCfg):
    scene: RescueSceneCfg = RescueSceneCfg(num_envs=1, env_spacing=20.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        self.decimation = 4
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.episode_length_s = 3600.0
        self.sim.disable_contact_processing = True
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        self.scene.contact_forces.update_period = self.sim.dt
        # Isaac Lab's own viewer: pick "asset root" in its window to follow the robot, or leave it on the world and
        # move the camera yourself. Nothing here moves it after startup.
        self.viewer.origin_type = "world"
        self.viewer.resolution = (1920, 1080)


def robot_cfg(robot_usd: str, spawn, spawn_rotation, arm_q, finger_travel) -> ArticulationCfg:
    """The welded Go2+D1: Rescue's legs, D1Training's arm."""
    cfg = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    joint_pos = dict(STANDING_LEG_POSE)
    joint_pos.update({name: float(q) for name, q in zip(d1_model.ARM_JOINTS, arm_q)})
    joint_pos.update({"Joint7_1": float(finger_travel), "Joint7_2": -float(finger_travel)})
    # Force drives: gains in N*m/rad (legs already are; the URDF import authors the arm as acceleration drives,
    # which let it sag 0.11 rad inside its torque limits, D1Training Week 1). `replace`, because the stock cfg
    # shares its spawn cfg.
    cfg.spawn = cfg.spawn.replace(
        usd_path=robot_usd,
        joint_drive_props=sim_utils.JointDrivePropertiesCfg(drive_type="force"),
        articulation_props=cfg.spawn.articulation_props.replace(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4),
    )
    cfg.init_state = ArticulationCfg.InitialStateCfg(pos=tuple(spawn), rot=tuple(spawn_rotation),
                                                     joint_pos=joint_pos, joint_vel={".*": 0.0})
    apply_go2_actuator(cfg)
    actuators = dict(cfg.actuators)
    actuators["d1_arm"] = ImplicitActuatorCfg(
        joint_names_expr=["Joint[1-6]"], stiffness=d1_model.ARM_STIFFNESS, damping=d1_model.ARM_DAMPING,
        effort_limit_sim=dict(d1_model.EFFORT_LIMIT_NM), velocity_limit_sim=dict(d1_model.VELOCITY_LIMIT_RAD_S),
    )
    # The URDF's 15 N and 0.02 m/s carry through; gripper timing has not been measured on the D1.
    actuators["d1_gripper"] = ImplicitActuatorCfg(
        joint_names_expr=["Joint7_.*"], stiffness=d1_model.GRIPPER_STIFFNESS, damping=d1_model.GRIPPER_DAMPING,
    )
    cfg.actuators = actuators
    return cfg


def make_env_cfg(*, terrain_usd: str, scan_mesh: str, robot_usd: str, cup_usd: str, cup_pose, camera, mount,
                 camera_body_usd: str | None, spawn, spawn_rotation, arm_q, finger_travel, device: str = "cuda:0",
                 seed: int = 42, camera_period_s: float = 0.0) -> RescueEnvCfg:
    cfg = RescueEnvCfg()
    cfg.seed = seed
    cfg.sim.device = device
    scene = cfg.scene
    scene.terrain = TerrainImporterCfg(
        prim_path="/World/competition", terrain_type="usd", usd_path=terrain_usd, env_spacing=20.0, debug_vis=False,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply", restitution_combine_mode="multiply",
            static_friction=1.0, dynamic_friction=1.0),
    )
    scene.height_scanner.mesh_prim_paths = [scan_mesh]
    scene.robot = robot_cfg(robot_usd, spawn, spawn_rotation, arm_q, finger_travel)
    scene.cup = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cup",
        spawn=sim_utils.UsdFileCfg(usd_path=cup_usd),
        init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(cup_pose[:3]), rot=tuple(cup_pose[3:])),
    )
    # D1Training scene.py's wrist camera: the calibration's intrinsics, the mount's pose on Link6, the near clip past
    # its own case. Its images are read back at the rate they are published (D1Training reads every policy step).
    scene.wrist_cam = CameraCfg(
        prim_path=LINK6_PATH + "/wrist_cam",
        update_period=camera_period_s,
        width=camera.width, height=camera.height,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
            intrinsic_matrix=camera.intrinsic_matrix.flatten().tolist(), width=camera.width, height=camera.height,
            clipping_range=(NEAR_CLIP_M, FAR_CLIP_M)),
        offset=CameraCfg.OffsetCfg(pos=tuple(mount.pos_link6), rot=mount.quat_wxyz(), convention="ros"),
    )
    if camera_body_usd:
        scene.rs_body = AssetBaseCfg(
            prim_path=LINK6_PATH + "/rs_body",
            spawn=sim_utils.UsdFileCfg(usd_path=camera_body_usd),
            init_state=AssetBaseCfg.InitialStateCfg(pos=tuple(mount.pos_link6), rot=mount.quat_wxyz()),
        )
    cfg.sim.physics_material = scene.terrain.physics_material
    return cfg
