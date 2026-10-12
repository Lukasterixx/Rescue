"""The K-Rail Square policy: step up onto a deck, over K-rails, and down off it, tracking the velocity command.

Task ids (`__init__.py`): `Isaac-Velocity-Rescue-KRail-Go2-v0` (train) and `...-Play-v0`. The result is exported as a
bundle the rescue sim loads with `--policy` (`../export_krail.py`), not as a 235-wide rsl_rl checkpoint.

WHAT IT IS BUILT FROM, AND WHY NOT FROM THIS PACKAGE'S OTHER TASK. `rough_env_cfg.py` is P2Dingo's Spot reward set,
and this package's README records why it never climbed: a step costs the gait and velocity terms more than it pays
(27 of the 29-unit gap was `gait` alone). This task starts instead from stock Isaac Lab's
`UnitreeGo2RoughEnvCfg`, which does learn stairs on a bare Go2 (its reward weights, events and 0.25 action scale,
set here on the base locomotion config, because the Go2's own `__post_init__` edits terms by their stock names), and
changes what the target needs:

* THE ROBOT IS THE SIM'S, built by the sim's own `rescue_sim.env_cfg.robot_cfg`: the D1 welded at its measured mount
  with its 3.152 kg model, folded at the rest the real arm sits in (F-023; Joint2 and Joint3 on their limits, the
  upper arm laid back along the dog), force drives at 4000/400 under the arm's torque limits, Unitree's measured
  leg motor (`go2_actuators.py`), 8/4 solver iterations and self-collisions, as in the sim. The earlier tasks held
  the arm at zero, which is not where it rides. The dock is not drawn (visual only, and 20k triangles a robot).
* THE GROUND IS THE TARGET'S: decks 3-24 cm high with K-rails on and between them (`krail_terrain.py`; the square is
  a 12.2 cm deck with 10 cm rails), plus stairs both ways, boxes, rough ground, slopes and flat ground.
* THE HEIGHT SCAN IS FINER: the stock 1.6 x 1.0 m grid centred on the base, at 5 cm instead of 10 (33 x 21 = 693
  rays). A 10 cm rail at 10 cm spacing is one row of rays and +-5 cm of where; at 5 cm it is two rows.
* COMMANDS AND THE TERRAIN LADDER: `krail_mdp.py`. +-1.0 m/s forward, +-0.8 sideways, +-1.5 rad/s, held for 5-10 s;
  half the robots heading-controlled walkers for the ladder, the rest held commands, in-place turns and stands, as
  nav2 sends them. A quarter of the resamples are stands of 1-4 s, so robots are stopped from a walk often, as a
  teleop key let go and nav2 stop them: at once.
* TRACKING IS PAID FOR IN THE YAW FRAME (`track_lin_vel_xy_yaw_frame_exp`, `track_ang_vel_z_world_exp`): nav2 asks
  for planar velocity, and on a step the body pitches, so the body frame would charge a climbing robot for the
  climb. Yaw tracking is weighted 1.5, up from the stock Go2's 0.75, and a sharp pair (std 0.2) sits beside the
  stock pair for low speeds.
* NOTHING CHARGES AN ATTEMPT AT A STEP that standing in front of it does not also pay: no termination penalty, no
  tilt termination, no stumble or shin-contact penalty, no orientation penalty against gravity. Run v1 had all five
  and learnt to stop at every riser (`KRailRewardsCfg` has the numbers).
* THE WEIGHT IS CARRIED BY ALL FOUR FEET (run v3): the centre of mass over the middle of the feet, the body level
  with the ground under it, and a stop on four feet, not two (`KRailRewardsCfg`, "the stance terms").

THE OBSERVATION is the stock rough Go2's, term for term, with the finer scan: base linear and angular velocity,
gravity, the command, the 12 legs' positions and velocities, the last action, and the scan, unscaled. It reads
base_lin_vel, as the sim's own policies do; the sim provides it. A real Go2 cannot measure it directly. The critic
gets the same terms without noise.
"""
from __future__ import annotations

import copy
import math
import os
import sys
from pathlib import Path

from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.sensors import patterns
from isaaclab.terrains import TerrainGeneratorCfg
import isaaclab.terrains as terrain_gen
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg

from . import krail_mdp
from .krail_terrain import KRailDeckTerrainCfg

LEG_JOINTS = [".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"]
FEET = ".*_foot"

#: The scan grid. The sim reads it back from the bundle's params/env.yaml (`exported_policy.Bundle._read_scan`).
SCAN_RESOLUTION_M = 0.05
SCAN_SIZE_M = (1.6, 1.0)

KRAIL_TERRAINS_CFG = TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        # Twenty columns, so each proportion is a whole number of columns (0.05 each).
        "krail_decks": KRailDeckTerrainCfg(proportion=0.30),
        "krail_field": KRailDeckTerrainCfg(proportion=0.15, num_decks=0, num_ground_rails=12),
        "steps_down": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.10, step_height_range=(0.03, 0.22), step_width=0.6, platform_width=2.0, border_width=1.0,
            holes=False),
        "steps_up": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.15, step_height_range=(0.03, 0.22), step_width=0.6, platform_width=2.0, border_width=1.0,
            holes=False),
        "boxes": terrain_gen.MeshRandomGridTerrainCfg(
            proportion=0.10, grid_width=0.45, grid_height_range=(0.02, 0.12), platform_width=2.0),
        "rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.05, noise_range=(0.01, 0.06), noise_step=0.01, border_width=0.25),
        "slopes": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.05, slope_range=(0.0, 0.3), platform_width=2.0, border_width=0.25),
        "flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.10),
    },
)


def sim_robot_cfg():
    """The rescue sim's robot (`rescue_sim.env_cfg.robot_cfg`), welded fresh into this process's own folder, with the
    arm folded at the sim's rest. Must run after the app is up (the weld imports the URDF through Kit)."""
    root = Path(__file__).resolve().parents[2]  # Isaac/go2_omniverse, through the install symlink
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

    import xml.etree.ElementTree as ET

    from arm_weld import D1_MOUNT_POS, build_welded_robot_usd
    from rescue_sim import d1_model
    from rescue_sim.env_cfg import robot_cfg

    urdf = root / "d1_arm" / "d1.urdf"
    out = root / "d1_arm" / "generated" / f"krail_{os.getpid()}" / "go2_d1_krail.usd"
    weld = build_welded_robot_usd(go2_usd_path=UNITREE_GO2_CFG.spawn.usd_path, d1_urdf_path=str(urdf),
                                  out_usd_path=str(out), mount_pos=D1_MOUNT_POS, arm_mass_kg=3.152,
                                  dock_visual=False)
    joints = {j.get("name"): j.find("limit") for j in ET.parse(urdf).getroot().findall("joint")}  # as sim.py reads them
    lower = [float(joints[n].get("lower")) for n in d1_model.ARM_JOINTS]
    upper = [float(joints[n].get("upper")) for n in d1_model.ARM_JOINTS]
    rest_q = d1_model.rest_joint_rad(lower, upper)
    travel = d1_model.units_to_travel(d1_model.REST_GRIPPER_UNITS)
    cfg = robot_cfg(weld.usd_path, spawn=(0.0, 0.0, 0.42), spawn_rotation=(1.0, 0.0, 0.0, 0.0), arm_q=rest_q,
                    finger_travel=travel)
    print(f"[krail] training robot: the sim's, arm at rest {[round(math.degrees(q), 1) for q in rest_q]} deg")
    return cfg


@configclass
class KRailObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
                            noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
                            noise=Unoise(n_min=-1.5, n_max=1.5))
        actions = ObsTerm(func=mdp.last_action)
        height_scan = ObsTerm(func=mdp.height_scan, params={"sensor_cfg": SceneEntityCfg("height_scanner")},
                              noise=Unoise(n_min=-0.05, n_max=0.05), clip=(-1.0, 1.0))

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(PolicyCfg):
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class KRailRewardsCfg:
    """Stock rough Go2's set, tracked in the yaw frame, plus a sharp tracking pair and a stand-still term.

    WHAT IS NOT HERE, AND WHY (run v1, 2026-10-11). v1 also had a -10 termination penalty, an episode end at 1 rad
    of tilt, a toe-into-an-edge penalty, a thigh/shin contact penalty and an orientation penalty, every one of which
    charges an ATTEMPT at a step and none of which charges standing in front of it. At iteration 1500, with the
    curriculum's mean level at 4.9, it would not climb a 9 cm riser: 24 robots held at 0.5 m/s in the up-stairs pit
    (difficulty 0.3) did not move (0.2 % of the command), 1 of 24 got onto a square-height deck and then fell, and in
    the K-Rail Square it stopped 0.6 m short of the deck in 9 of 10 trials. The mean level was carried by the flat,
    rough and slope columns. That is the refusal this package's README records for the earlier stair runs. The stock
    set, which learns stairs on a bare Go2, has none of those terms.

    THE SHARP PAIR (std 0.2 m/s and rad/s, beside the stock std 0.5). At iteration 1000 v1 stood still for a held
    0.3 m/s and turned at 67 % of 0.5 rad/s: under std 0.5, standing for 0.3 m/s keeps exp(-0.36) = 70 % of the
    tracking reward, and walking's torque and action-rate costs are about the rest. Under std 0.2 it keeps 11 %. It
    also makes standing in front of a step cost more. The stock pair stays, as the gradient that pays a robot for
    crawling up a step at half speed.

    YAW WEIGHTS 1.5 AND 1.0 from v2's iteration 3000 (1.0 and 0.5 before). With the K-Rail Square solved there
    (9 of 10 routes, no falls) yaw was the weak axis: on flat ground 0.5 rad/s was tracked at 69 % and 1.0-1.5 at
    88-93 %, against 99 % at 0.5 and 1.0 for the sim's current flat policy on the same robot.

    THE STANCE TERMS (run v3, 2026-10-12 to 13, resumed from v2's 5999). v2's policy carried its weight on its front
    feet and handstood when it was stopped. On flat ground (`../measure_stance.py`, 16 robots, GPU) the front pair
    carried 56-83 % of the weight walking at 1.0 down to 0.3 m/s and 74-86 % standing after a walk, and every robot
    stopped from a forward walk lifted both rear feet, for 0.33-0.46 s and up to 21 cm, its nose down 10-12 deg on
    average and 29 at worst. The arm puts the 18.2 kg robot's centre of mass 7 cm ahead of the middle of its feet in
    the default pose; v2 stood with its front feet tucked further under; and it was seldom told to stop (8 % of
    resamples, now 25 %, `Go2KRailEnvCfg`). Each term was added for what the measurements showed next:

    * `com_over_feet`: the centre of mass over the middle of the four feet, along the heading, either way.
    * `feet_down_still` and `ends_down_still`, while the command is to stand: each foot off the ground, and each end
      (front or rear) with both its feet off. At -1 per foot and nothing per end, nearly every forward stop still
      lifted both rear feet, for 0.15-0.3 s. At -5 and then -8 per end, from none to most of 16 robots did on the
      slow stops (0.3-0.5 m/s), checkpoint to checkpoint; -15 with the tilt term below.
    * `stand_still`'s 1 s grace. Charged from a stop's first step, the quickest way back to the default pose was to
      swing both rear legs into place at once (traced at v3's iteration 6500: both up for about 0.4 s, 16 cm, nose
      down 11 deg). Weakening the term to -0.2 instead let the robot stand by squatting, the base 0.15 m over its
      feet and the hips splayed 0.6-0.7 rad.
    * `base_low`, for that squat: the base under 0.26 m above its feet's mean height (0.33 m in the default pose).
    * `tilt_from_ground`: with the terms above, the policy centred its weight by tucking its rear feet forward and
      walked nose down, 8 deg at 0.3 m/s and 6 at 0.5, and its slow stops pitched as far as v2's. Measured against
      the ground's plane, not gravity, so a climb is barely charged; v1's orientation penalty, above, charged it in
      full.
    """

    track_lin_vel_xy = RewTerm(func=mdp.track_lin_vel_xy_yaw_frame_exp, weight=1.5,
                               params={"command_name": "base_velocity", "std": math.sqrt(0.25)})
    track_ang_vel_z = RewTerm(func=mdp.track_ang_vel_z_world_exp, weight=1.5,
                              params={"command_name": "base_velocity", "std": math.sqrt(0.25)})
    track_lin_vel_xy_fine = RewTerm(func=mdp.track_lin_vel_xy_yaw_frame_exp, weight=1.0,
                                    params={"command_name": "base_velocity", "std": 0.2})
    track_ang_vel_z_fine = RewTerm(func=mdp.track_ang_vel_z_world_exp, weight=1.0,
                                   params={"command_name": "base_velocity", "std": 0.2})
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-1.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    dof_torques_l2 = RewTerm(func=mdp.joint_torques_l2, weight=-2.0e-4,
                             params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)})
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7,
                         params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)})
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.0,
                             params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)})
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    feet_air_time = RewTerm(func=mdp.feet_air_time, weight=0.05,
                            params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=FEET),
                                    "command_name": "base_velocity", "threshold": 0.3})
    stand_still = RewTerm(func=krail_mdp.stand_still, weight=-0.5,
                          params={"command_name": "base_velocity", "grace_s": 1.0,
                                  "asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)})
    base_low = RewTerm(func=krail_mdp.base_low_over_feet, weight=-20.0,
                       params={"asset_cfg": SceneEntityCfg("robot", body_names=FEET), "min_height": 0.26})
    tilt_from_ground = RewTerm(func=krail_mdp.tilt_from_ground, weight=-10.0,
                               params={"asset_cfg": SceneEntityCfg("robot"),
                                       "sensor_cfg": SceneEntityCfg("height_scanner")})
    com_over_feet = RewTerm(func=krail_mdp.com_ahead_of_feet, weight=-10.0,
                            params={"asset_cfg": SceneEntityCfg("robot", body_names=FEET)})
    feet_down_still = RewTerm(func=krail_mdp.feet_off_ground_still, weight=-2.0,
                              params={"command_name": "base_velocity",
                                      "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FEET)})
    ends_down_still = RewTerm(func=krail_mdp.end_off_ground_still, weight=-15.0,
                              params={"command_name": "base_velocity",
                                      "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FEET)})


@configclass
class KRailCurriculumCfg:
    terrain_levels = CurrTerm(func=krail_mdp.terrain_levels_progress)


@configclass
class KRailTerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # The trunk only. With the sim's self-collisions on, a front leg brushes Head_lower in an in-place turn (264 N
    # measured on flat ground with the 2026-09-18 rough policy), so a head term would end episodes for turning. The
    # trunk showed no self-contact; a real fall reaches it. (No tilt term: v1's at 1 rad was one more charge on an
    # attempt at a step; see KRailRewardsCfg.)
    base_contact = DoneTerm(func=mdp.illegal_contact,
                            params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"),
                                    "threshold": 1.0})


@configclass
class Go2KRailEnvCfg(LocomotionVelocityRoughEnvCfg):
    observations: KRailObservationsCfg = KRailObservationsCfg()
    rewards: KRailRewardsCfg = KRailRewardsCfg()
    curriculum: KRailCurriculumCfg = KRailCurriculumCfg()
    terminations: KRailTerminationsCfg = KRailTerminationsCfg()

    def __post_init__(self):
        super().__post_init__()
        # --- the robot: the sim's --------------------------------------------------------------------------------
        self.scene.robot = sim_robot_cfg()
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/base"
        self.actions.joint_pos.joint_names = LEG_JOINTS
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.use_default_offset = True

        # --- the ground and the scan ---------------------------------------------------------------------------------
        self.scene.terrain.terrain_generator = copy.deepcopy(KRAIL_TERRAINS_CFG)
        self.scene.terrain.terrain_generator.curriculum = True  # the parent set it on the stock generator
        self.scene.terrain.max_init_terrain_level = 2
        self.scene.height_scanner.pattern_cfg = patterns.GridPatternCfg(resolution=SCAN_RESOLUTION_M,
                                                                        size=list(SCAN_SIZE_M))
        # More contact pairs than the bare Go2: the arm's links and self-collisions.
        self.sim.physx.gpu_max_rigid_patch_count = 20 * 2**15

        # --- commands -----------------------------------------------------------------------------------------
        self.commands.base_velocity = krail_mdp.KRailVelocityCommandCfg(
            asset_name="robot",
            resampling_time_range=(5.0, 10.0),
            # A stand on a quarter of the resamples, and a short one: robots are stopped from a walk often (v2 had
            # 0.08 and 5-10 s; see KRailRewardsCfg, "the stance terms").
            rel_standing_envs=0.25,
            standing_time_range=(1.0, 4.0),
            rel_heading_envs=0.5,
            heading_command=True,
            heading_control_stiffness=0.5,
            debug_vis=True,
            ranges=mdp.UniformVelocityCommandCfg.Ranges(
                lin_vel_x=(-1.0, 1.0), lin_vel_y=(-0.8, 0.8), ang_vel_z=(-1.5, 1.5), heading=(-math.pi, math.pi)),
        )

        # --- randomisation ------------------------------------------------------------------------------------
        # The sim's ground is 0.8 / 0.6 against the robot; train either side of it.
        self.events.physics_material.params["static_friction_range"] = (0.4, 1.2)
        self.events.physics_material.params["dynamic_friction_range"] = (0.3, 1.0)
        self.events.physics_material.params["make_consistent"] = True
        # The dock, the Jetson, the wrist camera and cabling are not in the model: +-1-2 kg covers them.
        self.events.add_base_mass.params["mass_distribution_params"] = (-1.0, 2.0)
        self.events.base_com = EventTerm(
            func=mdp.randomize_rigid_body_com, mode="startup",
            params={"asset_cfg": SceneEntityCfg("robot", body_names="base"),
                    "com_range": {"x": (-0.03, 0.03), "y": (-0.02, 0.02), "z": (-0.01, 0.01)}})
        self.events.add_base_mass.params["asset_cfg"] = SceneEntityCfg("robot", body_names="base")
        self.events.base_external_force_torque = None
        self.events.push_robot = EventTerm(
            func=mdp.push_by_setting_velocity, mode="interval", interval_range_s=(8.0, 12.0),
            params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}})
        # Legs only: scaling the arm's rest would push Joint2 and Joint3 through their limits.
        self.events.reset_robot_joints.params = {
            "position_range": (0.9, 1.1), "velocity_range": (-0.5, 0.5),
            "asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)}
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3), "yaw": (-math.pi, math.pi)},
            "velocity_range": {"x": (-0.2, 0.2), "y": (-0.2, 0.2), "z": (0.0, 0.0),
                               "roll": (-0.1, 0.1), "pitch": (-0.1, 0.1), "yaw": (-0.2, 0.2)},
        }


@configclass
class Go2KRailEnvCfg_PLAY(Go2KRailEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator.num_rows = 5
        self.scene.terrain.terrain_generator.num_cols = 10
        self.scene.terrain.terrain_generator.curriculum = False
        self.observations.policy.enable_corruption = False
        self.events.push_robot = None
        self.events.base_external_force_torque = None
        self.events.physics_material.params["static_friction_range"] = (0.8, 0.8)
        self.events.physics_material.params["dynamic_friction_range"] = (0.6, 0.6)
