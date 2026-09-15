# SHORT FINE-TUNE EXPERIMENTS, 2026-09-14. One hypothesis per arm, judged by
# training/measure_bench.py on the sim's own (nominal) robot. Not a deployable task: nothing in
# the sim references these, and the arms that lose get deleted.
#
# WHAT THE BASELINE MEASURED (model_7650, the last spike-free checkpoint of run 3):
#   * on the robot the SIM drives -- no base-mass or CoM randomisation -- forward tracking at
#     0.5 m/s is 80 %; training's CoM range is 0..+5 cm FORWARD, so that robot is at the very
#     edge of what the policy trained on, and forward speed follows the offset almost linearly
#     (103 % at +2.5 cm, 116 % at +5).
#   * it does not climb the maze's stairs: 0 % of robots onto the first step at 0.5 m/s, and at
#     1.0 m/s the robots mount the first step, fail, back off and turn away. P2Dingo's policy
#     fails the same way. Run 3's terrain curriculum sat at level ~1.1 of 10 (a ~6 cm riser).
#   * the feet gather under the body: front feet land 10.4 cm behind their hips, rear 17.8 cm
#     ahead, where P2Dingo's land within 1-2 cm of Raibert's neutral point.
#
# THE LADDER. Every arm resumes model_7650 and runs the same number of iterations, so the
# control measures what a fine-tune does by itself (the curriculum restarts on resume).
#
#   control     nothing changed
#   com         CoM x randomised symmetrically about the nominal robot, -3..+3 cm
#   riser       com + every stair tile's riser within 8-12 cm, so the maze's 10 cm is
#               practised at every curriculum level rather than from level ~6 up
#   stairfwd    riser + half the commands on stair tiles walk straight ahead at 0.3-1.0 m/s
#   placement   com + a gentle Raibert touchdown penalty (mdp.FootPlacementPenalty)
#   thigh       com + thigh deviations halved in the posture term while moving
#
# The CoM change is carried into every later arm because the benchmark measures the nominal
# robot: without it, each arm would be graded on a robot at the edge of its training range.
#
# ALL ARMS CLIP ACTIONS AT +-20 (agents/rsl_rl_ppo_cfg.py, Go2RescueExpPPORunnerCfg). The first
# control run blew up within 155 iterations exactly as run 3 did; the docstring there has the
# mechanism. measure_bench.py applies the same clip, so arms and baseline are graded alike.
#
# ALL ARMS start at the full command range (the resumed policy already had it) instead of
# re-walking the linear-velocity curriculum from +-0.3 m/s, which would spend the first
# ~600 iterations of a 1000-iteration run below the speeds being measured.

import copy

from isaaclab.managers import RewardTermCfg, SceneEntityCfg
from isaaclab.utils import configclass

from . import mdp as rescue_mdp
from .rough_env_cfg import UnitreeGo2RescueRoughEnvCfg

FEET = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
THIGHS = ["FL_thigh", "FR_thigh", "RL_thigh", "RR_thigh"]


@configclass
class ExpControlEnvCfg(UnitreeGo2RescueRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.commands.base_velocity.ranges = copy.deepcopy(self.commands.base_velocity.limit_ranges)
        self.curriculum.lin_vel_cmd_levels = None


@configclass
class ExpComEnvCfg(ExpControlEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.events.base_com.params["com_range"] = {"x": (-0.03, 0.03), "y": (-0.02, 0.02), "z": (-0.02, 0.02)}


@configclass
class ExpRiserEnvCfg(ExpComEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        for sub in self.scene.terrain.terrain_generator.sub_terrains.values():
            if hasattr(sub, "step_height_range"):
                sub.step_height_range = (0.08, 0.12)


@configclass
class ExpStairFwdEnvCfg(ExpRiserEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        old = self.commands.base_velocity
        new = rescue_mdp.StairForwardVelocityCommandCfg(
            asset_name=old.asset_name,
            resampling_time_range=old.resampling_time_range,
            heading_command=old.heading_command,
            debug_vis=old.debug_vis,
            ranges=copy.deepcopy(old.ranges),
            limit_ranges=copy.deepcopy(old.limit_ranges),
            rel_standing_envs=old.rel_standing_envs,
            rel_turning_envs=old.rel_turning_envs,
            rel_stair_forward=0.5,
            stair_forward_speed=(0.3, 1.0),
        )
        self.commands.base_velocity = new


@configclass
class ExpPlacementEnvCfg(ExpComEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # weight -1.0: at today's ~14 cm error this is ~-0.8 per second of episode, the size of
        # `joint_pos` (-0.81) and `hip_deviation` (-0.78) in run 3's logs, and it shrinks as the
        # placement improves. `gait` is +7.3 and `base_linear_velocity` +3.1, so it cannot outvote
        # either.
        self.rewards.foot_placement = RewardTermCfg(
            func=rescue_mdp.FootPlacementPenalty,
            weight=-1.0,
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=FEET, preserve_order=True),
                "thigh_cfg": SceneEntityCfg("robot", body_names=THIGHS, preserve_order=True),
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FEET, preserve_order=True),
                "nominal_dx": [-0.016, -0.016, -0.077, -0.077],
                "stance_time": 0.25,
                "full_speed": 0.5,
                "max_err": 0.20,
            },
        )


@configclass
class ExpThighEnvCfg(ExpComEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        jp = self.rewards.joint_pos
        self.rewards.joint_pos = RewardTermCfg(
            func=rescue_mdp.joint_position_penalty_thigh_relaxed,
            weight=jp.weight,
            params={**jp.params, "moving_thigh_scale": 0.5},
        )



@configclass
class ExpPlacementW30EnvCfg(ExpPlacementEnvCfg):
    """The placement arm at the weight it was meant to have.

    The first placement arm (weight -1.0) logged Episode_Reward/foot_placement at -0.024, not the
    ~-0.8 it was sized for. Isaac Lab logs the per-STEP mean of a term, and this term is an event:
    ~8 touchdowns a second is ~0.16 per 50 Hz step, so its per-step mean is ~35x smaller than the
    per-touchdown error the sizing used. It never pushed on anything, and its placement numbers
    duly did not move. -30 puts it where the comment above intended: beside joint_pos (-0.81) and
    hip_deviation (-0.78).
    """

    def __post_init__(self):
        super().__post_init__()
        self.rewards.foot_placement.weight = -30.0



# ----------------------------------------------------------------------------------------------
# FLAT ABLATIONS (2026-09-14, evening). The maze is flat now and the first flat run with the arm
# (go2_rescue_flat/2026-09-14_21-06-31_flat_arm) came out well short of P2Dingo's flat policy on
# measure_bench.py -- turn tracking 76-83 % against 104 %, feet gathered under the body (front
# -6.9 cm, rear +16.9 cm against P2Dingo's +7.1 / +5.5), backward motion after a forward request
# -- even though P2Dingo's policy is benchmarked on this same welded, measured-motor robot. So the
# fault is in what Rescue's task added, and on flat ground that is five things. Each arm removes
# ONE from the Rescue flat task; FlatP2D removes all of them and must reproduce P2Dingo, or the
# list is incomplete.
# ----------------------------------------------------------------------------------------------

from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG as _UNITREE_GO2_CFG

from .flat_env_cfg import UnitreeGo2RescueFlatEnvCfg
from .welded_robot import scope_to_legs


def _strip_mairo_randomisation(cfg):
    cfg.events.physics_material.params.update({
        "static_friction_range": (0.8, 0.8), "dynamic_friction_range": (0.6, 0.6),
        "restitution_range": (0.0, 0.0), "make_consistent": False})
    cfg.events.push_robot = None
    cfg.events.reset_robot_joints.params["velocity_range"] = (0.0, 0.0)
    cfg.terminations.bad_orientation = None


def _bare_go2(cfg, measured_actuator: bool):
    robot = _UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    if measured_actuator:
        from go2_actuators import apply_go2_actuator  # on sys.path once build_welded_go2_cfg has run
        robot = robot.copy()
        apply_go2_actuator(robot)
    cfg.scene.robot = robot
    scope_to_legs(cfg)


@configclass
class FlatNoDREnvCfg(UnitreeGo2RescueFlatEnvCfg):
    """Rescue flat minus MaiRo's randomisation: friction range, shoves, joint-velocity reset, tip-over termination."""

    def __post_init__(self):
        super().__post_init__()
        _strip_mairo_randomisation(self)


@configclass
class FlatNoArmEnvCfg(UnitreeGo2RescueFlatEnvCfg):
    """Rescue flat on the bare Go2 (measured motor kept)."""

    def __post_init__(self):
        super().__post_init__()
        _bare_go2(self, measured_actuator=True)


@configclass
class FlatNoActEnvCfg(UnitreeGo2RescueFlatEnvCfg):
    """Rescue flat with the arm, on Isaac Lab's stock Go2 motor model instead of the measured curve."""

    def __post_init__(self):
        super().__post_init__()
        # The weld is already built by the parent (building it twice in one process fails: the
        # USD layer exists), so keep that robot and put the stock leg motor group back.
        robot = self.scene.robot.copy()
        robot.actuators = dict(robot.actuators)
        robot.actuators["base_legs"] = copy.deepcopy(_UNITREE_GO2_CFG.actuators["base_legs"])
        self.scene.robot = robot


@configclass
class FlatClear08EnvCfg(UnitreeGo2RescueFlatEnvCfg):
    """Rescue flat with P2Dingo's 8 cm swing-clearance target instead of 10 cm."""

    def __post_init__(self):
        super().__post_init__()
        self.rewards.foot_clearance.params["target_height"] = 0.08


@configclass
class FlatP2DEnvCfg(UnitreeGo2RescueFlatEnvCfg):
    """All five removed: should reproduce P2Dingo's flat task (the action clip stays; it never binds)."""

    def __post_init__(self):
        super().__post_init__()
        _strip_mairo_randomisation(self)
        _bare_go2(self, measured_actuator=False)
        self.rewards.foot_clearance.params["target_height"] = 0.08
        self.events.base_com.params["com_range"] = {"x": (0.0, 0.05), "y": (-0.02, 0.02), "z": (-0.02, 0.02)}
