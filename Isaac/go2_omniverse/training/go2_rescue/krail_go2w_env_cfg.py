"""The K-Rail Square policy for the Go2-W: the Go2's task (`krail_env_cfg.py`) on the wheeled robot.

Task ids (`__init__.py`): `Isaac-Velocity-Rescue-KRail-Go2W-v0` (train) and `...-Play-v0`; runs log under
`go2w_krail`. Exported with `../export_krail.py` like the Go2's, as a bundle the sim walks the Go2-W with.

THE SAME TASK, ON PURPOSE. The ground (decks 3-24 cm with K-rails, stairs, boxes, rough, slopes, flat), the 5 cm
height scan, the command generator and its terrain ladder, the mirror training, the noise, the randomisation and
every reward term and weight are the Go2's final (v3e) task's, whose README section records why each is there. The
Go2's run got there in four segments resumed one from the next; this starts from scratch with all of it on.

WHAT THE WHEELS CHANGE, and nothing else:

* THE ROBOT is the sim's Go2-W (`../../go2w.py`) with the D1 welded at the Go2's mount, folded at its rest, built by
  the sim's own `rescue_sim.env_cfg.robot_cfg`: 22.7 kg, standing 0.41 m tall on wheels 0.172 m across.
* THE ACTION is the twelve legs' position targets, as the Go2's (0.25 per unit about the standing pose), then the four
  wheels' velocity targets (`rescue_sim.env_cfg.wheel_action_cfg`: 5 rad/s per unit, 0.43 m/s at the rim).
* THE OBSERVATION adds the wheels' speeds after the legs' in `joint_vel` (16 wide), and the last action is 16 wide:
  48 + 8 + 693 = 749. The wheels' angles are not observed; a wheel's angle carries no information.
* SIDEWAYS COMMANDS TO 0.5 m/s, from the Go2's 0.8. A wheel does not roll sideways, so every sideways metre is
  stepped, and nav2 never asks for one (its DWB drives and turns; `../measure_square.py`'s driver never strafes).
  Forward, backward and turning keep the Go2's envelope, so the sim's keys and nav2 ask the same of either robot.
* NO AIR-TIME REWARD. The Go2's task pays 0.05 for each foot's swing past 0.3 s, which is a trot's; on wheels it
  would pay for stepping where rolling does. Nothing else is added in its place: driving costs the legs less torque
  and acceleration than stepping, which the leg terms already charge, and nothing charges an attempt at a step.
* THE STANCE TERMS' HEIGHTS: the base is charged under 0.25 m over its feet's mean height (the axles here: 0.32 in the
  standing pose, as the Go2's 0.26 is against its 0.33), and the tilt term reads a ray that misses as ground 0.41 m
  under the base, the Go2-W's standing height, where the Go2's reads 0.33.
"""
from __future__ import annotations

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from .krail_env_cfg import Go2KRailEnvCfg, Go2KRailEnvCfg_PLAY, sim_robot_cfg

# Isaac Lab's leg order, then the wheels in the same leg order: how `joint_vel` reads them (the sim builds the same,
# `rescue_sim.env_cfg.exported_observations`). Named with preserve_order, so PhysX's parse order cannot change it.
LEG_ORDER = [
    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
]
WHEEL_ORDER = ["FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"]

LATERAL_COMMAND_MPS = 0.5
BASE_LOW_OVER_AXLES_M = 0.25
STANDING_HEIGHT_M = 0.41


def _go2w_post_init(cfg) -> None:
    """Everything the wheels change, applied to a Go2 K-rail cfg after its own `__post_init__`."""
    from rescue_sim.env_cfg import wheel_action_cfg   # importable now: the robot's build put the sim on sys.path

    cfg.actions.wheel_vel = wheel_action_cfg()
    joints = SceneEntityCfg("robot", joint_names=LEG_ORDER + WHEEL_ORDER, preserve_order=True)
    cfg.observations.policy.joint_vel.params = {"asset_cfg": joints}
    cfg.observations.critic.joint_vel.params = {"asset_cfg": joints.copy()}
    ranges = cfg.commands.base_velocity.ranges
    ranges.lin_vel_y = (-LATERAL_COMMAND_MPS, LATERAL_COMMAND_MPS)
    cfg.rewards.feet_air_time = None
    cfg.rewards.base_low.params["min_height"] = BASE_LOW_OVER_AXLES_M
    cfg.rewards.tilt_from_ground.params["miss_depth"] = STANDING_HEIGHT_M


@configclass
class Go2WKRailEnvCfg(Go2KRailEnvCfg):
    def _robot_cfg(self):
        return sim_robot_cfg("go2w")

    def __post_init__(self):
        super().__post_init__()
        _go2w_post_init(self)


@configclass
class Go2WKRailEnvCfg_PLAY(Go2KRailEnvCfg_PLAY):
    def _robot_cfg(self):
        return sim_robot_cfg("go2w")

    def __post_init__(self):
        super().__post_init__()
        _go2w_post_init(self)
