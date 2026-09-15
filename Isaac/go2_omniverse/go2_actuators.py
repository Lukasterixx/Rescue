"""The Go2's real motor, as a torque-speed curve instead of an ideal source.

WHY THIS EXISTS. Isaac Lab's stock Go2 drives every leg joint with `DCMotorCfg`: a straight
line from 23.5 N*m at zero speed to zero at 30 rad/s, the same in both directions, with no
joint friction. A real Go2 motor is not that shape. Unitree's measured curve is FLAT at its
peak torque up to a knee and only then falls away, it is STRONGER when braking than when
driving, and it has stiction. Measured numbers for the Go2's HV motor:

    X1 = 13.5 rad/s   knee: full torque is available up to here
    X2 = 30.0 rad/s   no-load speed: torque has fallen to zero
    Y1 = 20.2 N*m     peak torque DRIVING (torque and speed the same direction)
    Y2 = 23.4 N*m     peak torque BRAKING (torque opposing the motion)

The differences are not small, and they do not go the same way at both ends. Computed from
both models (stock numbers: saturation and effort limit 23.5 N*m, velocity limit 30 rad/s):

    joint speed      stock drive / brake      this model, drive / brake
      0 rad/s           23.5 / 23.5                20.2 / 23.4
      8                 17.2 / 23.5                20.2 / 23.4
     13.5 (knee)        12.9 / 23.5                20.2 / 23.4
     20                  7.8 / 23.5                12.2 / 14.2
     27                  2.3 / 23.5                 3.7 /  4.3
     30                  0.0 / 23.5                 0.0 /  0.0

Two errors, in opposite directions. Driving, the stock model UNDERSTATES the motor through
the whole mid-range -- 12.9 N*m against the real 20.2 at the knee -- so a policy trained on
it learns to swing a leg with two thirds of the torque the robot actually has. BRAKING, the
stock model is a flat 23.5 N*m at every speed, because `torque_speed_bottom` is clipped by
`effort_limit` rather than by the curve, and the real motor gives 4.3 N*m at 27 rad/s. That
second one is the dangerous half: catching the body on a step edge is a braking action at
high joint speed, and it is exactly where the stock model is most optimistic.

Ported from MaiRo's `unitree_rl_lab` (mairo-rl-lab-rinam,
source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree_actuators.py), which is a
Unitree-derived stack with demonstrated sim-to-real transfer on this robot. The class is
theirs; the numbers are Unitree's.

BOTH SIDES MUST USE IT. A policy trained against this motor and played back against the
stock one is being run on a different robot -- exactly the mismatch this is meant to remove.
`custom_rl_env.py` (the sim) and `training/go2_rescue/` (training) both call
`apply_go2_actuator()`, which is why this module sits beside the sim rather than inside the
training package.

    Torque limit, N*m
        ^
     Y2 |----------
        |----------\\------  Y1
        |          | \\
        |          |  \\
        +----------+---+----> joint speed, rad/s
                  X1   X2
"""

from __future__ import annotations

import torch

from isaaclab.actuators import DelayedPDActuator, DelayedPDActuatorCfg
from isaaclab.utils import configclass
from isaaclab.utils.types import ArticulationActions


class UnitreeActuator(DelayedPDActuator):
    """A PD actuator clipped by a measured torque-speed curve, with joint friction.

    Extends `DelayedPDActuator` rather than `DCMotor` for two reasons: the delay buffer
    comes free (see `min_delay`/`max_delay` on the cfg, and the note in
    `GO2_HV_ACTUATOR`), and `DCMotor`'s clipping is the straight-line model being
    replaced.
    """

    cfg: UnitreeActuatorCfg

    def __init__(self, cfg: UnitreeActuatorCfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)
        # `_clip_effort` needs the CURRENT joint velocity and is called from inside
        # `IdealPDActuator.compute`, which is not handed it. Stashing it in `compute`
        # before delegating upward is how the curve gets its x-axis.
        self._joint_vel = torch.zeros_like(self.computed_effort)
        self._effort_y1 = self._parse_joint_parameter(cfg.Y1, 1e9)
        self._effort_y2 = self._parse_joint_parameter(cfg.Y2, cfg.Y1)
        self._velocity_x1 = self._parse_joint_parameter(cfg.X1, 1e9)
        self._velocity_x2 = self._parse_joint_parameter(cfg.X2, 1e9)
        self._friction_static = self._parse_joint_parameter(cfg.Fs, 0.0)
        self._friction_dynamic = self._parse_joint_parameter(cfg.Fd, 0.0)
        self._activation_vel = self._parse_joint_parameter(cfg.Va, 0.01)

    def compute(
        self, control_action: ArticulationActions, joint_pos: torch.Tensor, joint_vel: torch.Tensor
    ) -> ArticulationActions:
        self._joint_vel[:] = joint_vel
        control_action = super().compute(control_action, joint_pos, joint_vel)
        # Coulomb plus viscous friction, subtracted AFTER clipping: stiction is a loss at
        # the joint, not a reduction in what the motor can produce. `tanh(v / Va)` is a
        # smooth sign() so the static term does not chatter around zero velocity.
        self.applied_effort -= (
            self._friction_static * torch.tanh(joint_vel / self._activation_vel)
            + self._friction_dynamic * joint_vel
        )
        control_action.joint_positions = None
        control_action.joint_velocities = None
        control_action.joint_efforts = self.applied_effort
        return control_action

    def _clip_effort(self, effort: torch.Tensor) -> torch.Tensor:
        # Driving or braking? The limit is higher when the motor opposes the motion.
        same_direction = (self._joint_vel * effort) > 0
        max_effort = torch.where(same_direction, self._effort_y1, self._effort_y2)
        # Flat below the knee, falling linearly above it.
        max_effort = torch.where(
            self._joint_vel.abs() < self._velocity_x1, max_effort, self._compute_effort_limit(max_effort)
        )
        return torch.clip(effort, -max_effort, max_effort)

    def _compute_effort_limit(self, max_effort: torch.Tensor) -> torch.Tensor:
        k = -max_effort / (self._velocity_x2 - self._velocity_x1)
        limit = k * (self._joint_vel.abs() - self._velocity_x1) + max_effort
        return limit.clip(min=0.0)


@configclass
class UnitreeActuatorCfg(DelayedPDActuatorCfg):
    """Configuration for a Unitree actuator described by its torque-speed curve."""

    class_type: type = UnitreeActuator

    X1: float = 1e9
    """Maximum speed at full torque, i.e. the knee of the curve. rad/s."""
    X2: float = 1e9
    """No-load speed: torque reaches zero here. rad/s."""
    Y1: float = 20.2
    """Peak torque while DRIVING (torque and speed in the same direction). N*m."""
    Y2: float | None = None
    """Peak torque while BRAKING (torque opposing the speed). N*m. Defaults to Y1."""
    Fs: float = 0.0
    """Static (Coulomb) friction, N*m."""
    Fd: float = 0.0
    """Dynamic (viscous) friction, N*m per rad/s."""
    Va: float = 0.01
    """Velocity at which static friction is fully developed. rad/s."""


def go2_hv_actuator_cfg(joint_names_expr: list[str]) -> UnitreeActuatorCfg:
    """The Go2 HV motor, for the given joints. Gains are the stock Go2's, unchanged.

    THE DELAY IS ZERO, WHICH MATCHES MaiRo AND MATCHES THIS SIM. `min_delay`/`max_delay`
    are in PHYSICS steps (5 ms), and a real 50 Hz controller never acts on the state it
    just read -- so a deployment-bound policy wants 1-4 here. Rescue's sim applies the
    policy with no delay, so training with one would train against a robot the sim does
    not simulate. Raise it in the same edit that adds a delay to deployment, not before.
    """
    return UnitreeActuatorCfg(
        joint_names_expr=list(joint_names_expr),
        # Unitree's measured Go2 HV curve.
        X1=13.5,
        X2=30.0,
        Y1=20.2,
        Y2=23.4,
        # The stock Go2's PD gains, and the same ones the deployed controller uses.
        stiffness=25.0,
        damping=0.5,
        # PhysX joint friction. In Isaac Sim 5.x this is an effort (N*m), not a
        # coefficient. MaiRo's value; the stock Go2 runs 0.0.
        friction=0.01,
        min_delay=0,
        max_delay=0,
    )


# The stock Go2 asset's actuator group name. Isaac Lab's `UNITREE_GO2_CFG` puts all twelve
# leg joints in one group under this key.
GO2_LEG_ACTUATOR_GROUP = "base_legs"


def apply_go2_actuator(robot_cfg, group: str = GO2_LEG_ACTUATOR_GROUP):
    """Swap a Go2 articulation cfg's leg actuator group for the measured motor, in place.

    Takes the joint regexes from the group it replaces, so it cannot widen the group's
    scope -- on the welded Go2+D1 the legs stay the legs and the arm keeps its own
    `ImplicitActuatorCfg` drives. Raises if the group is missing, because silently
    leaving the stock motor in place is the failure this is meant to prevent.
    """
    actuators = dict(robot_cfg.actuators)
    if group not in actuators:
        raise KeyError(
            f"[go2_actuators] No actuator group '{group}' on this robot; found "
            f"{sorted(actuators)}. The Go2 asset must have changed -- check which group "
            f"holds the leg joints before editing this."
        )
    joint_names_expr = list(actuators[group].joint_names_expr)
    actuators[group] = go2_hv_actuator_cfg(joint_names_expr)
    robot_cfg.actuators = actuators
    print(f"[go2_actuators] leg group '{group}' -> Unitree Go2 HV curve (Y1 20.2 / Y2 23.4 N*m, knee 13.5 rad/s)")
    return robot_cfg
