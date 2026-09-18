"""The D1-550 arm as D1Training models it: limits, timing, the firmware's motion planner, and the wire.

Everything here is copied from Lukas's D1Training repository (commit 5e19028), where each number was measured on
the physical arm or fitted to recordings of it. The finding numbers (F-0xx) refer to D1Training's
`results/findings.md`. Keep this file identical in substance to its sources, so a policy trained there drives an
arm here that moves the same way:

    motor_model.py             effort and speed limits, interface timing, the fitted planner's parameters
    position_only/core.py      TrapezoidTracker (the firmware planner) and SampleAndHold (the 9 Hz feedback)
    d1_ik.py                   SERVO_SIGN: which servos turn opposite to their URDF joints
    d1_hardware.py             servo 6 (gripper) units, the measured ends of the jaw
    demos/cup/pick_demo        the gripper's shut travel and how the simulator lets the pads meet

No Isaac imports, so the tests run on a plain CPU interpreter with torch.
"""
from __future__ import annotations

import math

import torch

ARM_JOINTS = [f"Joint{i}" for i in range(1, 7)]
FINGER_JOINTS = ["Joint7_1", "Joint7_2"]

# ------------------------------------------------------------------------------------------- the drives
# Unitree D1-550 published joint torques: the first two joints 3.3 N*m, the last four 1.7.
EFFORT_LIMIT_NM = dict(zip(ARM_JOINTS, (3.3, 3.3, 1.7, 1.7, 1.7, 1.7)))
# MEASURED (F-033): peak rate of a 30 deg step on each joint through funcode 2 mode 0. One controller-wide
# ceiling, not the URDF's per-joint 1.05/1.73 (which had no source). Lower bounds, from 9 Hz samples.
VELOCITY_LIMIT_RAD_S = dict(zip(ARM_JOINTS, (1.25, 1.29, 1.23, 1.21, 1.25, 1.25)))
# D1Training flat_env_cfg: a stiff force drive stands in for the servo's own position loop; the effort limit is
# what says what the arm cannot lift. 800 N*m/rad drooped 13 cm at the tool, 4000 drooped 3.3 cm.
ARM_STIFFNESS = 4000.0
ARM_DAMPING = 400.0
# The jaws: 4000 N/m reaches the URDF's 15 N at 3.75 mm, inside the stroke (200 N/m never did).
GRIPPER_STIFFNESS = 4000.0
GRIPPER_DAMPING = 400.0

# ------------------------------------------------------------------------------------------ the timing
# MEASURED (F-020): joint angles come back every 111 ms (8.999 Hz), quantised to 0.1 deg.
FEEDBACK_HZ = 9.0
FEEDBACK_QUANTUM_DEG = 0.1
# NOT measured: the rate the SDK samples and the VIP-Rescue bridge stream setpoints at.
COMMAND_HZ = 10.0

# FITTED (F-045) to the six recorded 30 deg single-joint steps and the F-035 streaming sweep.
COMMAND_DEAD_TIME_S = 0.010         # fitted range 0-10 ms
PLAN_ACCEL_RAD_S2 = 15.5
PLAN_DECEL_RAD_S2 = 17.4
# Fraction of planned speed a new setpoint keeps: 0, every setpoint restarts the plan from rest.
REPLAN_VELOCITY_RETENTION = 0.0
# MEASURED (F-031): funcode 2 mode 1 slews at 13.5 deg/s where mode 0 reaches 69.3, on the same joint and step.
# Only the speed is modelled; mode 1's 5 deg steady-state error is not.
MODE1_SPEED_FACTOR = 0.236 / 1.209


def interface_steps(policy_hz: float) -> dict:
    """D1Training's `interface_timing("estimated", ...)` for the arm: how many policy steps a command is held for
    and how many pass between feedback samples. At 50 Hz, 5 and 6 (100 ms and 120 ms)."""
    return {
        "command_hold_steps": round(policy_hz / COMMAND_HZ),
        "feedback_period_steps": round(policy_hz / FEEDBACK_HZ),
    }


# ---------------------------------------------------------------------------------------- the wire
# MEASURED (F-030, F-034): servos 0 and 3 turn opposite to their URDF joints; the rest match.
SERVO_SIGN = (-1.0, 1.0, 1.0, -1.0, 1.0, 1.0)

# Servo 6 is the gripper, in its own units. MEASURED (F-063): -19.8 is the pads touching, the arm follows commands
# down to it and clamps; +50.2 is the widest a command opens it. More units, wider jaw.
GRIPPER_UNITS_CLOSED = -19.8
GRIPPER_UNITS_OPEN = 50.2
# Per-finger travel (Joint7_1; Joint7_2 mirrors it) at the open end: the URDF's full stroke.
GRIPPER_OPEN_TRAVEL_M = 0.03
# The CAD pads stand 17.2 mm apart at zero travel. The real ones meet (F-063); D1Training's pick lets the
# simulated fingers travel past the URDF's stop to where the jaws shut to `PINCH_CLOSED_GAP_M` -- stated, not
# measured: the conservative reading of "almost fully" -- which is what servo 6's closed end means here.
CLOSED_GAP_M = 0.0172
PINCH_CLOSED_GAP_M = 0.002
GRIPPER_SHUT_TRAVEL_M = (PINCH_CLOSED_GAP_M - CLOSED_GAP_M) / 2.0    # -7.6 mm


def units_to_travel(units: float) -> float:
    """Servo 6 units -> per-finger travel (m). Linear between the two measured ends; the middle is assumed, as it
    is on the arm (F-063: nobody has put a ruler across the fingers)."""
    u = min(max(float(units), GRIPPER_UNITS_CLOSED), GRIPPER_UNITS_OPEN)
    fraction = (u - GRIPPER_UNITS_CLOSED) / (GRIPPER_UNITS_OPEN - GRIPPER_UNITS_CLOSED)
    return GRIPPER_SHUT_TRAVEL_M + fraction * (GRIPPER_OPEN_TRAVEL_M - GRIPPER_SHUT_TRAVEL_M)


def travel_to_units(travel_m: float) -> float:
    """Per-finger travel (m) -> servo 6 units, the inverse of `units_to_travel`."""
    fraction = (float(travel_m) - GRIPPER_SHUT_TRAVEL_M) / (GRIPPER_OPEN_TRAVEL_M - GRIPPER_SHUT_TRAVEL_M)
    fraction = min(max(fraction, 0.0), 1.0)
    return GRIPPER_UNITS_CLOSED + fraction * (GRIPPER_UNITS_OPEN - GRIPPER_UNITS_CLOSED)


def servo_to_joint_rad(servo_deg) -> list[float]:
    """Servo degrees (ids 0..5) -> URDF radians (Joint1..Joint6). SERVO_SIGN is its own inverse."""
    return [math.radians(float(a)) * s for a, s in zip(servo_deg, SERVO_SIGN)]


def joint_to_servo_deg(q_rad) -> list[float]:
    return [math.degrees(float(q)) * s for q, s in zip(q_rad, SERVO_SIGN)]


def quantise(value_deg: float, quantum: float = FEEDBACK_QUANTUM_DEG) -> float:
    return round(float(value_deg) / quantum) * quantum


# ----------------------------------------------------------------------------------------- the rest
# MEASURED (F-023): the folded arm at rest reports [2.1, -90.9, 91.7, -3.1, 4.1, 3.1] deg on servos 0-5 and 40.8 on
# the gripper, stable to 0.2 deg over 120 s. Servos 1 and 2 sit 0.9 and 1.7 deg past the URDF's +-89.95 deg, so the
# simulated arm rests on those limits instead. This is the arm "collapsed": upper arm laid back along the dog, the
# forearm folded forward over it, the wrist 22 cm above its base.
REST_SERVO_DEG = (2.1, -90.9, 91.7, -3.1, 4.1, 3.1)
REST_GRIPPER_UNITS = 40.8


# The rest pose sits this far inside a limit it would otherwise lie on: the URDF's limits reach PhysX as float32, and
# Isaac Lab refuses a default position a rounding error outside them.
REST_LIMIT_MARGIN_RAD = 1e-3


def rest_joint_rad(lower_rad, upper_rad) -> list[float]:
    """F-023's rest pose in URDF radians, clamped into the given (hard) joint limits."""
    q = servo_to_joint_rad(REST_SERVO_DEG)
    return [min(max(v, lo + REST_LIMIT_MARGIN_RAD), hi - REST_LIMIT_MARGIN_RAD)
            for v, lo, hi in zip(q, lower_rad, upper_rad)]


# ------------------------------------------------------------------------------------ the planner
class SampleAndHold:
    """Per-environment sample-and-hold of a signal every `period` steps, at a random phase.

    Models an interface slower than the policy (the D1's 10 Hz feedback and commands under a 50 Hz
    policy). Each environment samples when (step + phase) % period == 0; `rate` is the change between
    consecutive samples divided by the sample interval, which is how velocity is recovered from an
    angle-only feed. `update` is idempotent within a step, so recomputing observations is harmless.

    Verbatim from D1Training `position_only/core.py`.
    """

    def __init__(self, num_envs: int, dim: int, period: int, sample_dt: float, device="cpu"):
        if period < 1:
            raise ValueError("Sample period must be at least one step.")
        self.period, self.sample_dt = period, sample_dt
        self.value = torch.zeros(num_envs, dim, device=device)
        self.rate = torch.zeros(num_envs, dim, device=device)
        self.phase = torch.zeros(num_envs, dtype=torch.long, device=device)
        self.last_step = torch.full((num_envs,), -1, dtype=torch.long, device=device)
        self.primed = torch.zeros(num_envs, dtype=torch.bool, device=device)

    def reset(self, env_ids, value: torch.Tensor, generator: torch.Generator | None = None):
        """Hold `value` (rows for `env_ids`) with zero rate until the first sample; draw new phases."""
        ids = slice(None) if env_ids is None else env_ids
        count = self.value[ids].shape[0]
        self.value[ids] = value
        self.rate[ids] = 0.0
        self.phase[ids] = torch.randint(0, self.period, (count,), device=self.phase.device, generator=generator)
        self.last_step[ids] = -1
        self.primed[ids] = False

    def update(self, step: torch.Tensor, signal: torch.Tensor) -> torch.Tensor:
        """Sample `signal` where due at `step` (per-environment episode step). Returns the mask sampled."""
        due = ((step + self.phase) % self.period == 0) & (step != self.last_step)
        fresh = due & self.primed
        self.rate[fresh] = (signal[fresh] - self.value[fresh]) / (self.period * self.sample_dt)
        self.rate[due & ~self.primed] = 0.0
        self.value[due] = signal[due]
        self.last_step[due] = step[due]
        self.primed |= due
        return due


class TrapezoidTracker:
    """The D1 firmware's motion planner: rest-to-rest trapezoids toward each commanded angle.

    The D1 does not snap to a setpoint. Given a joint angle it plans a trapezoid -- accelerate, cruise
    at its speed ceiling, decelerate -- and its servo loop follows the plan. Fitted to the six recorded
    30 deg single-joint steps (F-033 sweeps, 12 legs, 234 samples; `arm_response.py`), that plan
    reaches cruise in about a tenth of a second and stops a little harder than it starts.

    Two behaviours the simulation otherwise lacks:

    - **Finite acceleration.** PhysX's stiff position drive is limited only by torque, so the simulated
      arm reaches its speed ceiling almost instantly. The plan here limits the *target* instead, and
      the drive follows it.
    - **Lossy replanning.** A new setpoint mid-motion does not continue the old plan at its current
      speed. Streaming a waypoint every feedback cycle (F-035) covered 4.9-5.3 deg per 111 ms where a
      velocity-preserving planner gives 7.8; restarting from rest reproduces the hardware.
      `retention` is the fraction of planned velocity a new setpoint keeps (0 = restart from rest).

    `dead_time` delays each setpoint before it replaces the goal; the latest pending setpoint wins, as
    a firmware with one setpoint register would behave. Everything is per environment and per joint;
    `accel`, `decel` and `vmax` broadcast over the last dimension.

    Braking is exact in discrete time: each step's speed is capped at the largest from which the joint
    can still stop on its goal, so the plan lands on the goal at the physics rate without creeping up
    to it, passing it, or chattering about it.

    Verbatim from D1Training `position_only/core.py`, plus `vmax` being settable per command (funcode 2 mode 1).
    """

    def __init__(self, num_envs: int, dim: int, accel, decel, vmax, dead_time: float = 0.0,
                 retention: float = 0.0, device="cpu"):
        if not 0.0 <= retention <= 1.0:
            raise ValueError(f"Velocity retention must be in [0, 1], got {retention}")
        if dead_time < 0.0:
            raise ValueError(f"Dead time cannot be negative, got {dead_time}")
        as_row = lambda v: torch.as_tensor(v, dtype=torch.float32, device=device).expand(dim).clone()
        self.accel, self.decel, self.vmax = as_row(accel), as_row(decel), as_row(vmax)
        if (self.accel <= 0).any() or (self.decel <= 0).any() or (self.vmax <= 0).any():
            raise ValueError("Acceleration, deceleration and speed limits must be positive.")
        self.dead_time, self.retention = float(dead_time), float(retention)
        self.position = torch.zeros(num_envs, dim, device=device)
        self.velocity = torch.zeros(num_envs, dim, device=device)
        self.goal = torch.zeros(num_envs, dim, device=device)
        self.pending = torch.zeros(num_envs, dim, device=device)
        self.timer = torch.full((num_envs,), -1.0, device=device)

    def reset(self, env_ids, position: torch.Tensor):
        """Park at `position` (rows for `env_ids`) at rest, with no pending setpoint."""
        ids = slice(None) if env_ids is None else env_ids
        self.position[ids] = position
        self.goal[ids] = position
        self.pending[ids] = position
        self.velocity[ids] = 0.0
        self.timer[ids] = -1.0

    def command(self, mask: torch.Tensor, target: torch.Tensor):
        """Send `target` to the environments in `mask`; it takes effect after the dead time."""
        if not mask.any():
            return
        self.pending[mask] = target[mask]
        self.timer[mask] = self.dead_time
        if self.dead_time == 0.0:
            self._activate(mask)

    def _activate(self, mask):
        self.goal[mask] = self.pending[mask]
        self.velocity[mask] *= self.retention
        self.timer[mask] = -1.0

    def step(self, dt: float) -> torch.Tensor:
        """Advance the plan by `dt` and return the planned position."""
        waiting = self.timer >= 0.0
        if waiting.any():
            self.timer[waiting] -= dt
            due = waiting & (self.timer <= 1e-9)
            if due.any():
                self._activate(due)

        error = self.goal - self.position
        direction = torch.sign(error)
        distance = error.abs()
        heading = self.velocity * direction          # speed toward the goal; negative = moving away

        # Toward the goal (or at rest): the largest speed this step from which the joint can still
        # stop on the goal, u*dt + u^2/(2*decel) <= distance, solved for u. Accelerating is capped by
        # it, so braking begins exactly when it must and the plan lands on the goal without creeping
        # up to it or passing it.
        reachable = self.decel * (torch.sqrt(dt * dt + 2.0 * distance / self.decel) - dt)
        toward = torch.minimum(torch.minimum(heading.clamp(min=0.0) + self.accel * dt, self.vmax), reachable)
        # Away from the goal (a replan that kept speed and reversed): brake back through zero.
        away = heading + self.decel * dt
        speed = torch.where(heading < 0.0, torch.minimum(away, torch.zeros_like(away)), toward)
        self.velocity = direction * speed

        step = self.velocity * dt
        arrive = (direction != 0) & (heading >= 0.0) & (step.abs() >= distance - 1e-9)
        self.position = torch.where(arrive, self.goal, self.position + step)
        self.velocity = torch.where(arrive | (direction == 0), torch.zeros_like(self.velocity), self.velocity)
        return self.position


def make_planner(num_envs: int = 1, device="cpu") -> TrapezoidTracker:
    """The planner D1Training's position-only task and pick scene run: the "measured" profile (F-045)."""
    return TrapezoidTracker(
        num_envs, len(ARM_JOINTS), PLAN_ACCEL_RAD_S2, PLAN_DECEL_RAD_S2,
        [VELOCITY_LIMIT_RAD_S[j] for j in ARM_JOINTS], dead_time=COMMAND_DEAD_TIME_S,
        retention=REPLAN_VELOCITY_RETENTION, device=device)


def gripper_joint_limits(lower: float, upper: float, soft_factor: float) -> tuple[float, float]:
    """Joint7_1's (lower, upper) hard limits widened so its *soft* limit reaches the shut travel, as D1Training's
    `run_pick_demo.py` does. Isaac Lab clamps targets to soft limits a factor inside the hard ones, so the hard limit
    has to open further than the travel wanted, or the pads stop short by the margin nobody can see.

    Joint7_2 mirrors it: its limits are (-upper', -lower')."""
    span = upper - lower
    shut = min(0.0, GRIPPER_SHUT_TRAVEL_M)
    if shut >= 0.0:
        return lower, upper
    reach = (2 * (shut - 0.0005) - span * (1 - soft_factor)) / (1 + soft_factor)
    return min(lower, reach), upper
