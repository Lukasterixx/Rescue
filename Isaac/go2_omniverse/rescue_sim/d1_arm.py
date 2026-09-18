"""The simulated D1: its firmware on the simulator's clock, and the DDS wire it speaks.

The sim runs no arm controller of its own. The arm sits where it was last told to be -- folded at the measured
rest pose until something says otherwise -- and anything that wants it elsewhere talks to it the way it would
talk to the real arm: `rt/arm_Command` in, `current_servo_angle` and `rt/arm_Feedback` out, over CycloneDDS on
domain 0. In the VIP-Rescue stack that is `maps/arm_bridge.py` (run with `--sim`), which the behaviour-tree nodes
reach over its TCP socket, exactly as on the robot.

What the wire does to the arm follows D1Training's model of the real one (`d1_model.py`):

* **Commands are held at 10 Hz.** A setpoint reaches the planner at most once per `command_hold_steps` policy
  steps (100 ms), the latest one winning, however fast the sender streams. D1Training's action term passes the
  policy's setpoint on at the same rate; here the rate limit also keeps a sim running slower than real time from
  seeing more setpoints per simulated second than the arm ever would.
* **The firmware plans each move** (`TrapezoidTracker`, F-045): 10 ms dead time, then a trapezoid at 15.5 rad/s^2
  up to each joint's measured ceiling and 17.4 rad/s^2 down; every new setpoint restarts from rest. Its position
  and velocity are the drive's targets every physics step.
* **Feedback is sampled every `feedback_period_steps`** (120 ms at a 50 Hz policy, D1Training's discretisation of
  the measured 111 ms, F-020) at a random phase, quantised to 0.1 deg, in servo space.
* **Servo space is the real arm's**: servos 0 and 3 turn opposite to Joint1 and Joint4 (F-030, F-034), and servo 6
  is the gripper in its own units, -19.8 with the pads touching to 50.2 wide open (F-063).
* **funcode 5 {"mode": 0} releases the arm**: the drives go limp and it falls, as the real one does (F-028). The
  next motion command energises it again from wherever it came to rest. {"mode": 1} does nothing, and neither does
  funcode 6 (power): on the real arm enable is implicit and power-off is ignored (D1Training `d1_hardware.py`).
* **funcode 7 returns to zero** through the same planner. The zero pose is the arm standing straight up, not its
  rest.
* **A level load parks the arm at rest**, and the target it was holding stops counting: a sender that keeps streaming
  it (the bridge holds and repeats its last target) does not pull the arm back out, and the arm stays folded until it
  is sent somewhere new. The real arm is never teleported, so it never needs this.

`D1Firmware` is pure logic on torch tensors, so the tests drive it on a CPU with no Isaac and no DDS.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math

import torch

from . import d1_model as m

# Wire constants. d1_sdk/d1_protocol.py has the full table; these are the ones the firmware acts on.
ADDR_COMMAND, ADDR_STATE, ADDR_ACK = 1, 2, 3
FUNC_SET_ANGLE, FUNC_SET_ALL_ANGLES, FUNC_SET_DAMPING, FUNC_ENABLE_ALL, FUNC_POWER, FUNC_ZERO = 1, 2, 4, 5, 6, 7
FUNC_STATE_ANGLES, FUNC_STATE_STATUS = 1, 3
FUNC_ACK_RECV, FUNC_ACK_EXEC = 1, 2
FEEDBACK_SEQ = 10            # the arm stamps every cyclic upload with this sequence number
STATUS_PERIOD_S = 0.1005     # MEASURED (F-020): the funcode 3 status stream, not the angles


@dataclass
class Feedback:
    """One feedback sample, as it goes on the wire."""

    servo_deg: list[float]       # servos 0..5, quantised
    gripper_units: float         # servo 6, quantised


class D1Firmware:
    """The D1's onboard controller, stepped by the simulator.

    Call order each policy step: `begin_step()` before the physics substeps, `physics_step(dt)` in each of them
    (it returns the drive targets), then `end_step(q, travel)` with the measured state, which returns a
    `Feedback` when one is due. `receive(frame)` may be called any time before `begin_step()`.
    """

    def __init__(self, policy_dt: float, lower_rad, upper_rad, rest_q, rest_travel_m: float, seed: int = 0,
                 device="cpu"):
        timing = m.interface_steps(1.0 / policy_dt)
        self.hold_steps = timing["command_hold_steps"]
        self.feedback_period_steps = timing["feedback_period_steps"]
        self.policy_dt = float(policy_dt)
        self.device = device
        self.lower = torch.tensor([list(lower_rad)], dtype=torch.float32, device=device)
        self.upper = torch.tensor([list(upper_rad)], dtype=torch.float32, device=device)
        self.planner = m.make_planner(1, device)
        self._vmax = self.planner.vmax.clone()
        self._generator = torch.Generator(device="cpu").manual_seed(int(seed))
        self.feedback = m.SampleAndHold(1, 7, self.feedback_period_steps, self.policy_dt, "cpu")
        self.step = 0
        self.energised = True
        self.commands_received = 0
        self.commands_issued = 0
        self._pending_q: torch.Tensor | None = None
        self._pending_speed = 1.0
        self._pending_gripper: float | None = None
        self._last_issue_step = -10 ** 9
        self._status_timer = 0.0
        self._last_received = None      # (q, gripper) of the last motion command
        self._stale = None              # a pre-park target, ignored while it is merely repeated
        self.goal_q = torch.zeros(1, 6, device=device)
        self.gripper_target_m = float(rest_travel_m)
        self.park(rest_q, rest_travel_m)

    # ------------------------------------------------------------------------------------------ state
    def park(self, q_rad, travel_m: float) -> None:
        """Hold still at `q_rad` with nothing pending: startup, and a level reset that moved the arm."""
        q = torch.as_tensor(list(q_rad), dtype=torch.float32, device=self.device).reshape(1, 6)
        self._stale = self._last_received
        self.planner.reset(None, q)
        self.goal_q = q.clone()
        self.gripper_target_m = float(travel_m)
        self._pending_q, self._pending_gripper = None, None
        self._last_issue_step = -10 ** 9
        self.energised = True
        signal = torch.cat([q.cpu(), torch.tensor([[float(travel_m)]])], dim=1)
        self.feedback.reset(None, signal, generator=self._generator)

    # ------------------------------------------------------------------------------------------ the wire
    def receive(self, frame: dict) -> bool:
        """Act on one decoded `rt/arm_Command` frame. Returns the exec status the arm acks with."""
        if frame.get("address") != ADDR_COMMAND:
            return False
        self.commands_received += 1
        funcode, data = frame.get("funcode"), frame.get("data") or {}
        try:
            if funcode == FUNC_SET_ALL_ANGLES:
                base = self._pending_q if self._pending_q is not None else self.goal_q
                q = base.clone()
                for i in range(6):
                    key = f"angle{i}"
                    if key in data:
                        q[0, i] = math.radians(float(data[key])) * m.SERVO_SIGN[i]
                q = torch.clamp(q, self.lower, self.upper)
                gripper = m.units_to_travel(float(data["angle6"])) if "angle6" in data else None
                self._last_received = (q.clone(), gripper)
                if self._repeats_stale(q, gripper):
                    return True
                self._stale = None
                self._set_pending(q, m.MODE1_SPEED_FACTOR if int(data.get("mode", 0)) == 1 else 1.0)
                if gripper is not None:
                    self._pending_gripper = gripper
                return True
            if funcode == FUNC_SET_ANGLE:
                joint = int(data.get("id", -1))
                angle = float(data.get("angle", 0.0))
                if joint == 6:
                    self._pending_gripper = m.units_to_travel(angle)
                    return True
                if 0 <= joint < 6:
                    base = self._pending_q if self._pending_q is not None else self.goal_q
                    q = base.clone()
                    q[0, joint] = math.radians(angle) * m.SERVO_SIGN[joint]
                    self._set_pending(q, 1.0)
                    return True
                return False
            if funcode == FUNC_ENABLE_ALL:
                if int(data.get("mode", 1)) == 0:
                    self.energised = False       # release: the drives go limp (F-028)
                return True                      # mode 1 is a no-op on the arm: enable is implicit
            if funcode == FUNC_ZERO:
                self._set_pending(torch.zeros(1, 6, device=self.device), 1.0)
                return True
            if funcode in (FUNC_POWER, FUNC_SET_DAMPING):
                return True                      # accepted and, as on the arm (power) or unmeasured (damping), ignored
        except (TypeError, ValueError):
            return False
        return False

    def _repeats_stale(self, q: torch.Tensor, gripper) -> bool:
        """This command is the target from before the last park, sent again: not a new instruction."""
        if self._stale is None:
            return False
        stale_q, stale_gripper = self._stale
        same_q = float((q - stale_q).abs().max()) < 1e-4
        same_gripper = gripper is None or stale_gripper is None or abs(gripper - stale_gripper) < 1e-6
        return same_q and same_gripper

    def _set_pending(self, q: torch.Tensor, speed: float) -> None:
        self._pending_q = torch.clamp(q, self.lower, self.upper)
        self._pending_speed = speed

    # ------------------------------------------------------------------------------------------ the loop
    def begin_step(self, measured_q=None) -> bool:
        """Pass the latest setpoint to the planner if one is waiting and the command hold has elapsed. A motion
        command re-energises a released arm, whose plan restarts from `measured_q`. Returns True if one went."""
        if self._pending_q is None and self._pending_gripper is None:
            return False
        if self.step - self._last_issue_step < self.hold_steps:
            return False
        if self._pending_q is not None:
            if not self.energised and measured_q is not None:
                here = torch.as_tensor(measured_q, dtype=torch.float32, device=self.device).reshape(1, 6)
                self.planner.reset(None, here)
            self.energised = True
            self.planner.vmax = self._vmax * self._pending_speed
            self.goal_q = self._pending_q
            self.planner.command(torch.ones(1, dtype=torch.bool, device=self.device), self._pending_q)
        if self._pending_gripper is not None:
            self.gripper_target_m = self._pending_gripper
        self._pending_q, self._pending_gripper = None, None
        self._last_issue_step = self.step
        self.commands_issued += 1
        return True

    def physics_step(self, dt: float) -> tuple[torch.Tensor, torch.Tensor]:
        """Advance the plan; the (position, velocity) the arm's drives should follow."""
        position = self.planner.step(dt)
        return position, self.planner.velocity

    def end_step(self, measured_q, measured_travel_m: float) -> Feedback | None:
        """Count the policy step and sample the measured state if a feedback message is due."""
        self.step += 1
        q = torch.as_tensor(measured_q, dtype=torch.float32).reshape(1, 6).cpu()
        signal = torch.cat([q, torch.tensor([[float(measured_travel_m)]])], dim=1)
        due = self.feedback.update(torch.tensor([self.step]), signal)
        if not bool(due[0]):
            return None
        value = self.feedback.value[0]
        servo = [m.quantise(a) for a in m.joint_to_servo_deg(value[:6].tolist())]
        return Feedback(servo, m.quantise(m.travel_to_units(float(value[6]))))

    def status_due(self, dt: float) -> bool:
        self._status_timer += dt
        if self._status_timer + 1e-9 < STATUS_PERIOD_S:
            return False
        self._status_timer -= STATUS_PERIOD_S
        return True


# ------------------------------------------------------------------------------------------- the wire
def encode(address: int, funcode: int, data: dict, seq: int = FEEDBACK_SEQ) -> str:
    return json.dumps({"seq": seq, "address": address, "funcode": funcode, "data": data}, separators=(",", ":"))


def decode(payload: str) -> dict | None:
    try:
        frame = json.loads(payload)
    except (TypeError, ValueError):
        return None
    if not isinstance(frame, dict) or "address" not in frame or "funcode" not in frame:
        return None
    return frame


class D1DdsLink:
    """CycloneDDS on the arm's side of the wire: the topics, types and domain of the physical D1."""

    def __init__(self, domain_id: int = 0):
        from cyclonedds.core import Policy, Qos
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.pub import DataWriter, Publisher
        from cyclonedds.sub import DataReader, Subscriber
        from cyclonedds.topic import Topic

        from d1_sdk.d1_msgs import ArmString_, PubServoInfo_

        self._ArmString, self._PubServoInfo = ArmString_, PubServoInfo_
        self._dp = DomainParticipant(domain_id)
        pub, sub = Publisher(self._dp), Subscriber(self._dp)
        # Commands can burst between drains; keep them all rather than only the newest.
        self._commands = DataReader(sub, Topic(self._dp, "rt/arm_Command", ArmString_),
                                    qos=Qos(Policy.History.KeepLast(64)))
        self._feedback = DataWriter(pub, Topic(self._dp, "rt/arm_Feedback", ArmString_))
        # No `rt/` prefix on this one: that asymmetry is the real firmware's.
        self._servo = DataWriter(pub, Topic(self._dp, "current_servo_angle", PubServoInfo_))
        self.domain_id = domain_id

    def commands(self):
        """Every command frame that arrived since the last call, oldest first."""
        while True:
            batch = self._commands.take(N=64)
            if not batch:
                return
            for sample in batch:
                frame = decode(getattr(sample, "data_", None))
                if frame is not None and frame.get("address") == ADDR_COMMAND:
                    yield frame

    def ack(self, ok: bool) -> None:
        self._feedback.write(self._ArmString(data_=encode(ADDR_ACK, FUNC_ACK_RECV, {"recv_status": 1})))
        self._feedback.write(self._ArmString(data_=encode(ADDR_ACK, FUNC_ACK_EXEC, {"exec_status": 1 if ok else 0})))

    def publish_feedback(self, feedback: Feedback) -> None:
        angles = list(feedback.servo_deg) + [feedback.gripper_units]
        self._servo.write(self._PubServoInfo(*[float(a) for a in angles]))
        self._feedback.write(self._ArmString(data_=encode(
            ADDR_STATE, FUNC_STATE_ANGLES, {f"angle{i}": round(a, 1) for i, a in enumerate(angles)})))

    def publish_status(self, energised: bool) -> None:
        self._feedback.write(self._ArmString(data_=encode(
            ADDR_STATE, FUNC_STATE_STATUS,
            {"enable_status": 1 if energised else 0, "power_status": 1, "error_status": 0})))
