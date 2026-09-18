"""Where the simulated D1's firmware meets PhysX: joint ids, drive targets and gains, and the DDS link.

`SimulatedD1.begin_step()` runs once per policy step before physics (the `D1ArmAction` term calls it): commands
that came over DDS are handed to the firmware, and the firmware passes the latest one to its planner if the
10 Hz hold allows. `apply()` runs every physics step and writes the planner's position and velocity to the arm's
drives. `end_step()` runs after the policy step and sends feedback when it is due.
"""
from __future__ import annotations

import torch

from . import d1_model as m
from .d1_arm import D1DdsLink, D1Firmware


class SimulatedD1:
    def __init__(self, env, domain_id: int = 0, seed: int = 0, dds: bool = True):
        core = env.unwrapped if hasattr(env, "unwrapped") else env
        self.env = core
        self.robot = core.scene["robot"]
        self.device = core.device
        self.arm_ids = self.robot.find_joints(m.ARM_JOINTS, preserve_order=True)[0]
        self.finger_ids = self.robot.find_joints(m.FINGER_JOINTS, preserve_order=True)[0]
        limits = self.robot.data.joint_pos_limits[0, self.arm_ids].detach().cpu()
        self.lower, self.upper = limits[:, 0].tolist(), limits[:, 1].tolist()
        self._widen_gripper()
        self.rest_q = m.rest_joint_rad(self.lower, self.upper)
        self.rest_travel = m.units_to_travel(m.REST_GRIPPER_UNITS)
        self.firmware = D1Firmware(core.step_dt, self.lower, self.upper, self.rest_q, self.rest_travel, seed=seed,
                                   device=self.device)
        self.link = D1DdsLink(domain_id) if dds else None
        self._energised = True
        self._gains()
        self.hold_rest()

    # --------------------------------------------------------------------------------------------- setup
    def _widen_gripper(self) -> None:
        """Let the fingers shut past the URDF's stop, to where the real pads meet (D1Training's pick, F-063)."""
        ids = self.finger_ids
        limits = self.robot.data.joint_pos_limits[:, ids].clone()
        factor = float(self.robot.cfg.soft_joint_pos_limit_factor)
        lower, upper = m.gripper_joint_limits(float(limits[0, 0, 0]), float(limits[0, 0, 1]), factor)
        limits[:, 0, 0], limits[:, 0, 1] = lower, upper
        limits[:, 1, 0], limits[:, 1, 1] = -upper, -lower
        self.robot.write_joint_position_limit_to_sim(limits, joint_ids=ids)
        soft = self.robot.data.soft_joint_pos_limits[0, ids].cpu().numpy()
        print(f"[D1] fingers may shut {1000 * -m.GRIPPER_SHUT_TRAVEL_M:.1f} mm past the URDF's stop, so the pads meet "
              f"at {1000 * m.PINCH_CLOSED_GAP_M:.0f} mm as the real arm's do (F-063); soft limits "
              f"{1000 * soft[0, 0]:.1f} to {1000 * soft[0, 1]:.1f} mm", flush=True)

    def _gains(self) -> None:
        """Drives on (D1Training's 4000/400 force drives) or, released, off: the arm falls (F-028)."""
        on = self.firmware.energised
        n = len(self.arm_ids) + len(self.finger_ids)
        stiffness = [m.ARM_STIFFNESS] * len(self.arm_ids) + [m.GRIPPER_STIFFNESS] * len(self.finger_ids)
        damping = [m.ARM_DAMPING] * len(self.arm_ids) + [m.GRIPPER_DAMPING] * len(self.finger_ids)
        scale = 1.0 if on else 0.0
        ids = list(self.arm_ids) + list(self.finger_ids)
        self.robot.write_joint_stiffness_to_sim(
            torch.tensor([[s * scale for s in stiffness]], device=self.device).expand(self.robot.num_instances, n),
            joint_ids=ids)
        self.robot.write_joint_damping_to_sim(
            torch.tensor([[d * scale for d in damping]], device=self.device).expand(self.robot.num_instances, n),
            joint_ids=ids)
        if on != self._energised:
            print(f"[D1] {'energised' if on else 'RELEASED: no torque, the arm falls (F-028)'}", flush=True)
        self._energised = on

    # --------------------------------------------------------------------------------------------- state
    def measured(self) -> tuple[list[float], float]:
        q = self.robot.data.joint_pos[0, self.arm_ids].detach().cpu().tolist()
        travel = float(self.robot.data.joint_pos[0, self.finger_ids[0]].detach().cpu())
        return q, travel

    def hold_rest(self) -> None:
        """The firmware holds the rest pose from here: startup, and a level load that put the arm there."""
        self.firmware.park(self.rest_q, self.rest_travel)
        self._write_targets(torch.tensor([self.rest_q], device=self.device), torch.zeros(1, 6, device=self.device))
        if not self._energised:
            self._gains()

    def _write_targets(self, position: torch.Tensor, velocity: torch.Tensor) -> None:
        n = self.robot.num_instances
        self.robot.set_joint_position_target(position.expand(n, 6), joint_ids=self.arm_ids)
        self.robot.set_joint_velocity_target(velocity.expand(n, 6), joint_ids=self.arm_ids)
        travel = self.firmware.gripper_target_m
        fingers = torch.tensor([[travel, -travel]], device=self.device).expand(n, 2)
        self.robot.set_joint_position_target(fingers, joint_ids=self.finger_ids)

    # --------------------------------------------------------------------------------------------- the loop
    def begin_step(self) -> None:
        if self.link is not None:
            for frame in self.link.commands():
                self.link.ack(self.firmware.receive(frame))
        q, _ = self.measured()
        self.firmware.begin_step(q)
        if self.firmware.energised != self._energised:
            self._gains()

    def apply(self, physics_dt: float) -> None:
        position, velocity = self.firmware.physics_step(physics_dt)
        # Velocity feedforward, as D1Training's action term: the fitted trapezoid is the joint's measured motion,
        # servo lag included, so the drive follows it rather than trailing it by another 97 ms.
        self._write_targets(position, velocity)

    def end_step(self) -> None:
        q, travel = self.measured()
        feedback = self.firmware.end_step(q, travel)
        if self.link is None:
            return
        if feedback is not None:
            self.link.publish_feedback(feedback)
        if self.firmware.status_due(self.env.step_dt):
            self.link.publish_status(self.firmware.energised)
