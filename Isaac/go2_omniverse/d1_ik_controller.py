"""Cartesian (IK) control for the D1 arm, driven entirely through the D1 API.

The real D1 has no Cartesian interface -- it accepts joint angles in degrees at
10 Hz and nothing else. So Cartesian control is necessarily a host-side loop:
read joint angles from the arm, solve IK, write joint angles back. That is what
this does, and it does it through `D1Arm`, so the loop is identical against the
simulator and against hardware.

Everything that touches Isaac lives behind `KinematicsBackend`. To drive the
real arm, implement one on top of a URDF kinematics library (pinocchio, PyKDL)
and hand it to the same controller -- see `README.md`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import torch

from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.utils.math import quat_apply, quat_inv, quat_mul

from d1_sdk import protocol as proto

_RAD2DEG = 180.0 / math.pi
_DEG2RAD = math.pi / 180.0


@dataclass
class KinSnapshot:
    """A self-consistent kinematic state: EE pose, Jacobian and the joint angles
    they were evaluated at. Keeping them together is what lets the solver stay
    correct regardless of which backend produced them."""

    ee_pos: torch.Tensor      # (N, 3) world
    ee_quat: torch.Tensor     # (N, 4) world, wxyz
    jacobian: torch.Tensor    # (N, 6, num_joints)
    joint_pos: torch.Tensor   # (N, num_joints) radians


class KinematicsBackend:
    """Supplies forward kinematics and a Jacobian for the arm."""

    def snapshot(self, reported_angles_rad: torch.Tensor) -> KinSnapshot:
        raise NotImplementedError

    def to_world(self, rel_pos: torch.Tensor, rel_quat: torch.Tensor):
        """Map a target expressed in the arm's command frame into the frame the
        snapshot uses."""
        raise NotImplementedError

    def to_local(self, world_pos: torch.Tensor, world_quat: torch.Tensor):
        """Inverse of to_world. Used to re-derive a Cartesian target from where
        the arm actually is."""
        raise NotImplementedError


class IsaacKinematics(KinematicsBackend):
    """Reads kinematics straight from PhysX.

    PhysX evaluates the Jacobian at the articulation's *live* configuration, so
    `reported_angles_rad` is ignored and the live joint positions are returned
    alongside it. The snapshot stays internally consistent, which is what the
    solver needs; the cost is that it does not model the arm's 10 Hz encoder
    latency. A pinocchio backend evaluating at the reported angles would.
    """

    def __init__(self, arm, robot, ee_body_name: str = "Link6"):
        self._arm = arm
        self._robot = robot
        self._ee_body_idx = arm.find_bodies(ee_body_name)[0][0]
        # Arm joints only: the URDF's gripper fingers are not part of the IK chain.
        self._joint_ids = [
            arm.find_joints(name)[0][0] for name in proto.JOINT_NAMES
        ]

    @property
    def joint_ids(self) -> list[int]:
        return self._joint_ids

    def snapshot(self, reported_angles_rad: torch.Tensor) -> KinSnapshot:
        arm = self._arm
        jac = arm.root_physx_view.get_jacobians()
        # A fixed base drops the root body from the Jacobian's row indexing.
        idx = self._ee_body_idx - 1 if arm.is_fixed_base else self._ee_body_idx
        jacobian = jac[:, idx, :, :]
        if not arm.is_fixed_base:
            jacobian = jacobian[:, :, 6:]  # strip the floating-base DOF columns
        jacobian = jacobian[:, :, self._joint_ids]

        return KinSnapshot(
            ee_pos=arm.data.body_pos_w[:, self._ee_body_idx, :],
            ee_quat=arm.data.body_quat_w[:, self._ee_body_idx, :],
            jacobian=jacobian,
            joint_pos=arm.data.joint_pos[:, self._joint_ids],
        )

    def to_world(self, rel_pos: torch.Tensor, rel_quat: torch.Tensor):
        """Targets are given relative to the quadruped's base, which is what the
        teleop layer works in and what the arm is mounted to."""
        root_pos = self._robot.data.root_state_w[:, :3]
        root_quat = self._robot.data.root_state_w[:, 3:7]
        return (
            root_pos + quat_apply(root_quat, rel_pos),
            quat_mul(root_quat, rel_quat),
        )

    def to_local(self, world_pos: torch.Tensor, world_quat: torch.Tensor):
        root_pos = self._robot.data.root_state_w[:, :3]
        root_quat = self._robot.data.root_state_w[:, 3:7]
        root_inv = quat_inv(root_quat)
        return (
            quat_apply(root_inv, world_pos - root_pos),
            quat_mul(root_inv, world_quat),
        )


class D1CartesianController:
    """Holds a Cartesian target and streams joint angles to a D1 arm.

    Runs at the arm's real 10 Hz command rate: `update()` may be called every
    simulator step but only emits on the 10 Hz boundary, so what you see in sim
    is the motion quality the hardware can actually deliver.
    """

    def __init__(
        self,
        arm_client,
        backend: KinematicsBackend,
        num_envs: int,
        device,
        rate_hz: float = proto.FEEDBACK_RATE_HZ,
        smoothing: float = 0.05,
    ):
        self._client = arm_client
        self._backend = backend
        self._device = device
        self._num_envs = num_envs
        self._period = 1.0 / rate_hz
        self._alpha = smoothing
        self._time_since_send = 0.0
        self._suspended = False

        cfg = DifferentialIKControllerCfg(command_type="pose", ik_method="dls")
        self._ik = DifferentialIKController(cfg, num_envs=num_envs, device=device)

        # The smoothed target the solver actually chases, base-relative.
        self.current_rel_pos = torch.zeros((num_envs, 3), device=device, dtype=torch.float32)
        self.current_rel_rot = torch.tensor(
            [[1.0, 0.0, 0.0, 0.0]], device=device, dtype=torch.float32
        ).repeat(num_envs, 1)

    def reset(self, rel_pos, rel_rot) -> None:
        self.current_rel_pos[:] = torch.tensor(
            rel_pos, device=self._device, dtype=torch.float32
        ).repeat(self._num_envs, 1)
        self.current_rel_rot[:] = torch.tensor(
            rel_rot, device=self._device, dtype=torch.float32
        ).repeat(self._num_envs, 1)
        self._time_since_send = 0.0
        self._suspended = False
        try:
            self._ik.reset()
        except Exception:
            pass

    @property
    def is_suspended(self) -> bool:
        return self._suspended

    def suspend(self) -> None:
        """Stop streaming joint commands.

        Required before any non-streaming command (return-to-zero, drag
        teaching): this loop rewrites all six joints every 100 ms, so it would
        otherwise overwrite such a command before the arm visibly moved. The
        same is true against real hardware.
        """
        self._suspended = True

    def resume_from_arm(self):
        """Re-engage streaming from wherever the arm actually is.

        Returns the arm's current pose as a base-relative (pos, quat) so the
        caller can adopt it as the teleop target. Without this the arm would
        snap back to the pre-suspend target the moment streaming resumed.
        """
        snap = self._backend.snapshot(self._reported_rad())
        rel_pos, rel_rot = self._backend.to_local(snap.ee_pos, snap.ee_quat)
        self.current_rel_pos[:] = rel_pos
        self.current_rel_rot[:] = rel_rot
        self._suspended = False
        self._time_since_send = 0.0
        try:
            self._ik.reset()
        except Exception:
            pass
        pos = rel_pos[0].detach().cpu().numpy().tolist()
        rot = rel_rot[0].detach().cpu().numpy().tolist()
        return pos, rot

    def _reported_rad(self) -> torch.Tensor:
        """The arm's reported joint angles, in radians, from the D1 API."""
        self._client.poll()
        return torch.tensor(
            [[a * _DEG2RAD for a in self._client.get_joint_angles()]],
            device=self._device,
            dtype=torch.float32,
        ).repeat(self._num_envs, 1)

    def update(self, target_rel_pos, target_rel_rot, dt: float) -> bool:
        """Advance the controller. Returns True if a command went out this tick."""
        target_pos = torch.tensor(
            np.asarray(target_rel_pos, dtype=np.float32), device=self._device
        ).repeat(self._num_envs, 1)
        target_rot = torch.tensor(
            np.asarray(target_rel_rot, dtype=np.float32), device=self._device
        ).repeat(self._num_envs, 1)

        # q and -q are the same rotation, so lerping toward an opposite-signed
        # quaternion takes the long way round -- and an exactly antipodal one
        # collapses the norm to zero and yields NaN joint targets. Teleop steps
        # incrementally and never hits this, but a ROS-published pose can.
        flip = (self.current_rel_rot * target_rot).sum(dim=-1, keepdim=True) < 0.0
        target_rot = torch.where(flip, -target_rot, target_rot)

        # Ease toward the target so a keypress does not become a step input.
        self.current_rel_pos += self._alpha * (target_pos - self.current_rel_pos)
        self.current_rel_rot += self._alpha * (target_rot - self.current_rel_rot)
        self.current_rel_rot /= torch.norm(self.current_rel_rot, dim=-1, keepdim=True)

        # Joint state comes from the arm over the D1 API -- the same source the
        # real arm would give us. Polled every tick (even while suspended, so
        # the cached state stays fresh) rather than only on send ticks, or the
        # arm's 10 Hz publishing would outrun us.
        reported_rad = self._reported_rad()

        if self._suspended:
            return False

        self._time_since_send += dt
        if self._time_since_send < self._period:
            return False
        self._time_since_send = 0.0

        snap = self._backend.snapshot(reported_rad)
        world_pos, world_rot = self._backend.to_world(
            self.current_rel_pos, self.current_rel_rot
        )

        self._ik.set_command(torch.cat([world_pos, world_rot], dim=-1))
        joint_targets = self._ik.compute(
            ee_pos=snap.ee_pos,
            ee_quat=snap.ee_quat,
            jacobian=snap.jacobian,
            joint_pos=snap.joint_pos,
        )

        # Out through the D1 API in degrees. Point the client at a real arm and
        # this exact call drives the hardware.
        angles_deg = (joint_targets[0] * _RAD2DEG).detach().cpu().numpy()
        self._client.set_all_joint_angles(angles_deg, mode=proto.SMOOTH_STREAMING)
        return True
