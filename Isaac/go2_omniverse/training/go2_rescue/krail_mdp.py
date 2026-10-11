"""Command, curriculum and reward terms for the K-Rail Square policy (`krail_env_cfg.py`).

THE COMMAND serves two masters that pull apart, which is the lesson of this package's earlier runs
(`mdp/commands.py`, `mdp/curriculums.py`):

* Tracking. nav2's DWB sends an angular velocity directly and holds it, turns on the spot to settle a heading, and
  stops dead. So most environments get a held (vx, vy, wz) with no heading controller, some turn on the spot with no
  linear command, and some stand.
* The terrain ladder. It only means anything if a promoted robot crossed the terrain, and a robot with a held yaw
  command walks in circles. So the rest are heading-controlled: they pick a direction and walk it, which is what
  makes them meet decks and rails.

THE CURRICULUM judges each robot on how far it walked ALONG ITS COMMAND while the command was straight. Every step,
the command's direction is turned into the world frame and the robot's world velocity is projected onto it; a step
counts only while the command asks for travel (|v| > 0.25 m/s) and little turning (|wz| < 0.3 rad/s). Circling and
turning robots add nothing, so they are neither promoted nor demoted. A robot that walked 4 m along its command (half
a tile) is promoted; one that was asked for at least 2 m of straight travel and delivered under half of it is
demoted. On these tiles 4 m straight from the spawn square crosses decks and rails nearly whichever way it points,
so promotion means crossing, not trotting round a platform (the failure `terrain_levels_traverse` records).
"""
from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs.mdp.commands import UniformVelocityCommand
from isaaclab.envs.mdp.commands.commands_cfg import UniformVelocityCommandCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply_inverse, yaw_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class KRailVelocityCommand(UniformVelocityCommand):
    """Held velocity commands, heading-controlled walkers, in-place turns and stands, with the straight-travel
    metrics the terrain curriculum reads."""

    cfg: "KRailVelocityCommandCfg"

    def __init__(self, cfg: "KRailVelocityCommandCfg", env):
        super().__init__(cfg, env)
        self.is_turning_env = torch.zeros_like(self.is_standing_env)
        self.metrics["straight_progress"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["straight_asked"] = torch.zeros(self.num_envs, device=self.device)
        # True velocity tracking, per step lived rather than per resampling period (the parent's metric divides by
        # the longest resampling period whatever the episode did): logged so the curves read in m/s and rad/s.
        self.metrics["lin_err_mps"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["yaw_err_radps"] = torch.zeros(self.num_envs, device=self.device)
        self._steps = torch.zeros(self.num_envs, device=self.device)

    def _update_metrics(self):
        super()._update_metrics()
        dt = self._env.step_dt
        cmd = self.vel_command_b
        speed = torch.linalg.norm(cmd[:, :2], dim=1)
        straight = (speed > self.cfg.straight_min_speed) & (cmd[:, 2].abs() < self.cfg.straight_max_yaw)
        # The command is in the base frame; on these tiles the base pitches, so take its direction in the yaw frame.
        yaw = self.robot.data.heading_w
        c, s = torch.cos(yaw), torch.sin(yaw)
        dir_b = cmd[:, :2] / speed.clamp(min=1e-6).unsqueeze(1)
        dir_w = torch.stack([c * dir_b[:, 0] - s * dir_b[:, 1], s * dir_b[:, 0] + c * dir_b[:, 1]], dim=1)
        along = (self.robot.data.root_lin_vel_w[:, :2] * dir_w).sum(dim=1)
        self.metrics["straight_progress"] += torch.where(straight, along * dt, torch.zeros_like(along))
        self.metrics["straight_asked"] += torch.where(straight, speed * dt, torch.zeros_like(speed))
        vel_yaw = quat_apply_inverse(yaw_quat(self.robot.data.root_quat_w), self.robot.data.root_lin_vel_w)
        lin_err = torch.linalg.norm(cmd[:, :2] - vel_yaw[:, :2], dim=1)
        yaw_err = (cmd[:, 2] - self.robot.data.root_ang_vel_w[:, 2]).abs()
        self._steps += 1.0
        # Running means, so the value at reset is the episode's mean error.
        self.metrics["lin_err_mps"] += (lin_err - self.metrics["lin_err_mps"]) / self._steps
        self.metrics["yaw_err_radps"] += (yaw_err - self.metrics["yaw_err_radps"]) / self._steps

    def reset(self, env_ids: Sequence[int] | None = None):
        extras = super().reset(env_ids)
        if env_ids is None:
            env_ids = slice(None)
        self._steps[env_ids] = 0.0
        return extras

    def _resample_command(self, env_ids):
        super()._resample_command(env_ids)
        ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        r = torch.empty(len(ids), device=self.device)
        # Turning on the spot: no linear command, and a yaw rate at least `turn_min_rate`, either way.
        turning = (r.uniform_(0.0, 1.0) <= self.cfg.rel_turning_envs) & ~self.is_heading_env[ids]
        self.is_turning_env[ids] = turning
        sel = ids[turning]
        if len(sel):
            lo, hi = self.cfg.turn_min_rate, self.cfg.ranges.ang_vel_z[1]
            rate = torch.empty(len(sel), device=self.device).uniform_(lo, hi)
            sign = torch.where(torch.rand(len(sel), device=self.device) < 0.5, -1.0, 1.0)
            self.vel_command_b[sel, :2] = 0.0
            self.vel_command_b[sel, 2] = rate * sign
        # Commands below `min_speed` are rounded to zero: the policy should stand for them, not creep.
        small = torch.linalg.norm(self.vel_command_b[ids, :2], dim=1) < self.cfg.min_speed
        self.vel_command_b[ids[small], :2] = 0.0

    def _update_command(self):
        super()._update_command()
        turning = self.is_turning_env.nonzero(as_tuple=False).flatten()
        self.vel_command_b[turning, :2] = 0.0


@configclass
class KRailVelocityCommandCfg(UniformVelocityCommandCfg):
    class_type: type = KRailVelocityCommand

    rel_turning_envs: float = 0.12
    """Share of the held-command (non-heading) environments that turn on the spot."""
    turn_min_rate: float = 0.3
    """An in-place turn is at least this fast, rad/s. nav2's DWB turns at 0.8 or more."""
    min_speed: float = 0.1
    """Linear commands slower than this are zeroed, m/s."""
    straight_min_speed: float = 0.25
    straight_max_yaw: float = 0.3


def terrain_levels_progress(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    command_name: str = "base_velocity",
    promote_m: float = 4.0,
    demote_fraction: float = 0.5,
    min_asked_m: float = 2.0,
) -> torch.Tensor:
    """Promote a robot that walked `promote_m` along its straight command; demote one that delivered under
    `demote_fraction` of at least `min_asked_m` asked. See the module docstring."""
    terrain = env.scene.terrain
    term = env.command_manager.get_term(command_name)
    progress = term.metrics["straight_progress"][env_ids]
    asked = term.metrics["straight_asked"][env_ids]
    move_up = progress > promote_m
    move_down = (asked > min_asked_m) & (progress < demote_fraction * asked) & ~move_up
    terrain.update_env_origins(env_ids, move_up, move_down)
    return torch.mean(terrain.terrain_levels.float())


def stand_still(env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg, lin_threshold: float = 0.1,
                yaw_threshold: float = 0.1) -> torch.Tensor:
    """Legs off their default pose while the command is to stand. Isaac Lab's `stand_still_joint_deviation_l1` gates
    on the linear command alone, so it also charges an in-place turn for moving its legs."""
    cmd = env.command_manager.get_command(command_name)
    still = (torch.linalg.norm(cmd[:, :2], dim=1) < lin_threshold) & (cmd[:, 2].abs() < yaw_threshold)
    asset = env.scene[asset_cfg.name]
    deviation = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    return torch.sum(torch.abs(deviation), dim=1) * still
