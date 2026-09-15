# A velocity command that actually asks the robot to turn on the spot.
#
# THE PROBLEM. Measured on the 2026-09-11_22-01-27 policy, commanding a pure yaw rate with
# zero linear velocity:
#
#   yaw commanded   achieved   tracking   drift speed   net displacement over 10 s
#        0.2 rad/s    -0.001      -0.4 %     0.00 m/s        0 cm   (it stands still)
#        0.3          -0.278     -92.8 %     0.07            60 cm  (it turns the WRONG WAY)
#        0.5          -0.016      -3.2 %     0.18           152 cm  (it walks away)
#        0.8           0.572      71.5 %     0.14            20 cm
#        1.0           0.871      87.1 %     0.13            60 cm
#
# Below about 0.8 rad/s the yaw command is not tracked at all, and at every rate the robot
# translates while it spins. That is not a tuning shortfall; it is a behaviour the policy was
# never asked to produce.
#
# WHY IT WAS NEVER ASKED. The stock velocity command runs `heading_command=True` with
# `rel_heading_envs=1.0`, so `_update_command` OVERWRITES the sampled `ang_vel_z` every step
# with `clip(heading_control_stiffness * heading_error, ...)` -- a P-controller on heading.
# The yaw command is therefore always a decaying transient: the robot turns, the error
# shrinks, the command falls to zero. Sustaining 0.5 rad/s at a stiffness of 0.5 would take a
# permanent 1.0 rad heading error, which turning necessarily destroys. In 2000 iterations the
# policy sees essentially no examples of "spin at a steady rate", and none at all of "spin at
# a steady rate while holding position", because a pure spin also needs both linear
# components near zero -- about 1% of samples from independent uniforms.
#
# It also means widening `ranges.ang_vel_z` widens a CLIP, not a sampled distribution. That
# is worth knowing before tuning it.
#
# THE FIX IS TWO PARTS, and this class is the second. First, `heading_command` goes off, so
# the sampled yaw is held for the resampling period (10 s) instead of being overwritten --
# which also matches deployment, where DWB commands an angular velocity directly and there is
# no heading controller anywhere in the loop. Second, a fraction of environments get their
# linear commands zeroed, so "turn on the spot" is a case the policy practises rather than
# extrapolates into.

from __future__ import annotations

from dataclasses import MISSING

import torch

from isaaclab.envs.mdp.commands import UniformVelocityCommand
from isaaclab.envs.mdp.commands.commands_cfg import UniformVelocityCommandCfg
from isaaclab.utils import configclass


class TurnInPlaceVelocityCommand(UniformVelocityCommand):
    """Uniform velocity commands, with a share of them being pure in-place turns."""

    cfg: TurnInPlaceVelocityCommandCfg

    def __init__(self, cfg: TurnInPlaceVelocityCommandCfg, env):
        super().__init__(cfg, env)
        self.is_turning_env = torch.zeros_like(self.is_standing_env)
        # PATH LENGTH, FOR THE TERRAIN CURRICULUM. `terrain_levels_vel` upstream judges a
        # robot by its straight-line DISPLACEMENT from spawn, which only measures locomotion
        # when something forces the robot to walk in a straight line -- and with
        # `heading_command` off, nothing does. A held yaw command traces a circle, and a
        # circle returns to where it started however well it is walked. Measured over the
        # command distribution this config actually samples, assuming PERFECT tracking:
        #
        #                                         promoted   demoted
        #   stock Go2, heading_command=True          87.8 %    11.0 %
        #   this config, judged on displacement      28.5 %    47.2 %
        #   this config, judged on path length       87.3 %     0.0 %
        #
        # The middle row is a net downward ratchet -- it demotes faster than it promotes even
        # with a flawless policy -- and it is why run 2026-09-12_11-46-13 finished at terrain
        # level 1.45 of 10, i.e. having seen 3.3 degree slopes. Accumulated here rather than
        # in the curriculum term because the curriculum only runs at reset, and this has to
        # integrate every step. Both are zeroed by `CommandTerm.reset`, which runs AFTER
        # `curriculum_manager.compute`, so the curriculum reads the finished episode.
        self.metrics["path_length"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["commanded_path"] = torch.zeros(self.num_envs, device=self.device)

    def _update_metrics(self):
        super()._update_metrics()
        dt = self._env.step_dt
        self.metrics["path_length"] += torch.linalg.norm(self.robot.data.root_lin_vel_b[:, :2], dim=1) * dt
        self.metrics["commanded_path"] += torch.linalg.norm(self.vel_command_b[:, :2], dim=1) * dt

    def _resample_command(self, env_ids):
        super()._resample_command(env_ids)
        ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        r = torch.empty(len(ids), device=self.device)
        self.is_turning_env[ids] = r.uniform_(0.0, 1.0) <= self.cfg.rel_turning_envs
        self.vel_command_b[ids[self.is_turning_env[ids]], :2] = 0.0

    def _update_command(self):
        # The parent zeroes ALL THREE components for standing envs; run it first so that a
        # standing env stays standing even if it was also drawn as a turning one. Re-zeroing
        # the linear terms here rather than only at resample keeps the command honest if the
        # parent ever starts writing them, the way it already does for yaw under
        # `heading_command`.
        super()._update_command()
        turning_ids = self.is_turning_env.nonzero(as_tuple=False).flatten()
        self.vel_command_b[turning_ids, :2] = 0.0


@configclass
class TurnInPlaceVelocityCommandCfg(UniformVelocityCommandCfg):
    class_type: type = TurnInPlaceVelocityCommand

    rel_turning_envs: float = 0.15
    """Fraction of environments commanded to turn on the spot -- yaw only, zero linear.

    0.15 matches `rel_standing_envs`, on the reasoning that holding position while spinning is
    about as distinct a skill as holding position while still, and the mission needs it just
    as often: every `ReadRowLabel` approach ends with nav2 settling the robot's heading in
    place in front of a sign.
    """

    limit_ranges: UniformVelocityCommandCfg.Ranges = MISSING
    """The widest the command range is ever allowed to get.

    `ranges` is what is SAMPLED right now and the curriculum (`mdp/curriculums.py`) walks it
    outward from a narrow start; this is the ceiling it walks toward. Splitting the two is the
    whole point -- without it there is nowhere to record the intended final range while the
    sampled one is still small, and a PLAY config has no way to ask for the full range.
    """


class StairForwardVelocityCommand(TurnInPlaceVelocityCommand):
    """EXPERIMENT (2026-09-14, ../experiments.py): on stair tiles, a share of commands walk straight.

    WHY. measure_bench.py: no policy trained so far climbs the maze's 10 cm x 20 cm stairs on the
    sim's robot, and run 3's terrain curriculum sat at level ~1.1 of 10 (a ~6 cm riser) for its
    whole life. The command distribution is one reason the stair tiles teach so little: 15 %
    stand, 15 % turn on the spot, and the rest draw vx and vy independently over +-1 m/s, so few
    robots on a stair tile are ever asked to walk straight at the steps -- and the curriculum
    only judges robots whose command is straight. On a pyramid tile every heading out of the
    platform meets risers, so "straight ahead in the body frame" is "at the stairs".

    On resample, each environment standing on a stair column is, with probability
    `rel_stair_forward`, given vx in `stair_forward_speed`, vy 0 and a small yaw, and is neither
    standing nor turning. Everything else is the parent's.
    """

    cfg: StairForwardVelocityCommandCfg

    def __init__(self, cfg: StairForwardVelocityCommandCfg, env):
        super().__init__(cfg, env)
        import numpy as np

        gen = env.scene.terrain.cfg.terrain_generator
        names = list(gen.sub_terrains.keys())
        props = np.array([gen.sub_terrains[n].proportion for n in names], dtype=float)
        props /= props.sum()
        # the generator's own column -> sub-terrain rule (TerrainGenerator._generate_curriculum_terrains)
        col_type = [int(np.min(np.where(c / gen.num_cols + 0.001 < np.cumsum(props))[0])) for c in range(gen.num_cols)]
        stair_cols = [c for c, t in enumerate(col_type) if "Stairs" in type(gen.sub_terrains[names[t]]).__name__]
        self._stair_col = torch.zeros(gen.num_cols, dtype=torch.bool, device=self.device)
        self._stair_col[stair_cols] = True
        print(f"[StairForwardVelocityCommand] stair columns {stair_cols} of {gen.num_cols}")

    def _resample_command(self, env_ids):
        super()._resample_command(env_ids)
        ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        on_stairs = self._stair_col[self._env.scene.terrain.terrain_types[ids]]
        pick = on_stairs & (torch.rand(len(ids), device=self.device) < self.cfg.rel_stair_forward)
        sel = ids[pick]
        if len(sel) == 0:
            return
        lo, hi = self.cfg.stair_forward_speed
        self.vel_command_b[sel, 0] = torch.empty(len(sel), device=self.device).uniform_(lo, hi)
        self.vel_command_b[sel, 1] = 0.0
        self.vel_command_b[sel, 2] = torch.empty(len(sel), device=self.device).uniform_(-0.2, 0.2)
        self.is_standing_env[sel] = False
        self.is_turning_env[sel] = False


@configclass
class StairForwardVelocityCommandCfg(TurnInPlaceVelocityCommandCfg):
    class_type: type = StairForwardVelocityCommand
    rel_stair_forward: float = 0.5
    stair_forward_speed: tuple[float, float] = (0.3, 1.0)
