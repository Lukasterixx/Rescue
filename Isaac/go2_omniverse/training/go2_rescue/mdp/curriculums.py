# A curriculum on the COMMAND RANGE, not on the terrain.
#
# THE PROBLEM IT TARGETS. Forward tracking at 1 m/s has been the one number that would not
# move. Measured across the last two flat runs:
#
#   command        achieved   tracking
#   walk 0.5         0.458      91.5 %      (and 0.537 / 107.4 % on the other run)
#   walk 1.0         0.836      83.6 %      (and 0.920 /  92.0 %)
#
# The fast end is consistently the worst-tracked, and the obvious lever -- narrowing
# `base_linear_velocity`'s kernel from std 0.5 to 0.3 -- buys tracking at the cost of the
# diagonal-pair limp, because a tighter kernel pays more for matching the commanded speed
# than for how the feet get there. So the kernel is the wrong lever.
#
# THE DIAGNOSIS IS SAMPLING, NOT SHAPING. `lin_vel_x` is sampled uniformly over (-1.0, 1.0)
# from iteration 0. At that point the policy cannot walk at all, so the fast samples are
# noise it learns nothing from, and by the time it can walk it has settled into a gait
# optimised for the middle of the range -- where most of the probability mass that ever
# produced usable gradient was. The fast end never gets a curriculum's worth of attention,
# it just gets a seventh of the samples for the whole run.
#
# THE FIX is the standard command-level curriculum: start the sampled range narrow, widen it
# only when the policy is actually tracking what it is already being asked for. Every widening
# is therefore earned, and the hard samples arrive when they can be learned from. Adapted from
# `unitree_rl_lab`'s `lin_vel_cmd_levels` (KIT MaiRo lab fork), with the firing gate rewritten
# -- see below.

from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def lin_vel_cmd_levels(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    command_name: str = "base_velocity",
    error_ratio_threshold: float = 0.55,
    step: float = 0.1,
    force_full_after_steps: int = 15000,
) -> torch.Tensor:
    """Widen the sampled linear-velocity range whenever tracking is good enough.

    The logged state is the current upper bound on `lin_vel_x`, so TensorBoard's
    `Curriculum/lin_vel_cmd_levels` reads directly as how far the curriculum has got: it
    starts at the configured `ranges.lin_vel_x[1]` and should reach `limit_ranges.lin_vel_x[1]`
    well before training ends. A flat line is the failure, and it is visible in minutes.

    Args:
        error_ratio_threshold: widen when mean tracking error, as a FRACTION of mean commanded
            speed, falls below this. See below for why it is a ratio and not an absolute.
        step: how much to add to each end of the range per widening, in m/s.
        force_full_after_steps: past this many environment steps, widen regardless of
            tracking. A backstop against the curriculum stalling for the whole run.
    """
    command_term = env.command_manager.get_term(command_name)
    ranges = command_term.cfg.ranges
    limit_ranges = command_term.cfg.limit_ranges

    # THE FIRING GATE IS A DISTANCE, NOT A MODULO. The upstream version gates on
    # `env.common_step_counter % env.max_episode_length == 0`. `common_step_counter` advances
    # by `num_steps_per_env` (24) per iteration, so it only ever lands on a multiple of
    # `max_episode_length` (1000 here) when 24k is divisible by 1000 -- every 125 iterations,
    # by arithmetic accident. Change `num_steps_per_env` to anything coprime with 1000 and the
    # curriculum silently never fires at all, which is the worst possible failure: training
    # completes, the range stays at its narrow starting value, and nothing in the logs says so.
    # Measuring the gap instead fires about once per episode length whatever the rollout size.
    last_fired = getattr(env, "_lin_vel_cmd_level_step", None)
    if last_fired is not None and env.common_step_counter - last_fired < env.max_episode_length:
        return torch.tensor(ranges.lin_vel_x[1], device=env.device)
    env._lin_vel_cmd_level_step = env.common_step_counter

    # THE GATE IS A RATIO, AND IT HAS TO BE. The obvious gates -- "is the tracking reward above
    # some value", as upstream does, or "is the tracking error below some value" -- both fail,
    # because BOTH quantities depend on the range being sampled, which is the very thing the
    # curriculum is moving. Measured on the 2026-09-12_11-46-13 rough run and on the command
    # distribution:
    #
    #                                        mean |err|     Episode_Reward/base_linear_velocity
    #   policy that does nothing, at +-0.3     0.23 m/s          ~2.7   (of a weight of 5.0)
    #   trained policy, at +-1.0               0.29 m/s           3.30
    #
    # A do-nothing policy on the NARROW range outscores a good policy on the WIDE one, on both
    # measures, because small commands are nearly satisfied by standing still. So there is no
    # fixed absolute threshold that refuses the first and admits the second: set it to admit
    # the trained policy and it fires immediately on an untrained one; set it to refuse the
    # untrained one and the curriculum can never reach full range. The upstream 0.8-of-weight
    # gate lands in the second case here -- it would have trained the whole run at +-0.3 and
    # logged nothing to say so.
    #
    # Normalising by the commanded speed removes the range from both sides. A policy that
    # ignores its command scores ~1.0 at ANY range. Against that, the v1 rough run's own
    # trajectory (true mean error over mean commanded speed of 0.536 m/s):
    #
    #     iteration    50    400    800   1200   2000
    #     ratio      1.02   0.91   0.39   0.25   0.27
    #
    # so 0.55 opens the gate somewhere between iterations 400 and 800 on a policy learning at
    # the FULL range, and much earlier than that on one starting at +-0.3 where tracking is
    # easy. That separation holds as the range moves, which is the only property that makes
    # this gate mean the same thing at the start of the curriculum and at the end.
    # THE METRIC IS NOT A MEAN, AND IT IS NOT PER EPISODE. `_update_metrics` accumulates
    # `err / max_command_step`, where `max_command_step` is the RESAMPLING period (10 s = 500
    # steps) while an episode is 20 s = 1000 steps. So a full episode reports 2x the mean
    # error -- and an environment that fell over after 200 steps reports a FIFTH of its true
    # error, which would read as excellent tracking and widen the range on the strength of the
    # robots that crashed. Normalising by the steps actually lived fixes both at once and puts
    # the ratio below on a true m/s scale.
    lived = env.episode_length_buf[env_ids].float()
    # Only environments that got most of the way through an episode carry a usable average.
    # This also covers the very first call, at env creation, where nothing has run at all.
    usable = lived >= 0.5 * env.max_episode_length
    steps_per_command = command_term.cfg.resampling_time_range[1] / env.step_dt

    widen = env.common_step_counter >= force_full_after_steps
    if not widen and bool(usable.any()):
        raw = command_term.metrics["error_vel_xy"][env_ids][usable]
        mean_error = (raw * steps_per_command / lived[usable]).mean()
        # All envs, not just the ones resetting: this estimates the command DISTRIBUTION's
        # mean speed, and must include the standing and turning envs whose linear command is
        # zeroed, because the error above includes them too.
        cmd_speed = torch.linalg.norm(env.command_manager.get_command(command_name)[:, :2], dim=1).mean()
        widen = bool(mean_error / torch.clamp(cmd_speed, min=1e-3) < error_ratio_threshold)

    if widen:
        delta = torch.tensor([-step, step], device=env.device)
        for axis in ("lin_vel_x", "lin_vel_y"):
            widened = torch.clamp(
                torch.tensor(getattr(ranges, axis), device=env.device) + delta,
                min=getattr(limit_ranges, axis)[0],
                max=getattr(limit_ranges, axis)[1],
            )
            setattr(ranges, axis, widened.tolist())

    return torch.tensor(ranges.lin_vel_x[1], device=env.device)


def terrain_levels_path(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    command_name: str = "base_velocity",
    demote_fraction: float = 0.5,
) -> torch.Tensor:
    """Terrain curriculum judged on DISTANCE WALKED, not on displacement from spawn.

    A drop-in replacement for `terrain_levels_vel`, which promotes an environment when its
    straight-line displacement from the spawn point exceeds half the terrain patch (4 m) and
    demotes it when that displacement is under half what the command asked for.

    WHY DISPLACEMENT IS THE WRONG MEASURE HERE. It is a proxy for "did this robot locomote",
    and the proxy only holds while something forces every robot to walk in a straight line.
    In the stock task something does: `heading_command=True` overwrites the yaw command every
    step with a P-controller on heading error, which decays to zero, so a robot picks a
    heading and goes. This config turns that off on purpose -- it is what took yaw tracking
    from ~55% to 95-104% and what makes an in-place turn a trained behaviour rather than an
    extrapolation. But a HELD yaw command traces a circle: at 1.0 rad/s with any forward
    speed the radius is well under a metre, and a circle ends where it began however well it
    is walked. Monte Carlo over the command distribution this config samples, with PERFECT
    tracking assumed, over a 20 s episode:

                                              promoted   demoted
        stock Go2, heading_command=True          87.8 %    11.0 %
        this config, displacement rule           28.5 %    47.2 %
        this config, PATH LENGTH rule            87.3 %     0.0 %

    The displacement rule demotes faster than it promotes even with a flawless policy, so the
    terrain level ratchets DOWN for the whole run. Measured: 2026-09-12_11-46-13 started near
    level 2.9, fell to 0.04 by iteration 400 and finished at 1.45 of 10 -- which is a 3.3
    degree slope, and is why that policy scuffs and bounces on real topography.

    Note the fix is NOT to narrow the yaw range or drop the in-place turns. Removing the
    standing and turning environments makes the displacement rule WORSE (38.1% up, 60.1%
    down), because a zeroed command is exempt from demotion -- `move_down` compares against
    `|cmd| * 10`, which is zero. The command generator is right; the ruler was wrong.

    Path length is shape-agnostic: it measures the thing the rule was always reaching for,
    and it reproduces the stock task's promotion rate under a command distribution the stock
    rule cannot handle. It also becomes a genuine measure of SKILL rather than of geometry --
    at 40% tracking it promotes 55% and demotes 37%, where the displacement rule promotes 4%.
    """
    terrain = env.scene.terrain
    command_term = env.command_manager.get_term(command_name)
    walked = command_term.metrics["path_length"][env_ids]
    asked = command_term.metrics["commanded_path"][env_ids]

    move_up = walked > terrain.cfg.terrain_generator.size[0] / 2
    # A standing or in-place-turning environment asked for nothing and so is neither promoted
    # nor demoted, which is the same treatment the stock rule gives it and the right one: it
    # was never a test of the terrain.
    move_down = (walked < asked * demote_fraction) & ~move_up
    terrain.update_env_origins(env_ids, move_up, move_down)
    return torch.mean(terrain.terrain_levels.float())


def terrain_levels_traverse(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    command_name: str = "base_velocity",
    straight_yaw: float = 0.4,
    min_speed: float = 0.1,
    demote_fraction: float = 0.5,
) -> torch.Tensor:
    """Promote only environments that actually CROSSED the terrain, judged on displacement.

    WHY `terrain_levels_path` ABOVE IS WRONG FOR THIS TASK, measured the hard way. That rule
    promotes on DISTANCE WALKED, which fixed P2Dingo's problem -- a held yaw command traces a
    circle and a circle has no displacement, so the stock rule demoted good policies. On
    P2Dingo's terrain that was safe: slopes, rough ground and boxes fill the whole patch, so a
    robot that walks anywhere is on the terrain.

    IT IS NOT SAFE ON STAIRS. A pyramid-stairs tile is a FLAT PLATFORM with the stairs at its
    edges, so a robot can trot in circles on the platform, bank path length, and be promoted
    every single episode without ever touching a step. Monte Carlo over this task's own
    command distribution, assuming PERFECT tracking:

                                        promoted   demoted
        stock rule, displacement           27.7 %    60.9 %
        terrain_levels_path                85.7 %     0.0 %   <- one-way ratchet
        this rule                          14.8 %     2.7 %   (judging 18.6 % of envs)

    Zero demotions is the tell. The 2026-09-12_19-16-59 run reached `Curriculum/terrain_levels`
    5.98 of 10 and I read that as "it climbs". It meant "it walks". Measured afterwards on the
    training terrain with `training/measure_climb.py`: 0 of 64 robots gained even 10 cm of
    height under a forward command. The policy was promoted past terrain it had never learned
    to cross, so it never learned to cross any of it.

    HOW THIS RULE WORKS. Displacement is the right measure -- it is the only one that forces
    the robot off the platform -- and the reason it could not be used is that environments
    commanded to turn cannot displace. So it JUDGES ONLY the environments that were asked to
    travel roughly straight (`|yaw| < straight_yaw`, linear command above `min_speed`) and
    leaves every other environment's level untouched. Within that subset it is exactly the
    stock rule, which is proven on this terrain. About 19 % of environments are judged per
    episode, roughly 770 of 4096, and over a full run each environment is judged often enough
    to walk its level up or down.

    `straight_yaw` 0.4 rad/s is not "no turning": held for a 10 s resampling period that is
    229 degrees of arc. It is the widest band that still leaves a straight-line displacement
    achievable, and the Monte Carlo above is computed with it.
    """
    terrain = env.scene.terrain
    asset = env.scene["robot"]
    command_term = env.command_manager.get_term(command_name)

    cmd = env.command_manager.get_command(command_name)[env_ids]
    lin = torch.linalg.norm(cmd[:, :2], dim=1)
    judged = (lin > min_speed) & (cmd[:, 2].abs() < straight_yaw)

    disp = torch.linalg.norm(
        asset.data.root_pos_w[env_ids, :2] - env.scene.env_origins[env_ids, :2], dim=1
    )
    asked = command_term.metrics["commanded_path"][env_ids]

    move_up = judged & (disp > terrain.cfg.terrain_generator.size[0] / 2)
    move_down = judged & (disp < asked * demote_fraction) & ~move_up
    terrain.update_env_origins(env_ids, move_up, move_down)
    return torch.mean(terrain.terrain_levels.float())
