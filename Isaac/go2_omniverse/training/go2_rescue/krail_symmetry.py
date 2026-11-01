"""Left-right mirror of the K-rail task's observations and actions, for rsl-rl's symmetry augmentation.

WHY. Turning was lopsided and the side swapped between checkpoints of run v2, on flat ground with nothing else
changed: at iteration 1000 the policy turned left at 49-74 % of 0.5-1.5 rad/s and right at 92 % of 1.0; at 2000 left
at 85-100 % and right at 16 % of 1.0 rad/s (`measure_tracking.py`, 8 robots each). A policy trained on each
transition and on its mirror image learns both turns from the same data. Front-back is not used: the Go2's knees,
its head and the folded arm make it a different robot backwards.

The mirror is about the base's x-z plane. The robot is not quite symmetric about it (the D1's shoulder sits 28 mm off
its axis and the folded arm with it), so the mirror is a slightly different robot, as the base-mass and
centre-of-mass randomisation already is.

The layout is `krail_env_cfg.KRailObservationsCfg`'s, the same in both groups: base_lin_vel 3, base_ang_vel 3,
projected_gravity 3, velocity_commands 3, then the 12 legs' positions, velocities and last actions in Isaac Lab's
order (FL, FR, RL, RR hips, then thighs, then calves), then the height scan, an x-fastest grid ("xy" ordering).

THE GO2-W (`krail_go2w_env_cfg.py`) has four wheels after the legs in the velocities and the actions (FL, FR, RL, RR),
so those two blocks are 16 wide; how many there are is read off the action manager. A wheel mirrors to the wheel
opposite with the same sign: all four turn about their +y axle, and a spin about the axis the mirror flips is
unchanged by it (an angular velocity is a pseudovector), so a left wheel rolling forward mirrors to a right wheel
rolling forward.
"""
from __future__ import annotations

import torch

# Isaac Lab's leg order; mirroring swaps FL<->FR and RL<->RR and negates the hips (abduction). The default pose's
# hips are +-0.1, so a position relative to it mirrors by the same rule.
_MIRROR_INDEX = [1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10]
_MIRROR_SIGN = [-1.0] * 4 + [1.0] * 8
_WHEEL_MIRROR_INDEX = [1, 0, 3, 2]
# (lin vel, ang vel, gravity, command): y, roll, yaw, gravity's y, the lateral and yaw commands change sign.
_BASE_SIGN = [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0, -1.0]


def _mirror_legs(x: torch.Tensor) -> torch.Tensor:
    return x[..., _MIRROR_INDEX] * torch.tensor(_MIRROR_SIGN, device=x.device, dtype=x.dtype)


def _mirror_joints(x: torch.Tensor, wheels: int) -> torch.Tensor:
    """The legs' 12 columns, then `wheels` wheel columns (0 or 4)."""
    legs = _mirror_legs(x[..., :12])
    return torch.cat([legs, x[..., 12:][..., _WHEEL_MIRROR_INDEX]], dim=-1) if wheels else legs


def _wheels(env) -> int:
    wheels = env.unwrapped.action_manager.total_action_dim - 12
    if wheels not in (0, 4):
        raise ValueError(f"the mirror knows the Go2's 12 actions and the Go2-W's 16, not {12 + wheels}")
    return wheels


def _scan_shape(env) -> tuple[int, int]:
    pattern = env.unwrapped.scene["height_scanner"].cfg.pattern_cfg
    if pattern.ordering != "xy":
        raise ValueError(f"the scan mirror assumes 'xy' ordering, got {pattern.ordering!r}")
    nx = int(round(pattern.size[0] / pattern.resolution)) + 1
    ny = int(round(pattern.size[1] / pattern.resolution)) + 1
    return ny, nx


def mirror_observation(env, obs: torch.Tensor) -> torch.Tensor:
    ny, nx = _scan_shape(env)
    w = _wheels(env)
    joints = 48 + 2 * w
    if obs.shape[-1] != joints + ny * nx:
        raise ValueError(f"observation is {obs.shape[-1]} wide, expected {joints} + {ny} x {nx}")
    out = obs.clone()
    out[..., :12] = obs[..., :12] * torch.tensor(_BASE_SIGN, device=obs.device, dtype=obs.dtype)
    out[..., 12:24] = _mirror_legs(obs[..., 12:24])                      # positions: the legs only
    for start in (24, 36 + w):                                            # velocities, then last actions
        out[..., start:start + 12 + w] = _mirror_joints(obs[..., start:start + 12 + w], w)
    # Rows are y from -size/2 to +size/2, symmetric about the base, so the mirror flips the row order.
    scan = obs[..., joints:].reshape(*obs.shape[:-1], ny, nx).flip(dims=[-2])
    out[..., joints:] = scan.reshape(*obs.shape[:-1], ny * nx)
    return out


@torch.no_grad()
def compute_symmetric_states(env, obs=None, actions=None):
    """rsl-rl's `data_augmentation_func`: the batch, then its mirror image (twice the rows)."""
    obs_aug = None
    if obs is not None:
        batch = obs.batch_size[0]
        obs_aug = obs.repeat(2)
        for group in obs.keys():
            obs_aug[group][:batch] = obs[group]
            obs_aug[group][batch:] = mirror_observation(env, obs[group])
    actions_aug = None
    if actions is not None:
        actions_aug = torch.cat([actions, _mirror_joints(actions, _wheels(env))], dim=0)
    return obs_aug, actions_aug
