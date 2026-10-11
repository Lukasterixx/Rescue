"""How well an exported K-rail policy tracks held velocity commands, on a plane or on K-rail tiles of one difficulty.

    ./isaaclab.sh -p ~/Rescue/Isaac/go2_omniverse/training/measure_tracking.py --headless \\
        --policy policies/go2_krail_2026-10-11 [--terrain flat | krail --difficulty 0.45] --out tracking.json

Each command in COMMANDS is held by `--reps` robots for `--seconds` (default 12 s); the first 3 s are dropped and the
rest averaged. Velocities are in the yaw frame, as nav2 commands them: `ratio` is achieved over commanded on the
commanded axis, `abs_err` the mean absolute error on each of (vx, vy, wz), and `drift` the linear speed of an
in-place turn or a stand. `fell` counts robots that ended their episode early (base or head contact, or tipped past
1 rad).

The robot and the observation are the training task's PLAY configuration (`go2_rescue/krail_env_cfg.py`): the sim's
welded robot with the arm at rest, no observation noise, friction 0.8 / 0.6 as the sim has it, no pushes, and here
no base-mass or centre-of-mass randomisation, so it is the nominal robot the sim drives. The policy is the bundle's
TorchScript, as the sim runs it.

`--terrain krail` puts every robot on `krail_decks` and `krail_field` tiles at `--difficulty` (0.45 is about the
square: 12.5 cm decks, 13 cm rails, each +-15 %); `--terrain steps_up` in the pit of the up-stairs (60 cm treads, a riser of
0.03 + 0.19 x difficulty m), where every heading climbs. `climbed` counts robots whose base rose `--climb-m` above
where it stood after settling, `climbed_and_upright` those that also never fell.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
parser.add_argument("--policy", required=True)
parser.add_argument("--terrain", choices=("flat", "krail", "steps_up"), default="flat")
parser.add_argument("--commands", default=None, help="comma-separated subset of COMMANDS (default all)")
parser.add_argument("--climb-m", type=float, default=0.06,
                    help="a robot 'climbed' if its base rose this far above where it stood after settling")
parser.add_argument("--difficulty", type=float, default=0.45)
parser.add_argument("--reps", type=int, default=16)
parser.add_argument("--seconds", type=float, default=12.0)
parser.add_argument("--out", default=None)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab_tasks  # noqa: E402,F401
from isaaclab.terrains import TerrainImporterCfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

COMMANDS = {
    "fwd_0.3": (0.3, 0.0, 0.0), "fwd_0.5": (0.5, 0.0, 0.0), "fwd_0.8": (0.8, 0.0, 0.0), "fwd_1.0": (1.0, 0.0, 0.0),
    "back_0.5": (-0.5, 0.0, 0.0), "left_0.4": (0.0, 0.4, 0.0), "right_0.4": (0.0, -0.4, 0.0),
    "turn_0.5": (0.0, 0.0, 0.5), "turn_1.0": (0.0, 0.0, 1.0), "turn_1.5": (0.0, 0.0, 1.5), "turn_-1.0": (0.0, 0.0, -1.0),
    "arc_0.5_0.5": (0.5, 0.0, 0.5), "stand": (0.0, 0.0, 0.0),
}
SETTLE_S = 3.0


@torch.inference_mode()
def main():
    names = args.commands.split(",") if args.commands else list(COMMANDS)
    n = len(names) * args.reps
    cfg = parse_env_cfg("Isaac-Velocity-Rescue-KRail-Go2-Play-v0", device=args.device, num_envs=n)
    cfg.events.add_base_mass = None
    cfg.events.base_com = None
    cfg.episode_length_s = args.seconds + 1.0
    cfg.curriculum.terrain_levels = None
    cfg.commands.base_velocity.resampling_time_range = (1.0e9, 1.0e9)
    cfg.commands.base_velocity.heading_command = False
    cfg.commands.base_velocity.rel_heading_envs = 0.0
    cfg.commands.base_velocity.rel_standing_envs = 0.0
    cfg.commands.base_velocity.rel_turning_envs = 0.0
    cfg.commands.base_velocity.debug_vis = False
    if args.terrain == "flat":
        cfg.scene.terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane",
                                               physics_material=cfg.scene.terrain.physics_material)
    else:
        gen = cfg.scene.terrain.terrain_generator
        keep = ("krail",) if args.terrain == "krail" else ("steps_up",)
        gen.sub_terrains = {k: v for k, v in gen.sub_terrains.items() if k.startswith(keep)}
        gen.difficulty_range = (args.difficulty, args.difficulty)
        gen.num_rows, gen.num_cols, gen.curriculum = 4, 8, False
    env = gym.make("Isaac-Velocity-Rescue-KRail-Go2-Play-v0", cfg=cfg)
    core = env.unwrapped
    policy = torch.jit.load(str(Path(args.policy).expanduser() / "policy.pt"), map_location=core.device).eval()
    term = core.command_manager.get_term("base_velocity")
    target = torch.tensor([COMMANDS[k] for k in names for _ in range(args.reps)], device=core.device)
    robot = core.scene["robot"]

    obs, _ = env.reset()
    term.vel_command_b[:] = target
    obs = core.observation_manager.compute_group("policy")
    steps = int(args.seconds / core.step_dt)
    settle = int(SETTLE_S / core.step_dt)
    alive = torch.ones(n, dtype=torch.bool, device=core.device)
    sums = torch.zeros(n, 3, device=core.device)
    abs_err = torch.zeros(n, 3, device=core.device)
    count = 0
    start_z = torch.zeros(n, device=core.device)
    rise = torch.zeros(n, device=core.device)
    with torch.inference_mode():
        for k in range(steps):
            term.vel_command_b[:] = target
            obs = core.observation_manager.compute_group("policy")
            obs, _, terminated, truncated, _ = env.step(policy(obs))
            alive &= ~(terminated & (k < steps - 1))
            if k == settle:
                start_z[:] = robot.data.root_pos_w[:, 2]
            if k >= settle:
                rise = torch.maximum(rise, robot.data.root_pos_w[:, 2] - start_z)
                yaw = robot.data.heading_w
                c, s = torch.cos(yaw), torch.sin(yaw)
                vw = robot.data.root_lin_vel_w
                v = torch.stack([c * vw[:, 0] + s * vw[:, 1], -s * vw[:, 0] + c * vw[:, 1],
                                 robot.data.root_ang_vel_w[:, 2]], dim=1)
                sums += v
                abs_err += (v - target).abs()
                count += 1
    mean_v = (sums / count).double().cpu().numpy()
    err = (abs_err / count).double().cpu().numpy()
    alive = alive.cpu().numpy()
    rise = rise.cpu().numpy()
    results = {}
    for i, name in enumerate(names):
        rows = slice(i * args.reps, (i + 1) * args.reps)
        cmd = np.array(COMMANDS[name])
        ok = alive[rows]
        v = mean_v[rows][ok] if ok.any() else mean_v[rows]
        e = err[rows][ok] if ok.any() else err[rows]
        axis = int(np.argmax(np.abs(cmd))) if np.abs(cmd).max() > 0 else None
        results[name] = {
            "command": cmd.tolist(), "mean_velocity": v.mean(0).round(3).tolist(),
            "abs_err": e.mean(0).round(3).tolist(), "fell": int((~ok).sum()), "of": args.reps,
            "ratio": round(float(v[:, axis].mean() / cmd[axis]), 3) if axis is not None else None,
            "drift_mps": round(float(np.linalg.norm(v[:, :2], axis=1).mean()), 3) if cmd[0] == cmd[1] == 0 else None,
            "climbed": int((rise[rows] > args.climb_m).sum()),
            "climbed_and_upright": int(((rise[rows] > args.climb_m) & ok).sum()),
            "max_rise_m": round(float(rise[rows].max()), 3),
        }
        print(f"[tracking] {name:12s} {json.dumps(results[name])}", flush=True)
    out = {"policy": args.policy, "terrain": args.terrain,
           "difficulty": args.difficulty if args.terrain == "krail" else None, "seconds": args.seconds,
           "results": results}
    if args.out:
        Path(args.out).write_text(json.dumps(out, indent=1) + "\n")


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        import traceback

        traceback.print_exc()
        sys.stderr.flush()
        sys.stdout.flush()
        os._exit(1)
    # Kit's close hangs after these runs (and can swallow the traceback, above); the results are already written.
    sys.stdout.flush()
    os._exit(0)
