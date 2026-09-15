"""Can the policy climb the stairs it TRAINED on? Height gained under a forward command.

WHY. Driven in the maze the robot would not climb in any direction. That has two possible
causes needing completely different fixes: either the policy never learned to climb and the
training result is misleading, or it climbs its own terrain fine and something about the maze
stops it. This measures the first directly, with the maze out of the picture.

WHAT IT REPORTS. Height gained and distance walked over a fixed window under a pinned forward
command, on the rough play task. The training curriculum's own promotion rule asks for half a
terrain patch -- 4 m -- in a 20 s episode, so `path` is directly comparable with the quantity
that drove `Curriculum/terrain_levels` to 6 during training. On the inverted-pyramid tiles the
robot starts in the pit and every direction out is up, so a climbing policy gains height.

It also prints the height-scan range actually encountered. That is the number to hold against
the maze: if the maze presents values the training terrain never does, that is the transfer
gap, and no amount of gait tuning closes it.
"""

import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, required=True)
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--seconds", type=float, default=12.0)
parser.add_argument("--friction", type=float, default=None,
                    help="pin foot friction (static; dynamic = 0.75x) instead of the config's range")
parser.add_argument("--out", type=str, default="climb.txt")
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
sys.argv = [sys.argv[0]]
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
import importlib.metadata as _md
import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg
from isaaclab.utils.assets import retrieve_file_path
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    agent_cfg = load_cfg_from_registry(args_cli.task, "rsl_rl_cfg_entry_point")
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, _md.version("rsl-rl-lib"))

    cmd = env_cfg.commands.base_velocity
    cmd.heading_command = False
    cmd.rel_standing_envs = 0.0
    if hasattr(cmd, "rel_turning_envs"):
        cmd.rel_turning_envs = 0.0
    cmd.resampling_time_range = (1.0e6, 1.0e6)
    cmd.ranges.lin_vel_x = (0.0, 0.0)
    cmd.ranges.lin_vel_y = (0.0, 0.0)
    cmd.ranges.ang_vel_z = (0.0, 0.0)
    if hasattr(cmd.ranges, "heading"):
        cmd.ranges.heading = (0.0, 0.0)
    # A reset mid-window would reposition the robot and destroy the displacement measurement.
    env_cfg.episode_length_s = max(env_cfg.episode_length_s, args_cli.seconds * 3.0)

    # PIN THE FOOT FRICTION. Training randomises it over 0.3-1.2, a range taken from a stack
    # that trains on FLAT ground. On a 20 cm tread a foot at 0.3 slides off, so this sweeps
    # whether low friction makes stairs merely harder or outright impossible -- which decides
    # whether that randomisation is teaching robustness or teaching the robot to avoid stairs.
    if args_cli.friction is not None:
        f = args_cli.friction
        env_cfg.events.physics_material.params.update({
            "static_friction_range": (f, f),
            "dynamic_friction_range": (0.75 * f, 0.75 * f),
            "restitution_range": (0.0, 0.0),
        })

    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(retrieve_file_path(args_cli.checkpoint))
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    robot = env.unwrapped.scene["robot"]

    obs = env.get_observations()
    pol = obs["policy"] if hasattr(obs, "keys") else obs
    scan_lo = pol.shape[1] - 187
    dt = env.unwrapped.step_dt
    steps = int(args_cli.seconds / dt)
    vel_cmd = env.unwrapped.command_manager.get_command("base_velocity")

    def hold(vx):
        vel_cmd[:, 0], vel_cmd[:, 1], vel_cmd[:, 2] = vx, 0.0, 0.0

    rows = []
    for label, vx in (("forward 0.5", 0.5), ("forward 1.0", 1.0)):
        hold(vx)
        for _ in range(60):                       # settle before taking the datum
            with torch.inference_mode():
                obs, _, _, _ = env.step(policy(obs))
            hold(vx)
        z0 = robot.data.root_pos_w[:, 2].clone()
        xy0 = robot.data.root_pos_w[:, :2].clone()
        path = torch.zeros_like(z0)
        smin, smax = 10.0, -10.0
        for _ in range(steps):
            with torch.inference_mode():
                obs, _, _, _ = env.step(policy(obs))
            hold(vx)
            path += torch.linalg.norm(robot.data.root_lin_vel_b[:, :2], dim=1) * dt
            p = obs["policy"] if hasattr(obs, "keys") else obs
            sc = p[:, scan_lo:]
            smin, smax = min(smin, sc.min().item()), max(smax, sc.max().item())
        dz = robot.data.root_pos_w[:, 2] - z0
        disp = torch.linalg.norm(robot.data.root_pos_w[:, :2] - xy0, dim=1)
        # POOLED MEAN HEIGHT IS NEARLY USELESS HERE and that is worth saying once: the play
        # terrain mixes ascending (inverted pyramid) and descending (pyramid) tiles, so
        # climbers and descenders cancel. Report the ASCENT side separately -- the 90th
        # percentile and the share clearing one step -- and at a 5 cm threshold as well as
        # 10 cm, because early in a run the terrain level is low and the steps are ~6 cm, so a
        # 10 cm gate reads zero while the robot is in fact climbing.
        rows.append((label,
                     dz.mean().item(),
                     torch.quantile(dz, 0.90).item(),
                     dz.max().item(),
                     (dz > 0.05).float().mean().item() * 100,
                     (dz > 0.1).float().mean().item() * 100,
                     (dz < -0.1).float().mean().item() * 100,
                     path.mean().item(), disp.mean().item(), smin, smax))

    out = [f"{args_cli.task}" + (f"   foot friction pinned at {args_cli.friction}" if args_cli.friction is not None else ""),
           f"{args_cli.seconds:.0f} s under a pinned forward command, {args_cli.num_envs} robots",
           "=" * 124,
           f"{'command':>12} {'dz mean':>8} {'dz p90':>7} {'dz best':>8} {'rose>5cm':>9} "
           f"{'rose>10cm':>10} {'fell>10cm':>10} {'path':>7} {'net disp':>9} {'scan min':>9} {'scan max':>9}",
           f"{'':>12} {'m':>8} {'m':>7} {'m':>8} {'%':>9} {'%':>10} {'%':>10} {'m':>7} {'m':>9} {'':>9} {'':>9}",
           "-" * 124]
    for r in rows:
        out.append(f"{r[0]:>12} {r[1]:8.3f} {r[2]:7.3f} {r[3]:8.3f} {r[4]:9.1f} "
                   f"{r[5]:10.1f} {r[6]:10.1f} {r[7]:7.2f} {r[8]:9.2f} {r[9]:9.3f} {r[10]:9.3f}")
    out.append("=" * 124)
    out.append("The training curriculum promotes on 4 m of path in 20 s, so 'path' here is the")
    out.append("same quantity that drove the terrain level to 6. 'rose>10cm' is the share of robots")
    out.append("that gained a step's worth of height, i.e. actually climbed something.")
    text = "\n".join(out)
    print(text)
    with open(args_cli.out, "w") as f:
        f.write(text + "\n")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
