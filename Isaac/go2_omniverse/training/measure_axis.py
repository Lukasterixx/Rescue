"""Sweep the linear-velocity command across BOTH directions and report what is delivered.

WHY IT EXISTS. `measure_posture.py` commands 0.5 and 1.0 m/s FORWARD and nothing else, so a
policy that is fast backwards and slow forwards reads as merely slow. That is exactly the
report this was written to chase: driven in the maze, the robot "rarely moved forwards when
commanded to, and moved backwards often instead, extra fast".

WHAT IT SEPARATES. Run on the FLAT play task there are no walls, no stairs and no height
relief, so anything asymmetric here is the POLICY. If flat comes back symmetric, the fault is
in what the maze adds -- walls in the height scan -- and not in the policy.

IT ALSO DUMPS THE HEIGHT SCAN, because the observation's last 187 values are the only part of
the input the maze changes, and they are worth looking at directly before blaming them. Watch
the `clip` column: `height_scan` returns `sensor.pos_w.z - hit.z - 0.5`, and the scanner sits
20 m above the base, so the raw value is ~20 and the observation term's clip=(-1, 1) may be
saturating every ray. A column of 100 % means the policy is blind and the terrain part of its
input is a constant.

Pinning the command takes the same three settings `measure_posture.py` documents: heading
control off, standing envs zeroed, resampling pushed past the run. Getting any of them wrong
silently measures a different robot.

    cd ~/IsaacLab && ./isaaclab.sh -p ~/Rescue/Isaac/go2_omniverse/training/measure_axis.py \
      --task Isaac-Velocity-Rescue-Unitree-Go2-Flat-Play-v0 \
      --checkpoint ~/IsaacLab/logs/rsl_rl/go2_rescue/<run>/model_3999.pt \
      --num_envs 32 --headless --out axis.txt
"""

import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, required=True)
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=32)
parser.add_argument("--out", type=str, default="axis.txt")
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

    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(retrieve_file_path(args_cli.checkpoint))
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    robot = env.unwrapped.scene["robot"]
    vel_cmd = env.unwrapped.command_manager.get_command("base_velocity")

    # The policy observation is [lin_vel 3, ang_vel 3, gravity 3, command 3, joint_pos 12,
    # joint_vel 12, actions 12, height_scan 187] = 235. Slice the scan off the end rather
    # than assuming 48, so this still reports correctly if the front ever changes.
    obs = env.get_observations()
    pol = obs["policy"] if hasattr(obs, "keys") else obs
    scan_n = 187
    scan_lo = pol.shape[1] - scan_n
    print(f"[OBS] width {pol.shape[1]}, height_scan assumed to be columns {scan_lo}..{pol.shape[1]-1}")

    blocks = []
    for vx in (-1.0, -0.75, -0.5, -0.25, 0.25, 0.5, 0.75, 1.0):
        blocks.append((f"vx {vx:+.2f}", (vx, 0.0, 0.0)))
    for vy in (-0.4, 0.4):
        blocks.append((f"vy {vy:+.2f}", (0.0, vy, 0.0)))

    settle, sample = 120, 250
    rows = []
    scan_rows = []
    for label, (vx, vy, wz) in blocks:
        vel_cmd[:, 0], vel_cmd[:, 1], vel_cmd[:, 2] = vx, vy, wz
        vel_hist, scan_hist = [], []
        for step in range(settle + sample):
            with torch.inference_mode():
                actions = policy(obs)
                obs, _, _, _ = env.step(actions)
            vel_cmd[:, 0], vel_cmd[:, 1], vel_cmd[:, 2] = vx, vy, wz
            if step < settle:
                continue
            vel_hist.append(
                torch.cat([robot.data.root_lin_vel_b[:, :2], robot.data.root_ang_vel_b[:, 2:3]], dim=1)
            )
            p = obs["policy"] if hasattr(obs, "keys") else obs
            scan_hist.append(p[:, scan_lo:])
        vels = torch.stack(vel_hist)                       # (T, E, 3)
        scans = torch.stack(scan_hist)                     # (T, E, 187)
        want = vx if vy == 0.0 else vy
        axis = 0 if vy == 0.0 else 1
        got = vels[..., axis].mean().item()
        rows.append((label, want, got, 100.0 * got / want if want else float("nan"),
                     vels[..., 0].mean().item(), vels[..., 1].mean().item(), vels[..., 2].mean().item()))
        at_hi = (scans >= 0.999).float().mean().item() * 100
        at_lo = (scans <= -0.999).float().mean().item() * 100
        scan_rows.append((label, scans.min().item(), scans.max().item(), scans.mean().item(), at_hi, at_lo))

    out = []
    out.append("=" * 104)
    out.append(f"{'command':>10} {'want':>8} {'got':>8} {'track':>8}   {'vx body':>8} {'vy body':>8} {'yaw body':>9}")
    out.append(f"{'':>10} {'m/s':>8} {'m/s':>8} {'%':>8}   {'m/s':>8} {'m/s':>8} {'rad/s':>9}")
    out.append("-" * 104)
    for r in rows:
        out.append(f"{r[0]:>10} {r[1]:8.2f} {r[2]:8.3f} {r[3]:8.1f}   {r[4]:8.3f} {r[5]:8.3f} {r[6]:9.3f}")
    out.append("=" * 104)
    out.append("")
    out.append("height_scan observation (the last 187 columns), per command block:")
    out.append(f"{'command':>10} {'min':>8} {'max':>8} {'mean':>8} {'at +1':>8} {'at -1':>8}")
    out.append("-" * 60)
    for r in scan_rows:
        out.append(f"{r[0]:>10} {r[1]:8.3f} {r[2]:8.3f} {r[3]:8.3f} {r[4]:7.1f}% {r[5]:7.1f}%")
    out.append("")
    out.append("'at +1' near 100% means the clip is saturating the scan and the policy is blind to terrain.")
    text = "\n".join(out)
    print(text)
    with open(args_cli.out, "w") as f:
        f.write(text + "\n")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
