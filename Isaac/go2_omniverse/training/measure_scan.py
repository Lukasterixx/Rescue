"""Does a WALL in the height scan make the policy refuse to go forward?

THE REPORT THIS CHASES. Driven in the maze the robot "rarely moved forwards when commanded
to, and moved backwards often instead". `measure_axis.py` on flat ground does NOT reproduce
that: commanded +1.0 m/s the policy delivers +0.75, slow but forward. So the cause is
something the maze adds, and the only part of the observation the maze changes is the height
scan -- a 1.2 m corridor with 1.0 m walls, against a training terrain whose tallest feature
was a 0.16 m riser.

HOW IT TESTS THAT WITHOUT BUILDING THE MAZE. The policy is run on FLAT ground and the height
scan is OVERWRITTEN in the observation before the policy sees it, every step. Nothing else
differs between the conditions, so any change in the delivered speed is caused by the scan
pattern alone. That is a cleaner experiment than driving the maze, where the walls, the
stairs, the corridor and the operator all vary at once.

THE SCAN'S LAYOUT, which the injection depends on. `GridPatternCfg(resolution=0.1,
size=[1.6, 1.0])` with the default ordering "xy" builds `meshgrid(x, y, indexing="xy")`,
so the 187 values reshape to (11, 17) = (lateral y, longitudinal x). Column 16 is 0.8 m
AHEAD of the base, column 0 is 0.8 m behind, row 0 and row 10 are 0.5 m to either side.

WHAT A WALL READS AS. `height_scan` is `sensor.pos_w.z - hit.z - 0.5`, so HIGHER ground is a
MORE NEGATIVE value, and the observation clips at -1.0. Measured on flat, the scan sits at
about -0.16. A 1.0 m wall top drives it past -1.0 and clips. In training the tallest riser
was 0.16 m, which reads about -0.33 -- so -1.0 is roughly three times beyond anything the
policy ever saw, and "-1.0 ahead" means "an impossibly tall step". Backing away from it is a
plausible extrapolation, and this measures whether that is what happens.
"""

import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, required=True)
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=32)
parser.add_argument("--command", type=float, default=1.0, help="forward command, m/s")
parser.add_argument("--out", type=str, default="scan.txt")
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

NY, NX = 11, 17          # lateral, longitudinal
WALL = -1.0              # what a 1 m wall clips to


def patched_scan(scan, pattern, value):
    """Return a patched copy of the (E, 187) scan, plus how many rays changed per robot.

    A COPY, NOT AN IN-PLACE EDIT. The observation comes out of `env.step` under
    `torch.inference_mode()`, so its tensors are inference tensors and PyTorch refuses to
    mutate them afterwards. Building a new tensor and assigning it into a CLONE of the
    observation is the supported way round that.

    `value` is what the obstacle reads as. -1.0 is the raw clip a 1 m wall produces; the
    point of sweeping it is that CLAMPING the scan in the sim is the proposed fix, and a
    clamp is exactly equivalent to the wall reading a less negative number.
    """
    g = scan.reshape(-1, NY, NX).clone()
    before = g.clone()
    if pattern == "ahead":
        g[:, :, 13:] = value
    elif pattern == "corridor":
        g[:, :2, :] = value
        g[:, -2:, :] = value
    elif pattern == "corridor+ahead":
        g[:, :2, :] = value
        g[:, -2:, :] = value
        g[:, :, 13:] = value
    n = int((g != before).sum().item() / max(1, g.shape[0]))
    return g.reshape(scan.shape), n


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

    obs = env.get_observations()
    pol = obs["policy"] if hasattr(obs, "keys") else obs
    scan_lo = pol.shape[1] - NY * NX
    assert scan_lo == 48, f"expected the scan to start at column 48, got {scan_lo}"

    vx = args_cli.command
    # A clamp in the sim is the same thing as the wall reading a less negative value, so
    # sweeping `value` IS sweeping the clamp. The constraint is that legitimate stairs also
    # read negative: the maze's own 10 cm riser on a 20 cm tread reaches about -0.57 at the
    # front of the scan, and the steepest stair the policy trained on reaches about -0.81.
    # A clamp has to sit above the stairs it must still see and below the wall it must ignore.
    blocks = [("baseline", None, 0.0)]
    for v in (-1.00, -0.85, -0.70, -0.55, -0.40):
        blocks.append((f"ahead @ {v:+.2f}", "ahead", v))
    for v in (-1.00, -0.70, -0.55, -0.40):
        blocks.append((f"corridor @ {v:+.2f}", "corridor", v))
    for v in (-0.70, -0.55):
        blocks.append((f"corr+ahead @ {v:+.2f}", "corridor+ahead", v))

    settle, sample = 120, 250
    rows = []
    for label, pattern, value in blocks:
        vel_cmd[:, 0], vel_cmd[:, 1], vel_cmd[:, 2] = vx, 0.0, 0.0
        vel_hist, nrays = [], 0
        for step in range(settle + sample):
            if pattern is None:
                obs_in, nrays = obs, 0
            else:
                obs_in = obs.clone()
                tgt = obs_in["policy"] if hasattr(obs_in, "keys") else obs_in
                new_scan, nrays = patched_scan(tgt[:, scan_lo:], pattern, value)
                tgt[:, scan_lo:] = new_scan
            with torch.inference_mode():
                actions = policy(obs_in)
                obs, _, _, _ = env.step(actions)
            vel_cmd[:, 0], vel_cmd[:, 1], vel_cmd[:, 2] = vx, 0.0, 0.0
            if step >= settle:
                vel_hist.append(
                    torch.cat([robot.data.root_lin_vel_b[:, :2], robot.data.root_ang_vel_b[:, 2:3]], dim=1)
                )
        v = torch.stack(vel_hist)
        rows.append((label, nrays, v[..., 0].mean().item(), v[..., 1].mean().item(), v[..., 2].mean().item(),
                     (v[..., 0] < 0).float().mean().item() * 100))

    out = [f"forward command {vx:+.2f} m/s on FLAT ground; only the height scan differs between rows",
           "=" * 92,
           f"{'scan condition':>20} {'rays':>6} {'vx got':>9} {'vy got':>9} {'yaw got':>9} {'% of time moving back':>22}",
           "-" * 92]
    for r in rows:
        out.append(f"{r[0]:>20} {r[1]:6d} {r[2]:9.3f} {r[3]:9.3f} {r[4]:9.3f} {r[5]:21.1f}%")
    out.append("=" * 92)
    base = rows[0][2]
    out.append(f"baseline delivers {base:+.3f} m/s. A row far below that is the scan pattern stopping the robot;")
    out.append("a NEGATIVE row is the robot actively retreating from terrain it reads as impassable.")
    text = "\n".join(out)
    print(text)
    with open(args_cli.out, "w") as f:
        f.write(text + "\n")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
