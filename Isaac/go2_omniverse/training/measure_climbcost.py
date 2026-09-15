"""Do the reward terms I added to help with stairs actually PENALISE climbing?

THE SUSPICION. Two terms were added to this task to help it learn stairs:

  * `undesired_contacts` on calves and thighs, weight -1.0, on the reasoning that a shin
    resting on a step edge is how a climb stalls. Note that Isaac Lab's own Go2 rough config
    DISABLES this term, and that was overridden deliberately.
  * `foot_stumble`, weight -0.5, which fires when a foot's contact force is mostly horizontal
    -- a toe catching a riser.

Both may be self-defeating. On a 20 cm tread the Go2's shin has nowhere to be except near the
step edge, and stepping ONTO a riser begins with the toe touching its vertical face, which is
exactly what `foot_stumble` is built to detect. If so, the two terms charge the robot for the
act of climbing, climbing earns nothing extra in return (velocity tracking is no better
uphill), and not climbing is strictly the better policy.

HOW THIS TESTS IT WITHOUT A TRAINING RUN. Take a policy that DOES climb this terrain -- the
P2Dingo checkpoint, whose own reward set contains neither term -- run it on the Rescue task
whose reward manager does, and compare the penalty accrued by the robots that ascend against
those that do not. The terms are computed by the env either way; nothing acts on them during
playback, so this reads what training WOULD have charged.

If climbers pay substantially more, the two terms are a cause of the failure and they are
mine, not inherited.
"""

import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, required=True)
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--seconds", type=float, default=12.0)
parser.add_argument("--command", type=float, default=1.0)
parser.add_argument("--out", type=str, default="climbcost.txt")
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

# Terms to attribute. The first two are the suspects; the rest are context, so a difference
# in the suspects can be judged against how much everything else also differs on stairs.
WATCH = ["undesired_contacts", "foot_stumble", "base_motion", "base_orientation",
         "foot_slip", "action_smoothness", "base_linear_velocity", "gait", "foot_clearance"]


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
    env_cfg.episode_length_s = max(env_cfg.episode_length_s, args_cli.seconds * 3.0)

    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(retrieve_file_path(args_cli.checkpoint))
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    robot = env.unwrapped.scene["robot"]
    rm = env.unwrapped.reward_manager
    vel_cmd = env.unwrapped.command_manager.get_command("base_velocity")

    available = list(rm.active_terms)
    watch = [t for t in WATCH if t in available]
    missing = [t for t in WATCH if t not in available]
    print(f"[TERMS] watching {watch}")
    if missing:
        print(f"[TERMS] not in this task: {missing}")

    vx = args_cli.command
    def hold():
        vel_cmd[:, 0], vel_cmd[:, 1], vel_cmd[:, 2] = vx, 0.0, 0.0

    obs = env.get_observations()
    hold()
    for _ in range(60):
        with torch.inference_mode():
            obs, _, _, _ = env.step(policy(obs))
        hold()

    z0 = robot.data.root_pos_w[:, 2].clone()
    dt = env.unwrapped.step_dt
    steps = int(args_cli.seconds / dt)
    # Accumulate each watched term per environment, by re-reading the manager's running
    # episode sums and differencing them. `_episode_sums` holds the WEIGHTED contribution.
    start = {t: rm._episode_sums[t].clone() for t in watch}
    for _ in range(steps):
        with torch.inference_mode():
            obs, _, _, _ = env.step(policy(obs))
        hold()
    acc = {t: rm._episode_sums[t] - start[t] for t in watch}
    dz = robot.data.root_pos_w[:, 2] - z0

    climbed = dz > 0.05
    descended = dz < -0.05
    level = ~climbed & ~descended
    groups = [("ASCENDED >5cm", climbed), ("level ground", level), ("descended >5cm", descended)]

    out = [f"{args_cli.task}",
           f"policy: {args_cli.checkpoint.split('/')[-3]}/{args_cli.checkpoint.split('/')[-1]}",
           f"forward {vx:+.2f} m/s, {args_cli.seconds:.0f} s, {args_cli.num_envs} robots",
           "",
           "Reward accrued per robot over the window (already weighted), split by what the",
           "robot actually did. A term far more negative for ASCENDED than for the others is a",
           "term that charges the robot for climbing.",
           "=" * 100,
           f"{'term':>22}" + "".join(f"{g[0]:>19}" for g in groups),
           f"{'':>22}" + "".join(f"{'n=' + str(int(g[1].sum().item())):>19}" for g in groups),
           "-" * 100]
    for t in watch:
        row = f"{t:>22}"
        for _, mask in groups:
            v = acc[t][mask].mean().item() if bool(mask.any()) else float("nan")
            row += f"{v:19.3f}"
        out.append(row)
    out.append("-" * 100)
    tot = {name: sum(acc[t][mask].mean().item() for t in watch if bool(mask.any()))
           for name, mask in groups}
    out.append(f"{'sum of watched':>22}" + "".join(f"{tot[g[0]]:19.3f}" for g in groups))
    out.append("=" * 100)
    if bool(climbed.any()) and bool(level.any()):
        for t in ("undesired_contacts", "foot_stumble"):
            if t in acc:
                c, l = acc[t][climbed].mean().item(), acc[t][level].mean().item()
                out.append(f"{t}: climbers pay {c:.3f}, non-climbers {l:.3f} "
                           f"-> climbing costs an extra {c - l:+.3f}")
    text = "\n".join(out)
    print(text)
    with open(args_cli.out, "w") as f:
        f.write(text + "\n")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
