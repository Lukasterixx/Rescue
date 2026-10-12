"""How an exported K-rail policy carries its weight: the front/rear split, the pitch, and what a stop does.

    ./isaaclab.sh -p ~/Rescue/Isaac/go2_omniverse/training/measure_stance.py --headless \\
        --policy policies/go2_krail_2026-10-13 --out stance.json

Each scenario in SCENARIOS is run by `--reps` robots on a plane: the command is held for `--walk-s` (default 4 s),
then set to zero, as nav2 and a teleop stop, and held at zero for `--stop-s` (5 s). `stand` is zero throughout.

Per scenario, over the robots that did not fall:

* `walk`: the last 2 s of the walk. `front_share` is the front feet's part of the four feet's vertical contact force,
  `pitch_deg` the base's pitch (nose down positive), `com_ahead_m` how far the whole robot's centre of mass is
  ahead of the midpoint between the front feet's and the rear feet's mean x (in the yaw frame; 0 is centred).
* `stop`: from the zero command on. `pitch_max_deg` the most nose-down, `rear_air_s` the longest time both rear
  feet were off the ground together (a handstand), `rear_lift_m` the highest a rear foot rose, `margin_min_m` the
  least distance the centre of mass stayed behind the front feet's mean x (negative is ahead of them: tipping),
  `stop_m` how far the base travelled after the command went to zero.
* `rest`: the last 2 s, standing. `front_share`, `pitch_deg`, `com_ahead_m` as for the walk; `static_front_share`
  what the feet's positions and the centre of mass alone give for the front feet's share, standing still;
  `feet_down` the share of time all four feet touched; `pose_dev_rad` the summed |joint - default|; `height_m` the
  base's height above its feet's mean (0.33 m in the default pose; also given for the walk), `half_width_m` the
  feet's mean distance from the base's centre line (0.13 m in the default pose).
* `rear_air_robots` counts robots with a rear_air_s over 0.1 s; `fell` those that ended their episode early.

The robot and the observation are the training task's PLAY configuration, nominal (no mass or centre-of-mass
randomisation), as in `measure_tracking.py`.
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
parser.add_argument("--scenarios", default=None, help="comma-separated subset of SCENARIOS (default all)")
parser.add_argument("--reps", type=int, default=16)
parser.add_argument("--walk-s", type=float, default=4.0)
parser.add_argument("--stop-s", type=float, default=5.0)
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

SCENARIOS = {
    "stand": (0.0, 0.0, 0.0), "stop_fwd_0.3": (0.3, 0.0, 0.0), "stop_fwd_0.5": (0.5, 0.0, 0.0),
    "stop_fwd_0.8": (0.8, 0.0, 0.0), "stop_fwd_1.0": (1.0, 0.0, 0.0), "stop_back_0.5": (-0.5, 0.0, 0.0),
    "stop_back_1.0": (-1.0, 0.0, 0.0), "stop_left_0.4": (0.0, 0.4, 0.0), "stop_left_0.8": (0.0, 0.8, 0.0),
    "stop_turn_1.0": (0.0, 0.0, 1.0), "stop_arc_0.5_0.5": (0.5, 0.0, 0.5),
}
# The sim's keyboard drives at 1.0 m/s and 1.0 rad/s (sideways clamped to the policy's 0.8) and lets go to zero.
CONTACT_N = 1.0
TASK = "Isaac-Velocity-Rescue-KRail-Go2-Play-v0"


@torch.inference_mode()
def main():
    names = args.scenarios.split(",") if args.scenarios else list(SCENARIOS)
    n = len(names) * args.reps
    cfg = parse_env_cfg(TASK, device=args.device, num_envs=n)
    cfg.events.add_base_mass = None
    cfg.events.base_com = None
    cfg.episode_length_s = args.walk_s + args.stop_s + 1.0
    cfg.curriculum.terrain_levels = None
    cfg.commands.base_velocity.resampling_time_range = (1.0e9, 1.0e9)
    cfg.commands.base_velocity.heading_command = False
    cfg.commands.base_velocity.rel_heading_envs = 0.0
    cfg.commands.base_velocity.rel_standing_envs = 0.0
    cfg.commands.base_velocity.rel_turning_envs = 0.0
    cfg.commands.base_velocity.debug_vis = False
    cfg.scene.terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane",
                                           physics_material=cfg.scene.terrain.physics_material)
    env = gym.make(TASK, cfg=cfg)
    core = env.unwrapped
    policy = torch.jit.load(str(Path(args.policy).expanduser() / "policy.pt"), map_location=core.device).eval()
    term = core.command_manager.get_term("base_velocity")
    robot = core.scene["robot"]
    sensor = core.scene["contact_forces"]
    dev = core.device

    feet_s, feet_names = sensor.find_bodies(".*_foot", preserve_order=True)
    feet_r, feet_names_r = robot.find_bodies(".*_foot", preserve_order=True)
    assert feet_names == feet_names_r, (feet_names, feet_names_r)
    front = torch.tensor([nm.startswith("F") for nm in feet_names], device=dev)
    rear = ~front
    legs, _ = robot.find_joints([".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"], preserve_order=False)
    masses = robot.root_physx_view.get_masses().to(dev)  # (n, bodies)
    total_mass = float(masses[0].sum())

    moving = torch.tensor([SCENARIOS[k] for k in names for _ in range(args.reps)], device=dev)
    zero = torch.zeros_like(moving)

    def yaw_frame(p):
        """World points (n, k, 3) to the base's yaw frame, relative to the base."""
        yaw = robot.data.heading_w
        c, s = torch.cos(yaw).unsqueeze(1), torch.sin(yaw).unsqueeze(1)
        d = p - robot.data.root_pos_w.unsqueeze(1)
        return torch.stack([c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1], d[..., 2]], dim=-1)

    def sample():
        fz = sensor.data.net_forces_w[:, feet_s, 2].clamp(min=0.0)
        total = fz.sum(1)
        share = torch.where(total > 1.0, (fz * front).sum(1) / total.clamp(min=1e-6),
                            torch.full_like(total, float("nan")))
        com_w = (masses.unsqueeze(-1) * robot.data.body_com_pos_w).sum(1, keepdim=True) / masses.sum(1).view(-1, 1, 1)
        com = yaw_frame(com_w)[:, 0]
        feet = yaw_frame(robot.data.body_pos_w[:, feet_r])
        fx = (feet[..., 0] * front).sum(1) / 2.0
        rx = (feet[..., 0] * rear).sum(1) / 2.0
        static_share = ((com[:, 0] - rx) / (fx - rx)).clamp(0.0, 1.0)
        pitch = torch.rad2deg(torch.asin(robot.data.projected_gravity_b[:, 0].clamp(-1.0, 1.0)))
        down = fz > CONTACT_N
        foot_z = robot.data.body_pos_w[:, feet_r, 2]
        return {"share": share, "pitch": pitch, "com_ahead": com[:, 0] - 0.5 * (fx + rx), "margin": fx - com[:, 0],
                "static_share": static_share, "rear_down": (down & rear).any(1), "all_down": down.all(1),
                "height": -feet[..., 2].mean(1), "width": feet[..., 1].abs().mean(1),
                "rear_z": (foot_z * rear).max(1).values,
                "pose_dev": (robot.data.joint_pos[:, legs] - robot.data.default_joint_pos[:, legs]).abs().sum(1)}

    obs, _ = env.reset()
    walk_n = int(args.walk_s / core.step_dt)
    stop_n = int(args.stop_s / core.step_dt)
    window = int(2.0 / core.step_dt)
    total_n = walk_n + stop_n
    alive = torch.ones(n, dtype=torch.bool, device=dev)
    rec = {k: [] for k in ("share", "pitch", "com_ahead", "margin", "static_share", "rear_down", "all_down", "rear_z",
                           "pose_dev", "height", "width")}
    xy = []
    for k in range(total_n):
        cmd = moving if k < walk_n else zero
        term.vel_command_b[:] = cmd
        obs = core.observation_manager.compute_group("policy")
        obs, _, terminated, truncated, _ = env.step(policy(obs))
        alive &= ~(terminated & (k < total_n - 1))
        s = sample()
        for key in rec:
            rec[key].append(s[key])
        xy.append(robot.data.root_pos_w[:, :2].clone())
    r = {key: torch.stack(v, 0).float().cpu().numpy() for key, v in rec.items()}  # (steps, n)
    xy = torch.stack(xy, 0).cpu().numpy()
    alive = alive.cpu().numpy()
    floor_z = float(r["rear_z"][:walk_n].min()) if walk_n else 0.0  # a foot on the ground, sphere centre height

    walk = slice(max(walk_n - window, 0), walk_n)
    stop = slice(walk_n, total_n)
    rest = slice(total_n - window, total_n)

    def longest_run(mask):
        best = cur = 0
        for m in mask:
            cur = cur + 1 if m else 0
            best = max(best, cur)
        return best * core.step_dt

    results = {}
    for i, name in enumerate(names):
        rows = np.arange(i * args.reps, (i + 1) * args.reps)
        ok = rows[alive[rows]] if alive[rows].any() else rows
        rear_air = np.array([longest_run(~r["rear_down"][stop, j].astype(bool)) for j in rows])

        def m(key, sl, fn=np.nanmean):
            return round(float(fn(r[key][sl][:, ok])), 3)

        res = {"command": list(SCENARIOS[name]), "of": args.reps, "fell": int((~alive[rows]).sum())}
        if name != "stand":
            res["walk"] = {"front_share": m("share", walk), "pitch_deg": m("pitch", walk),
                           "com_ahead_m": m("com_ahead", walk), "height_m": m("height", walk)}
        res["stop"] = {
            "pitch_max_deg": round(float(np.nanmean(r["pitch"][stop][:, ok].max(0))), 2),
            "pitch_max_worst_deg": round(float(r["pitch"][stop][:, ok].max()), 2),
            "rear_air_s": round(float(rear_air[alive[rows]].mean() if alive[rows].any() else rear_air.mean()), 3),
            "rear_air_worst_s": round(float(rear_air.max()), 3),
            "rear_lift_m": round(float((r["rear_z"][stop][:, ok] - floor_z).max(0).mean()), 3),
            "margin_min_m": round(float(r["margin"][stop][:, ok].min(0).mean()), 3),
            "margin_min_worst_m": round(float(r["margin"][stop][:, ok].min()), 3),
            "stop_m": round(float(np.linalg.norm(xy[-1, ok] - xy[walk_n - 1, ok], axis=1).mean()), 3),
        }
        res["rear_air_robots"] = int((rear_air > 0.1).sum())
        res["rest"] = {"front_share": m("share", rest), "static_front_share": m("static_share", rest),
                       "pitch_deg": m("pitch", rest), "com_ahead_m": m("com_ahead", rest),
                       "feet_down": m("all_down", rest), "pose_dev_rad": m("pose_dev", rest),
                       "height_m": m("height", rest), "half_width_m": m("width", rest)}
        results[name] = res
        print(f"[stance] {name:16s} {json.dumps(res)}", flush=True)
    out = {"policy": args.policy, "walk_s": args.walk_s, "stop_s": args.stop_s, "total_mass_kg": round(total_mass, 3),
           "results": results}
    print(f"[stance] total mass {total_mass:.2f} kg", flush=True)
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
    sys.stdout.flush()
    os._exit(0)
