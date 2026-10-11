"""Walk an exported policy through the K-Rail Square, in the rescue sim's own environment, along the course's route.

    ./isaaclab.sh -p ~/Rescue/Isaac/go2_omniverse/training/measure_square.py --headless \\
        --policy policies/go2_krail_2026-10-11 --trials 20 --speed 0.5 --out square.json

THE ENVIRONMENT IS THE SIM'S: `rescue_sim.env_cfg.make_env_cfg` with the competition hall exported by
`competition.build.export_scene`, the robot welded by `arm_weld`, the arm folded at its rest, the scan read off the
hall's walkable-surface mesh, and the policy loaded and checked by the sim's own `_load_exported`. What it leaves out
is everything that needs ROS or DDS: the lidar, the wrist camera's images (and the camera itself), the arm's
simulated firmware (its drives hold the rest, as the firmware does with no command) and the cup.

THE ROUTE is the course drawing's, in the square's own frame (origin at its centre, +y north): from the blue START
pad outside the south-west corner, north along the outside of the west edge, east through the open north half of
the west edge (the 12.2 cm step up), north-east over the NW K-rail into the north triangle, east past the crate,
south-east over the NE K-rail into the east triangle, east out through the open south half of the east edge (the
step down), and north to the green END pad. The waypoints were checked against the lane's meshes: 12.2 cm up, two
22.2 cm rail tops (10 cm above the deck), 12.2 cm down, at least 0.3 m from the crate and the railings.

THE DRIVER is nav2-like: it turns on the spot while the heading error is over 0.5 rad, otherwise drives forward at
`--speed` (scaled by the cosine of the error) with a yaw rate of 1.5 x the heading error, capped at 1.0 rad/s. It
never strafes. Each trial starts at the START pad with +-5 cm and +-5 deg of jitter and a 1.5 s settle.

A TRIAL SUCCEEDS when the base reaches the END pad's waypoint within `--timeout` s without falling (the trunk
touching anything, the base tilted past 60 deg, or under 7 cm above the surface beneath it), and without 20 s
passing between waypoints. Tracking is measured along the way, in the yaw frame: commanded
(vx, vy, wz) against the base's velocity, mean absolute error over every step, and again over "steady" steps (the
command unchanged by more than 0.05 for the last 0.5 s), which is what a policy can be held to.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
os.chdir(SIM_DIR)

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
parser.add_argument("--policy", required=True, help="exported bundle (policy.pt + params/deploy.yaml)")
parser.add_argument("--trials", type=int, default=10)
parser.add_argument("--speed", type=float, default=0.5, help="forward speed the driver asks for, m/s")
parser.add_argument("--max-yaw-rate", type=float, default=1.0)
parser.add_argument("--timeout", type=float, default=60.0, help="seconds per trial")
parser.add_argument("--k-rail-height", type=float, default=0.10, help="the square's rails (competition option)")
parser.add_argument("--seed", type=int, default=1)
parser.add_argument("--out", default=None, help="write the results here as JSON")
parser.add_argument("--dump", action="store_true", help="keep each trial's per-step command, velocity and position")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402

from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG  # noqa: E402

from arm_weld import D1_MOUNT_POS, build_welded_robot_usd  # noqa: E402
from competition.build import TERRAIN_PATH, export_scene  # noqa: E402
from competition.geometry import Options  # noqa: E402
from rescue_sim import d1_model, env_cfg as envs, levels as lv  # noqa: E402
from rescue_sim.exported_policy import Bundle, resolve  # noqa: E402
from rescue_sim.realsense import DEFAULT_MOUNT, load_mount, resolve_camera  # noqa: E402
from rescue_sim.sim import GENERATED, _load_exported, _urdf_arm_limits  # noqa: E402

# The square's frame: its lane origin, +y north. Waypoints in that frame (see the module docstring).
ROUTE = [(-1.85, 0.6), (-0.75, 0.6), (-0.15, 0.85), (0.25, 0.85), (1.0, -0.3), (1.0, -0.65), (1.9, -0.65),
         (1.9, 0.9)]
REACHED_M = 0.2
STAGES = ("stepped_up", "rail_1", "rail_2", "stepped_down", "done")


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@torch.inference_mode()
def main():
    rng = np.random.default_rng(args.seed)
    bundle = Bundle.load(resolve(args.policy, SIM_DIR))
    pid = os.getpid()
    GENERATED.mkdir(parents=True, exist_ok=True)
    scene_usd = GENERATED / f"measure_square_{pid}.usdc"
    lanes = export_scene(scene_usd, Options(gravel="static", k_rail_height=args.k_rail_height))
    origin = np.array(next(lane for lane in lanes if lane.key == "square").origin[:2])
    level = next(level for level in lv.catalogue(lanes) if level.key == "square")
    urdf = SIM_DIR / "d1_arm" / "d1.urdf"
    weld = build_welded_robot_usd(go2_usd_path=UNITREE_GO2_CFG.spawn.usd_path, d1_urdf_path=str(urdf),
                                  out_usd_path=str(GENERATED / f"measure_square_{pid}" / "go2_d1.usd"),
                                  mount_pos=D1_MOUNT_POS, arm_mass_kg=3.152)
    lower, upper = _urdf_arm_limits(urdf)
    cfg = envs.make_env_cfg(
        terrain_usd=str(scene_usd), scan_mesh=f"{TERRAIN_PATH}/WalkableScan", robot_usd=weld.usd_path,
        cup_usd="", cup_pose=(0, 0, 0, 1, 0, 0, 0), camera=resolve_camera("calibration"),
        mount=load_mount(DEFAULT_MOUNT), camera_body_usd=None, spawn=level.spawn,
        spawn_rotation=level.spawn_rotation, arm_q=d1_model.rest_joint_rad(lower, upper),
        finger_travel=d1_model.units_to_travel(d1_model.REST_GRIPPER_UNITS), device=args.device, seed=args.seed,
        policy=bundle)
    cfg.scene.wrist_cam = None
    cfg.scene.cup = None
    cfg.scene.rs_body = None
    envs.base_command["0"] = [0.0, 0.0, 0.0]
    envs.command_limits = bundle.contract.command_ranges
    lows, highs = (np.array(v) for v in bundle.contract.command_ranges)
    env = ManagerBasedRLEnv(cfg=cfg)
    policy = _load_exported(bundle, env)
    robot = env.scene["robot"]
    scanner = env.scene["height_scanner"]
    contacts = env.scene["contact_forces"]
    body_ids, _ = contacts.find_bodies("base")
    dt = env.step_dt

    def floor_below():
        """The scanned surface under the base: the ray landing nearest it (a bundle may centre its grid ahead)."""
        hits = scanner.data.ray_hits_w[0]
        near = torch.linalg.norm(hits[:, :2] - robot.data.root_pos_w[0, :2], dim=1).nan_to_num(1e9).argmin()
        return float(hits[near, 2])

    trials = []
    for trial in range(args.trials):
        wall = time.time()
        obs, _ = env.reset()
        root = robot.data.default_root_state.clone()
        root[:, :2] += torch.as_tensor(rng.uniform(-0.05, 0.05, 2), device=env.device, dtype=root.dtype)
        yaw0 = level.yaw_deg * math.pi / 180 + rng.uniform(-1, 1) * math.radians(5)
        root[:, 3:7] = torch.tensor([math.cos(yaw0 / 2), 0, 0, math.sin(yaw0 / 2)], device=env.device)
        root[:, :3] += env.scene.env_origins
        robot.write_root_pose_to_sim(root[:, :7])
        robot.write_root_velocity_to_sim(torch.zeros_like(root[:, 7:]))
        envs.base_command["0"] = [0.0, 0.0, 0.0]
        with torch.inference_mode():
            for _ in range(int(1.5 / dt)):
                obs, *_ = env.step(policy(obs))
        wp, stage, t, fell, last_progress = 0, set(), 0.0, False, 0.0
        log = {"cmd": [], "vel": [], "pos": [], "steady": []}
        prev_cmds = []
        max_action = 0.0
        while t < args.timeout:
            pos = robot.data.root_pos_w[0].cpu().numpy()
            xy = pos[:2] - origin
            yaw = float(robot.data.heading_w[0])
            d = np.array(ROUTE[wp]) - xy
            if np.linalg.norm(d) < REACHED_M:
                wp += 1
                last_progress = t
                if wp == len(ROUTE):
                    stage.add("done")
                    break
                d = np.array(ROUTE[wp]) - xy
            err = wrap(math.atan2(d[1], d[0]) - yaw)
            wz = float(np.clip(1.5 * err, -args.max_yaw_rate, args.max_yaw_rate))
            vx = 0.0 if abs(err) > 0.5 else args.speed * math.cos(err)
            cmd = np.clip(np.array([vx, 0.0, wz]), lows, highs)
            envs.base_command["0"] = cmd.tolist()
            with torch.inference_mode():
                action = policy(obs)
                max_action = max(max_action, float(action.abs().max()))
                obs, *_ = env.step(action)
            t += dt
            # Stages, in the square's frame.
            pos = robot.data.root_pos_w[0].cpu().numpy()
            x, y = pos[:2] - origin
            if x > -1.0 and abs(y) < 1.2 and floor_below() > 0.1:
                stage.add("stepped_up")
            if x < 0 and y > abs(x) and "stepped_up" in stage:
                stage.add("rail_1")
            if x > 0 and abs(y) < x and "rail_1" in stage:
                stage.add("rail_2")
            if x > 1.35 and "rail_2" in stage and floor_below() < 0.05:
                stage.add("stepped_down")
            # Tracking, yaw frame.
            q = robot.data.root_quat_w[0]
            yaw = float(robot.data.heading_w[0])
            c, s = math.cos(yaw), math.sin(yaw)
            vw = robot.data.root_lin_vel_w[0].cpu().numpy()
            v = [c * vw[0] + s * vw[1], -s * vw[0] + c * vw[1], float(robot.data.root_ang_vel_w[0, 2])]
            prev_cmds.append(cmd)
            window = prev_cmds[-int(0.5 / dt):]
            steady = len(window) == int(0.5 / dt) and float(np.abs(np.array(window) - cmd).max()) <= 0.05
            log["cmd"].append(cmd.tolist())
            log["vel"].append(v)
            log["pos"].append([float(x), float(y), float(pos[2])])
            log["steady"].append(bool(steady))
            # A fall: the trunk touching anything, tipping past 60 deg, or the base within 7 cm of the surface under
            # it (lying on it). Not the head: with self-collisions on, a front leg brushes Head_lower in a turn. Not
            # a ride height: crossing onto the deck the base is briefly ~16 cm above the deck's top.
            gz = float(robot.data.projected_gravity_b[0, 2])
            touch = float(contacts.data.net_forces_w[0, body_ids].norm(dim=-1).max())
            clearance = pos[2] - floor_below()
            if gz > -0.5 or touch > 1.0 or clearance < 0.07:
                fell = f"tilt {math.degrees(math.acos(min(1.0, -gz))):.0f} deg, trunk force {touch:.0f} N, " \
                       f"{clearance:.2f} m over the surface, at ({x:.2f}, {y:.2f}, {pos[2]:.2f})"
                break
            if t - last_progress > 20.0:
                break
            del q
        cmd_a, vel_a = np.array(log["cmd"]), np.array(log["vel"])
        steady_a = np.array(log["steady"], dtype=bool)
        err_a = np.abs(cmd_a - vel_a) if len(cmd_a) else np.zeros((0, 3))
        moving = steady_a & (cmd_a[:, 0] > 0.05) if len(cmd_a) else steady_a
        turning = steady_a & (np.abs(cmd_a[:, 2]) > 0.3) & (cmd_a[:, 0] == 0.0) if len(cmd_a) else steady_a
        result = {
            "trial": trial, "success": "done" in stage and not fell, "fell": bool(fell), "fall": fell or None,
            "stages": [s for s in STAGES if s in stage], "waypoints": wp, "time_s": round(t, 2),
            "max_action": round(max_action, 2),
            "mean_abs_err": err_a.mean(0).round(3).tolist() if len(err_a) else None,
            "steady_mean_abs_err": err_a[steady_a].mean(0).round(3).tolist() if steady_a.any() else None,
            "walking_speed_ratio": round(float(vel_a[moving, 0].mean() / cmd_a[moving, 0].mean()), 3)
            if moving.any() else None,
            "turning_rate_ratio": round(float((vel_a[turning, 2] / cmd_a[turning, 2]).mean()), 3)
            if turning.any() else None,
            "max_height_m": round(float(np.array(log["pos"])[:, 2].max()), 3) if log["pos"] else None,
            "wall_s": round(time.time() - wall, 1),
        }
        if args.dump:
            result["log"] = log
        trials.append(result)
        print(f"[square] trial {trial}: {json.dumps(result)}", flush=True)

    ok = [r for r in trials if r["success"]]
    summary = {
        "policy": str(bundle.directory), "speed": args.speed, "k_rail_height": args.k_rail_height,
        "trials": len(trials), "success": len(ok), "fell": sum(r["fell"] for r in trials),
        "stage_counts": {s: sum(s in r["stages"] for r in trials) for s in STAGES},
        "mean_time_s": round(float(np.mean([r["time_s"] for r in ok])), 2) if ok else None,
        "steady_mean_abs_err": np.mean([r["steady_mean_abs_err"] for r in trials if r["steady_mean_abs_err"]],
                                       axis=0).round(3).tolist(),
        "walking_speed_ratio": round(float(np.mean([r["walking_speed_ratio"] for r in trials
                                                    if r["walking_speed_ratio"]])), 3),
        "turning_rate_ratio": round(float(np.mean([r["turning_rate_ratio"] for r in trials
                                                   if r["turning_rate_ratio"]])), 3)
        if any(r["turning_rate_ratio"] for r in trials) else None,
    }
    print(f"[square] SUMMARY {json.dumps(summary)}", flush=True)
    if args.out:
        Path(args.out).write_text(json.dumps({"summary": summary, "trials": trials}, indent=1) + "\n")
    # This run's own copies of the hall and the robot (~11 MB); the sim's generated/ folder is otherwise its own.
    import shutil

    for path in GENERATED.glob(f"measure_square_{pid}*"):
        shutil.rmtree(path, ignore_errors=True) if path.is_dir() else path.unlink(missing_ok=True)


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
