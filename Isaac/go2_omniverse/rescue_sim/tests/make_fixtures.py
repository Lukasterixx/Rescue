"""Record what D1Training computes on fixed inputs, so the tests can hold the sim's copies to it.

    python rescue_sim/tests/make_fixtures.py ~/D1Training      # needs torch and numpy; no Isaac

Rerun after a change to D1Training's arm or camera model, then rerun the tests: what fails is what the sim has to
follow. Writes tests/fixtures/d1training.json.
"""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


def main(root: str):
    root = Path(root).expanduser().resolve()
    sys.path.insert(0, str(root))
    import numpy as np
    import torch

    import motor_model
    from position_only.core import TrapezoidTracker
    from demos.cup.pick_demo import camera, realsense
    from demos.cup.pick_demo import camera_body

    commit = subprocess.run(["git", "-C", str(root), "rev-parse", "--short", "HEAD"], capture_output=True,
                            text=True).stdout.strip()
    joints = [f"Joint{i}" for i in range(1, 7)]
    plan = motor_model.arm_trajectory("measured")

    def tracker():
        return TrapezoidTracker(1, 6, plan["accel_rad_s2"], plan["decel_rad_s2"],
                                [motor_model.D1_VELOCITY_LIMIT_RAD_S[j] for j in joints],
                                dead_time=plan["dead_time_s"], retention=plan["replan_velocity_retention"])

    # Three command scripts at the physics rate (5 ms), setpoints sent at the policy steps named (20 ms each).
    scripts = {
        # A 30 deg step on every joint, from rest, sent once.
        "step": {0: [0.5236] * 6},
        # A waypoint every 5 policy steps (10 Hz), 5 deg further each time, on Joint1 and Joint4.
        "stream": {k * 5: [0.0873 * (k + 1), 0.0, 0.0, 0.0873 * (k + 1), 0.0, 0.0] for k in range(8)},
        # Out, then reversed mid-motion.
        "reverse": {0: [1.0, -0.5, 0.8, 0.0, 0.3, -1.0], 15: [-0.2, 0.4, 0.0, 0.5, -0.3, 0.2]},
    }
    trajectories = {}
    for name, script in scripts.items():
        planner = tracker()
        planner.reset(None, torch.zeros(1, 6))
        positions, velocities = [], []
        for policy_step in range(60):
            if policy_step in script:
                planner.command(torch.ones(1, dtype=torch.bool), torch.tensor([script[policy_step]]))
            for _ in range(4):
                planner.step(0.005)
                positions.append(planner.position[0].tolist())
                velocities.append(planner.velocity[0].tolist())
        trajectories[name] = {"script": {str(k): v for k, v in script.items()}, "position": positions,
                              "velocity": velocities}

    calibration = realsense.load_calibration(
        root / "demos/cup/pick_demo/assets/calibration/d435i_238222076237_640x480.json")
    model = realsense.camera_model(calibration)
    mount = camera.load_mount(root / "demos/cup/pick_demo/assets/mounts/wrist_mount.json")
    depth_true = np.linspace(0.05, 3.5, 48, dtype=np.float32).reshape(6, 8)
    depth_true[0, 0] = np.nan
    fixture = {
        "source": f"D1Training {commit}",
        "motor": {
            "effort_limit_nm": [motor_model.D1_EFFORT_LIMIT_NM[j] for j in joints],
            "velocity_limit_rad_s": [motor_model.D1_VELOCITY_LIMIT_RAD_S[j] for j in joints],
            "trajectory": plan,
            "timing_50hz": motor_model.interface_timing("estimated", 50.0, "unitree"),
            "feedback_hz": motor_model.D1_FEEDBACK_HZ, "command_hz": motor_model.D1_COMMAND_HZ,
        },
        "trajectories": trajectories,
        "camera_model": {k: getattr(model, k) for k in ("name", "width", "height", "fx", "fy", "cx", "cy",
                                                        "min_depth_m", "max_depth_m", "baseline_m", "depth_fx",
                                                        "subpixel_rms")},
        "d435_preset": {k: getattr(camera.CAMERAS["d435"], k) for k in ("fx", "fy", "cx", "cy", "min_depth_m",
                                                                         "depth_fx")},
        "mount": {"pose": mount.pose.tolist(), "quat_wxyz": list(mount.quat_wxyz())},
        "depth": {"true": np.where(np.isfinite(depth_true), depth_true, -1.0).tolist(), "nan_at": [0, 0],
                  "noisy_seed_7": camera.realsense_depth(model, depth_true, np.random.default_rng(7)).tolist(),
                  "clean": camera.realsense_depth(model, depth_true, None).tolist()},
        "near_clip_m": camera_body.NEAR_CLIP_PAST_HOUSING_M,
        "mesh_to_optical": camera_body.MESH_TO_OPTICAL.tolist(),
    }
    out = Path(__file__).resolve().parent / "fixtures" / "d1training.json"
    out.write_text(json.dumps(fixture, indent=1) + "\n")
    print(f"wrote {out} from D1Training {commit}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "~/D1Training")
