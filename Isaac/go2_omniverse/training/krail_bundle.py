"""Which training task an exported K-rail bundle belongs to, for the probes that rebuild its robot from the task
(`measure_tracking.py`, `measure_stance.py`). No Isaac imports.

A bundle with a `wheels` section in its deploy.yaml is the Go2-W's (`go2_rescue/krail_go2w_env_cfg.py`); any other is
the Go2's (`go2_rescue/krail_env_cfg.py`). The command envelope is the one the policy trained in, which the sim clamps
every command to (`rescue_sim/env_cfg.py`, `command_limits`), so the probes clamp to it too.
"""
from __future__ import annotations

from pathlib import Path

import yaml

PLAY_TASKS = {"go2": "Isaac-Velocity-Rescue-KRail-Go2-Play-v0", "go2w": "Isaac-Velocity-Rescue-KRail-Go2W-Play-v0"}


def read(policy_dir) -> tuple[str, str, tuple[list[float], list[float]]]:
    """(robot, PLAY task id, (lows, highs) of (vx, vy, wz)) for the bundle in `policy_dir`."""
    contract = yaml.safe_load((Path(policy_dir).expanduser() / "params" / "deploy.yaml").read_text())
    robot = "go2w" if "wheels" in contract else "go2"
    ranges = contract["commands"]["base_velocity"]["ranges"]
    axes = ("lin_vel_x", "lin_vel_y", "ang_vel_z")
    return robot, PLAY_TASKS[robot], ([float(ranges[a][0]) for a in axes], [float(ranges[a][1]) for a in axes])
