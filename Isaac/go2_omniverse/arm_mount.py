"""Where the D1's end-effector prim lives, which depends on how it is mounted.

The two mount modes put Link6 in different places on the stage:

    teleport : /World/envs/env_N/Arm/Link6          (its own articulation)
    weld     : /World/envs/env_N/Robot/D1/Link6     (nested inside the Go2's)

Four things hang off that prim -- the EE flashlight, the front camera, the
camera's render product, and the ROS camera graph -- and every one of them used
to hardcode the teleport path. This module is the single place that knows, so
adding a mount mode does not mean hunting for string literals.

Kept free of heavy imports so `ros2.py` and `omnigraph.py` can import it without
caring about USD or Isaac. `omniverse_sim.py` calls `set_welded()` once, right
after the CLI is parsed and before anything builds a prim path.
"""
from __future__ import annotations

_WELDED = False


def set_welded(welded: bool) -> None:
    """Record the mount mode. Call once, before any prim paths are resolved."""
    global _WELDED
    _WELDED = bool(welded)


def is_welded() -> bool:
    return _WELDED


def ee_prim_path(env_index: int) -> str:
    """Absolute prim path of the D1's Link6 (end-effector) for one environment."""
    subpath = "Robot/D1" if _WELDED else "Arm"
    return f"/World/envs/env_{env_index}/{subpath}/Link6"
