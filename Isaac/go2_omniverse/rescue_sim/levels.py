"""The levels the sim can load at runtime: the competition's lanes, and D1Training's cup demo.

Everything is built into one stage at startup and a level is a place in it, so loading one is a teleport: the
robot, the arm and (for the cup demo) the cup are put back at the level's start, and nothing is rebuilt. The
competition lanes come from `competition/` (geometry and fabrication details in its README); adding a lane there
adds a level here.

**The cup demo** is D1Training's pick scene (`demos/cup/pick_demo/scene.py`, commit 5e19028): the Go2 lying down,
a 55 x 100 mm mug 42 cm ahead of it and 3 cm to its left, handle pointing away. Lying is Unitree's lie-down target
from `unitree_ros2`'s `go2_stand_example.cpp`, held by the legs' own PD gains; the walking policy is off until the
robot is stood up. It sits on the hall floor west of the first lane, facing away from the lanes, so nothing but
floor is behind the cup.

No Isaac imports: the runtime that applies these lives in `runtime.py`.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

STANDING, LYING = "standing", "lying"

# D1Training scene.LYING_LEG_POSE. Unitree LowCmd order is FR, FL, RR, RL; by name here.
LYING_LEG_POSE = {
    "FR_hip_joint": 0.0, "FL_hip_joint": 0.0, "RR_hip_joint": -0.2, "RL_hip_joint": 0.2,
    ".*_thigh_joint": 1.36, ".*_calf_joint": -2.65,
}
# Above where the folded legs rest, so the robot settles rather than starts inside the floor.
LYING_SPAWN_HEIGHT_M = 0.18

# D1Training run_pick_demo defaults: the cup in the lying robot's base frame, and the cup itself.
CUP_XY_B = (0.42, 0.03)
CUP_YAW_DEG = 0.0            # handle along the robot's +x: straight away from it
CUP_DIAMETER_M = 0.055
CUP_HEIGHT_M = 0.10
CUP_MASS_KG = 0.12
# Where "new cup position" puts it, relative to the robot: the band the pick's final set chose from, with the handle
# within 45 deg of pointing straight at or away from the robot (across the jaws it blocks the descent, F-049).
RANDOM_CUP_X = (0.36, 0.44)
RANDOM_CUP_Y = (-0.10, 0.10)
RANDOM_HANDLE_BAND_DEG = 45.0

# The cup demo's robot on the hall floor, 1.2 m west of the first lane's west end (x = -2.42) and facing west (-x),
# away from every lane. competition.geometry.ground_mesh runs the floor 5 m past the lanes, so there are over 3 m of
# floor beyond the cup.
CUP_DEMO_BASE_XY = (-3.6, 0.0)
CUP_DEMO_YAW_DEG = 180.0


def yaw_quat_wxyz(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(yaw_deg) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


@dataclass(frozen=True)
class Level:
    key: str
    title: str
    spawn: tuple[float, float, float]
    spawn_rotation: tuple[float, float, float, float]   # wxyz
    posture: str = STANDING
    subtitle: str = ""

    @property
    def yaw_deg(self) -> float:
        w, _, _, z = self.spawn_rotation
        return math.degrees(2.0 * math.atan2(z, w))


def competition_levels(lanes) -> list[Level]:
    """One level per competition lane, starting on the lane's entry pad."""
    return [Level(lane.key, lane.title, tuple(lane.spawn), tuple(lane.spawn_rotation), STANDING,
                  getattr(lane, "subtitle", "")) for lane in lanes]


CUP_DEMO = Level(
    "cup", "Cup demo",
    (CUP_DEMO_BASE_XY[0], CUP_DEMO_BASE_XY[1], LYING_SPAWN_HEIGHT_M),
    yaw_quat_wxyz(CUP_DEMO_YAW_DEG), LYING,
    "D1Training's pick scene: the Go2 lying, a 55 x 100 mm mug 42 cm ahead",
)


def catalogue(lanes) -> list[Level]:
    return competition_levels(lanes) + [CUP_DEMO]


def base_to_world(level: Level, xy_b, yaw_b_deg: float = 0.0) -> tuple[float, float, float]:
    """A point and heading on the floor, given in the level's spawn frame, in world (x, y, yaw_deg)."""
    yaw = math.radians(level.yaw_deg)
    x = level.spawn[0] + math.cos(yaw) * xy_b[0] - math.sin(yaw) * xy_b[1]
    y = level.spawn[1] + math.sin(yaw) * xy_b[0] + math.cos(yaw) * xy_b[1]
    return x, y, level.yaw_deg + yaw_b_deg


def cup_pose(level: Level, xy_b=CUP_XY_B, yaw_b_deg: float = CUP_YAW_DEG, floor_z: float = 0.0):
    """The cup's root pose (x, y, z, qw, qx, qy, qz): its origin is on its axis at its base."""
    x, y, yaw = base_to_world(level, xy_b, yaw_b_deg)
    return (x, y, floor_z + 0.001, *yaw_quat_wxyz(yaw))


def random_cup(rng: np.random.Generator):
    """(x, y) in the robot's frame and a handle yaw (deg) for the next cup, as D1Training's R key places it."""
    x, y = float(rng.uniform(*RANDOM_CUP_X)), float(rng.uniform(*RANDOM_CUP_Y))
    away = math.degrees(math.atan2(y, x))
    offset = float(rng.uniform(-RANDOM_HANDLE_BAND_DEG, RANDOM_HANDLE_BAND_DEG))
    return (x, y), away + offset + (180.0 if rng.random() < 0.5 else 0.0)


def leg_pose(joint_names, pose: dict) -> dict[int, float]:
    """Resolve a {regex: angle} pose against the robot's joint names: {joint index: angle}."""
    import re

    out = {}
    for index, name in enumerate(joint_names):
        for pattern, value in pose.items():
            if re.fullmatch(pattern, name):
                out[index] = float(value)
                break
    return out


@dataclass
class Selection:
    """A queued level change: UI and keyboard callbacks never write physics tensors, the loop does."""

    count: int
    current: int = 0
    pending: int | None = None

    def select(self, index):
        if not 0 <= index < self.count:
            raise IndexError(index)
        self.pending = index

    def cycle(self, direction):
        start = self.current if self.pending is None else self.pending
        self.pending = (start + direction) % self.count

    def consume(self):
        if self.pending is not None:
            self.current = self.pending
            self.pending = None
        return self.current


class PostureRamp:
    """Stand down or up the way a Go2 does it on command: leg targets move linearly from where the legs are to the
    new posture over `duration_s`. While lying or moving, the walking policy is not consulted; once standing, it is
    handed the robot back, and `resumed` says so for one step so the caller can refresh the policy's inputs."""

    def __init__(self, duration_s: float = 1.5):
        self.duration_s = duration_s
        self.posture = STANDING
        self.resumed = False
        self._start = self._goal = None
        self._elapsed = 0.0

    @property
    def walking(self) -> bool:
        """The policy has the legs: standing, and not on the way there."""
        return self.posture == STANDING and self._goal is None

    def set(self, posture: str, hold=None) -> None:
        """Be in `posture` now, with no transition (a level load). Lying needs the leg targets to `hold`."""
        if posture == LYING and hold is None:
            raise ValueError("lying needs the leg targets to hold")
        self.posture, self.resumed = posture, False
        self._start = self._goal = None if hold is None else np.asarray(hold, float)
        self._elapsed = self.duration_s

    def begin(self, posture: str, start, goal) -> None:
        """Move from the leg angles `start` to `goal`, ending in `posture`."""
        self.posture, self.resumed = posture, False
        self._start, self._goal, self._elapsed = np.asarray(start, float), np.asarray(goal, float), 0.0

    def targets(self, dt: float):
        """Leg targets for this step, or None when the policy has the legs."""
        self.resumed = False
        if self._goal is None:
            return None
        self._elapsed = min(self._elapsed + dt, self.duration_s)
        share = self._elapsed / self.duration_s
        out = self._start + share * (self._goal - self._start)
        if share >= 1.0 and self.posture == STANDING:
            self._start = self._goal = None
            self.resumed = True
        return out
