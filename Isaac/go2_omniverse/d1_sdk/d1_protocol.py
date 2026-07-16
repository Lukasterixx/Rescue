"""Wire protocol constants and JSON codec for the Unitree D1 arm.

Every external command is a JSON object inside an ArmString_:

    {"seq": <int>, "address": <int>, "funcode": <int>, "data": {...}}

`address` selects the instruction group and `funcode` the function within it.
This module is pure data + JSON with no transport and no simulator dependency,
so it is shared verbatim between the simulated arm and the real one.
"""
from __future__ import annotations

import json

# ---------------------------------------------------------------- DDS topics
# Note the asymmetry: the command/feedback topics carry the `rt/` prefix but the
# servo angle topic does not. That inconsistency is in the real firmware -- do
# not "fix" it or the arm will not hear us.
TOPIC_ARM_COMMAND = "rt/arm_Command"
TOPIC_ARM_FEEDBACK = "rt/arm_Feedback"
TOPIC_SERVO_ANGLE = "current_servo_angle"

# ------------------------------------------------------------------ addresses
ADDR_COMMAND = 1   # host -> arm
ADDR_STATE = 2     # arm -> host, cyclic state
ADDR_ACK = 3       # arm -> host, per-command acknowledgement

# ------------------------------------------- funcodes under ADDR_COMMAND (1)
FUNC_SET_ANGLE = 1          # {"id", "angle", "delay_ms"}
FUNC_SET_ALL_ANGLES = 2     # {"mode", "angle0".."angle6"}
FUNC_SET_DAMPING = 4        # {"id", "mode"}
FUNC_SET_ALL_DAMPING = 5    # {"mode"}
FUNC_MOTOR_POWER = 6        # {"power"} 0=off 1=on  (e-stop)
FUNC_RETURN_TO_ZERO = 7     # no data

# --------------------------------------------- funcodes under ADDR_STATE (2)
FUNC_STATE_ANGLES = 1        # {"angle0".."angle6"}
FUNC_STATE_STATUS = 3        # {"enable_status","power_status","error_status"}
FUNC_STATE_MOTOR_STATUS = 4  # {"motor0_status".."motor6_status"}

# ----------------------------------------------- funcodes under ADDR_ACK (3)
FUNC_ACK_RECV = 1  # {"recv_status"} 1 = command accepted
FUNC_ACK_EXEC = 2  # {"exec_status"} 1 = command executed

# The arm stamps every cyclic upload with this fixed sequence number.
FEEDBACK_SEQ = 10
FEEDBACK_RATE_HZ = 10.0

# --------------------------------------------------------------- joint layout
# Protocol ids are 0-based; the URDF names joints 1-based. Protocol id 0 is
# URDF `Joint1`. Getting this wrong moves the wrong joint, silently.
NUM_ARM_JOINTS = 6      # ids 0..5
GRIPPER_ID = 6          # id 6, addressed like a joint but in millimetres
NUM_SERVOS = 7          # ids 0..6

JOINT_NAMES = [f"Joint{i + 1}" for i in range(NUM_ARM_JOINTS)]  # Joint1..Joint6
GRIPPER_JOINT_NAMES = ["Joint7_1", "Joint7_2"]

# Joint travel in degrees, matching the URDF's radian limits.
JOINT_LIMITS_DEG = [
    (-135.0, 135.0),  # id 0  Joint1  base yaw
    (-90.0, 90.0),    # id 1  Joint2  shoulder
    (-90.0, 90.0),    # id 2  Joint3  elbow
    (-135.0, 135.0),  # id 3  Joint4  wrist roll
    (-90.0, 90.0),    # id 4  Joint5  wrist pitch
    (-135.0, 135.0),  # id 5  Joint6  wrist yaw
]

# The gripper's `angle` field is millimetres of jaw stroke, NOT degrees. Same
# field name, same funcode, different unit -- the one real trap in this API.
GRIPPER_STROKE_MM = 65.0
# Each URDF finger travels 30 mm, so the modelled jaw spans 60 mm. Commands are
# accepted across the full 65 mm the spec advertises and clamp at the mechanism.
GRIPPER_FINGER_TRAVEL_M = 0.03
# The gripper needs a long dwell to actually execute; the SDK samples use 2 s.
GRIPPER_DEFAULT_DELAY_MS = 2000

# Smoothing selector on FUNC_SET_ALL_ANGLES. Not a control mode.
SMOOTH_STREAMING = 0   # small smoothing, for ~10 Hz streamed setpoints
SMOOTH_TRAJECTORY = 1  # large smoothing, for discrete trajectory waypoints

# Damping is a 0..80000 range, not a boolean: 0 frees the joint for drag
# teaching, 80000 locks it hard.
DAMPING_RELEASED = 0
DAMPING_LOCKED = 80000


def encode(seq: int, address: int, funcode: int, data: dict | None = None) -> str:
    """Build a D1 wire frame. `data` is omitted entirely when empty, as the
    firmware's own samples do for e.g. return-to-zero."""
    frame: dict = {"seq": int(seq), "address": int(address), "funcode": int(funcode)}
    if data:
        frame["data"] = data
    return json.dumps(frame, separators=(",", ":"))


def decode(payload: str) -> dict | None:
    """Parse a D1 wire frame, returning None on anything malformed.

    The arm is a physical device on a shared bus; a decoder that raises would
    take the control loop down on the first stray packet.
    """
    try:
        frame = json.loads(payload)
    except (ValueError, TypeError):
        return None
    if not isinstance(frame, dict):
        return None
    if "address" not in frame or "funcode" not in frame:
        return None
    return frame


def clamp_joint_deg(joint_id: int, angle_deg: float) -> float:
    """Clamp to the joint's travel. The real arm rejects out-of-range angles;
    clamping here keeps sim and hardware behaviour aligned."""
    lo, hi = JOINT_LIMITS_DEG[joint_id]
    return max(lo, min(hi, float(angle_deg)))


def clamp_gripper_mm(stroke_mm: float) -> float:
    return max(0.0, min(GRIPPER_STROKE_MM, float(stroke_mm)))
