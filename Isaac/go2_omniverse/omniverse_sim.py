"""Script to play a checkpoint if an RL agent with RSL-RL."""
from __future__ import annotations

# ----------------------- Launch Isaac Sim first -----------------------
import argparse
from omni.isaac.orbit.app import AppLauncher

import cli_args
import time
import os
import threading
from rclpy.executors import MultiThreadedExecutor

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument("--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-Velocity-Rough-Unitree-Go2-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--custom_env", type=str, default="office", help="Setup the environment")
parser.add_argument("--robot", type=str, default="go2", help="Setup the robot")
parser.add_argument("--terrain", type=str, default="rough", help="Setup the robot")
parser.add_argument("--robot_amount", type=int, default=1, help="Setup the robot amount")

# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app  # omni/* available after this

# ----------------------- Now omni / carb / USD are available -----------------------
import omni
import carb
import gymnasium as gym
import torch

from omni.isaac.orbit_tasks.utils import get_checkpoint_path
from omni.isaac.orbit_tasks.utils.wrappers.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlVecEnvWrapper,
)
import omni.isaac.orbit.sim as sim_utils
import omni.appwindow
from rsl_rl.runners import OnPolicyRunner

# Enable required extensions BEFORE importing ros2.py (fixes omni.isaac.sensor import)
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)
for _ext in ["omni.isaac.sensor", "omni.isaac.range_sensor"]:
    try:
        ext_manager.set_extension_enabled_immediate(_ext, True)
    except Exception:
        pass

# Now modules that rely on those extensions
import rclpy
from ros2 import RobotBaseNode, add_camera, add_rtx_lidar, pub_robo_data_ros2
from geometry_msgs.msg import Twist

from agent_cfg import unitree_go2_agent_cfg, unitree_g1_agent_cfg
from custom_rl_env import UnitreeGo2CustomEnvCfg, G1RoughEnvCfg
import custom_rl_env

from omnigraph import create_front_cam_omnigraph

# USD helpers (for moving the warehouse visually)
import omni.usd
from pxr import UsdGeom, Gf, Tf
from omni.isaac.orbit.utils.math import quat_apply


# ===================== Warehouse Visual Offset (purely visual) =====================
WAREHOUSE_PATH = "/World/warehouse"

# Base step size **in meters** (robot/world space) for arrow nudges.
# Change if you want bigger/smaller nudges.
WH_STEP_M = 0.3

# State for visual offset and user-requested features (all in ROBOT meters, Z-up).
WH_OFFSET = Gf.Vec3d(0.0, 0.0, 0.0)     # cumulative visual offset (meters, robot frame: x,y,z with Z-up)
WH_CHECKPOINT = None                    # saved visual offset (meters)
CP_ROBOT_POS = None                     # saved robot world position (meters)
_ARROW_LAST_DIR = None                  # last arrow direction pressed
_ARROW_CUR_STEP = WH_STEP_M             # current step (multiplies while same dir)
_ARROW_MULT = 1.1                       # multiplier applied on consecutive presses

# Reference to env so checkpoint helpers can query robot pose
_ENV_REF = None

# --- NEW: Reference to ROS node for resetting odom ---
_ROS_NODE_REF = None

# ===================== Arm Teleoperation State =====================
ARM_TELEOP_ACTIVE = False
# Starting default position (x, y, z) slightly forward and up relative to the base
ARM_TELEOP_POS = [0.3, 0.0, 0.2] 
# Default rotation (w, x, y, z) - facing forward
ARM_TELEOP_ROT = [1.0, 0.0, 0.0, 0.0]
# Track the current yaw angle independently
ARM_TELEOP_YAW = 0.0

from omni.isaac.core.utils.viewports import set_camera_view

from omni.isaac.core.articulations import ArticulationView
import omni.isaac.core.utils.prims as prim_utils

import numpy as np

import os

from omni.isaac.orbit.controllers import DifferentialIKController, DifferentialIKControllerCfg
from omni.isaac.orbit.utils.math import quat_apply, quat_mul

def generate_arm_usd():
    """Converts the URDF to a USD file and returns the path."""
    from omni.isaac.core.utils.extensions import enable_extension
    enable_extension("omni.importer.urdf")
    import omni.importer.urdf as urdf_importer
    import omni.kit.commands
    
    urdf_interface = urdf_importer._urdf.acquire_urdf_interface()
    import_config = urdf_importer._urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = False 
    import_config.make_default_prim = True 
    
    project_root = os.path.dirname(os.path.abspath(__file__)) 
    urdf_path = os.path.join(project_root, "z1_arm", "z1.urdf")
    dest_usd_path = os.path.join(project_root, "z1_arm", "z1.usd")
    
    if os.path.exists(dest_usd_path):
        os.remove(dest_usd_path)
        
    print(f"[INFO] Converting URDF to USD: {dest_usd_path}")
    
    omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=dest_usd_path
    )
    
    return dest_usd_path

def _stage():
    return omni.usd.get_context().get_stage()

# ---------- Units & axis conversion helpers ----------
def _get_meters_per_unit(stage=None) -> float:
    """Meters per USD stage unit (default 0.01 if unspecified per USD spec)."""
    if stage is None:
        stage = _stage()
    try:
        mpu = UsdGeom.GetStageMetersPerUnit(stage)
        if mpu is None or mpu <= 0.0:
            return 0.01  # fallback: centimeters
        return float(mpu)
    except Exception:
        return 0.01

def _get_up_axis(stage=None) -> str:
    """Return 'Y' or 'Z' (stage up axis)."""
    if stage is None:
        stage = _stage()
    try:
        up_token = UsdGeom.GetStageUpAxis(stage)
        if up_token == UsdGeom.Tokens.y:
            return 'Y'
        return 'Z'
    except Exception:
        return 'Y'

def _robotMeters_to_stageUnits_vec3(robot_m: Gf.Vec3d) -> Gf.Vec3d:
    """
    Convert robot/world meters (x,y,z with Z-up) -> stage translate vector in stage units
    with correct axis mapping. If stage is Y-up, map robot Z -> stage Y.
    """
    stage = _stage()
    mpu = _get_meters_per_unit(stage)  # meters per unit, probably 1 instead of 0.01, so *100
    up = 'Y'

    # Scale meters -> stage units
    x_u = robot_m[0] / mpu
    y_u = robot_m[1] / mpu
    z_u = robot_m[2] / mpu

    if up == 'Y':
        # Stage axes are X,Y,Z with Y-up; robot is Z-up.
        # Map: stage(X,Y,Z) = (robot X, robot Z, robot Y)
        return Gf.Vec3d(x_u, z_u, y_u)
    else:
        # Stage is already Z-up: direct mapping
        return Gf.Vec3d(x_u, y_u, z_u)

# ---------- USD translate op helpers ----------
def _get_or_create_wh_translate_op():
    """Create/find /World/warehouse translate op for visual sliding and order it LAST."""
    stage = _stage()
    prim = stage.GetPrimAtPath(WAREHOUSE_PATH)
    if not prim or not prim.IsValid():
        print(f"[WH][WARN] Missing prim: {WAREHOUSE_PATH}")
        return None

    xform = UsdGeom.Xformable(prim)

    vis_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate and str(op.GetName()).endswith(":translate:visSlide"):
            vis_op = op
            break
    if vis_op is None:
        vis_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble, "visSlide")

    ops = xform.GetOrderedXformOps()
    ops = [op for op in ops if op.GetOpName() != vis_op.GetOpName()]
    ops.append(vis_op)  # make it LAST
    xform.SetXformOpOrder(ops)

    return vis_op


def init_wh_vis_offset_once():
    """Create the visual-slide translate op and zero it once (in stage units)."""
    op = _get_or_create_wh_translate_op()
    if op is not None:
        op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
        mpu = _get_meters_per_unit()
        up = _get_up_axis()
        print(f"[WH] visSlide ready on {WAREHOUSE_PATH} | stage up={up}, metersPerUnit={mpu:g}")

def _apply_wh_offset():
    """Write WH_OFFSET (robot meters) to the dedicated translate op (stage units)."""
    op = _get_or_create_wh_translate_op()
    if op is not None:
        v_units = _robotMeters_to_stageUnits_vec3(WH_OFFSET)
        op.Set(v_units)

def nudge_warehouse(dx=0.0, dy=0.0, dz=0.0):
    """Accumulate and apply a purely visual offset (inputs in robot meters), with Y/Z swapped."""
    global WH_OFFSET
    # Swap Y and Z when adding the offset
    WH_OFFSET = Gf.Vec3d(
        WH_OFFSET[0] + dx,
        WH_OFFSET[1] + dy,  # swap: use dz for Y
        WH_OFFSET[2] + dz   # swap: use dy for Z
    )
    _apply_wh_offset()
    print(f"[WH] Visual offset (meters) = ({WH_OFFSET[0]:.3f}, {WH_OFFSET[1]:.3f}, {WH_OFFSET[2]:.3f})")

    # --- NEW: Update ROS Odom ---
    if _ROS_NODE_REF is not None:
        
        _ROS_NODE_REF.nudge_odom(dx, dz, dy) # Mapping (dx -> x, dz -> y, dy -> z)

_PANEL_TARGETS = {}

def rotate_panels_local_x(sign=1):
    """
    Updates the TARGET rotation for panels 1-7. 
    Does not set USD immediately; the smooth update loop handles that.
    """
    stage = _stage()
    base_path = "/World/warehouse/panels/panel"
    step_deg = 5.0 * sign

    for i in range(1, 8):
        prim_path = f"{base_path}{i}"
        
        # Initialize target if not tracked yet
        if prim_path not in _PANEL_TARGETS:
            # Read current value from USD to start smoothly from where we are
            prim = stage.GetPrimAtPath(prim_path)
            if prim and prim.IsValid():
                xform = UsdGeom.Xformable(prim)
                # Find current angle
                current_angle = 0.0
                for op in xform.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                        val = op.Get()
                        if val: current_angle = val[0]
                        break
                _PANEL_TARGETS[prim_path] = current_angle
            else:
                continue

        # Increment Target
        _PANEL_TARGETS[prim_path] += step_deg
    
    print(f"[PANELS] Targets updated. Smoothing to new positions...")

def update_panels_smoothly(smoothing_factor=0.1):
    """
    Call this in the main loop. Interpolates current panel rotation towards target.
    """
    if not _PANEL_TARGETS:
        return

    stage = _stage()
    
    # Iterate through all tracked panels
    for prim_path, target_x in _PANEL_TARGETS.items():
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            continue
            
        xform = UsdGeom.Xformable(prim)
        
        # Find the RotateXYZ op
        rot_op = None
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                rot_op = op
                break
        
        if rot_op:
            curr_vec = rot_op.Get()
            if curr_vec is None: curr_vec = Gf.Vec3d(0,0,0)
            
            curr_x = curr_vec[0]
            
            # Simple Lerp (Linear Interpolation) for smoothness
            # New = Current + (Target - Current) * factor
            diff = target_x - curr_x
            
            # Optimization: stop updating if very close
            if abs(diff) > 0.01:
                new_x = curr_x + (diff * smoothing_factor)
                rot_op.Set(Gf.Vec3d(new_x, curr_vec[1], curr_vec[2]))

# --------- Robot pose helpers ----------
def _get_robot_world_pos():
    """
    Best-effort getter for the robot's world position (env 0) in METERS (x,y,z; Z-up).
    Returns Gf.Vec3d or None if unavailable.
    """
    env = _ENV_REF
    if env is None:
        return None
    try:
        scene = env.unwrapped.scene
        if hasattr(scene, "envs") and len(scene.envs) > 0 and hasattr(scene.envs[0], "robot"):
            rs = scene.envs[0].robot.data.root_state_w  # (num_envs, 13)
            if rs is not None:
                pos = rs[0, 0:3].detach().cpu().numpy()
                return Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2]))
    except Exception:
        pass
    try:
        robot_iface = scene["robot"]
        rs = robot_iface.data.root_state_w
        pos = rs[0, 0:3].detach().cpu().numpy()
        return Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2]))
    except Exception:
        pass

    print("[WH][POSE] Could not retrieve robot world position.")
    return None

# --------- Visual Checkpoint helpers (C to save, R to restore) ----------
def save_vis_checkpoint():
    """Save the current visual transform (offset in meters) and robot world position (meters)."""
    global WH_CHECKPOINT, CP_ROBOT_POS
    WH_CHECKPOINT = Gf.Vec3d(WH_OFFSET[0], WH_OFFSET[1], WH_OFFSET[2])
    CP_ROBOT_POS = _get_robot_world_pos()
    if CP_ROBOT_POS is None:
        print(f"[WH][CHECKPOINT] Saved offset only (m) = ({WH_CHECKPOINT[0]:.3f}, {WH_CHECKPOINT[1]:.3f}, {WH_CHECKPOINT[2]:.3f})")
    else:
        print(f"[WH][CHECKPOINT] Saved offset (m)=({WH_CHECKPOINT[0]:.3f}, {WH_CHECKPOINT[1]:.3f}, {WH_CHECKPOINT[2]:.3f}); "
              f"robot pos (m)=({CP_ROBOT_POS[0]:.3f}, {CP_ROBOT_POS[1]:.3f}, {CP_ROBOT_POS[2]:.3f})")

def load_vis_checkpoint():
    """
    Restore the saved visual transform. If a saved robot pose exists and we can
    read the current robot pose, additionally compensate for any robot motion:
        WH_OFFSET_new = WH_CHECKPOINT + (current_robot_pos - saved_robot_pos)
    All quantities are in METERS (robot/world Z-up), then converted to stage units/axes on apply.
    """
    global WH_OFFSET, WH_CHECKPOINT, CP_ROBOT_POS
    if WH_CHECKPOINT is None:
        print("[WH][CHECKPOINT] No checkpoint saved yet (press 'C' to save).")
        return

    new_offset = Gf.Vec3d(WH_CHECKPOINT[0], WH_CHECKPOINT[1], WH_CHECKPOINT[2])

    if CP_ROBOT_POS is not None:
        cur_pos = _get_robot_world_pos()
        if cur_pos is not None:
            # Ground-plane only: ignore robot Z (vertical) in compensation
            delta_x = float(cur_pos[0] - CP_ROBOT_POS[0])
            delta_y = float(cur_pos[1] - CP_ROBOT_POS[1])
            # delta_z = float(cur_pos[2] - CP_ROBOT_POS[2])  # IGNORED on purpose
            new_offset = Gf.Vec3d(new_offset[0] + delta_x,
                                  new_offset[1],
                                  new_offset[2] + delta_y)
            print(f"[WH][CHECKPOINT] Compensating ground Δ(m)=({delta_x:.3f}, {delta_y:.3f})")


    WH_OFFSET = new_offset
    _apply_wh_offset()
    print(f"[WH][CHECKPOINT] Restored offset (m) = ({WH_OFFSET[0]:.3f}, {WH_OFFSET[1]:.3f}, {WH_OFFSET[2]:.3f})")

    # --- NEW: Trigger Odom Reset ---
    if _ROS_NODE_REF is not None:
        _ROS_NODE_REF.trigger_odom_reset()
        print("[WH][CHECKPOINT] Triggered ROS Odometry Reset (Zeroing).")

# --------- Arrow step acceleration helpers ----------
def _arrow_dir_pressed(dir_name: str):
    """
    Maintain the 'consecutive press' multiplier with a limiter:
    - If the same direction key is pressed repeatedly, multiply step by 1.2 up to 3x WH_STEP_M.
    - If a different direction is pressed, reset to base step.
    All in METERS.
    """
    global _ARROW_LAST_DIR, _ARROW_CUR_STEP
    if _ARROW_LAST_DIR == dir_name and _ARROW_CUR_STEP < (10.0 * WH_STEP_M):
        _ARROW_CUR_STEP *= _ARROW_MULT
    else:
        _ARROW_LAST_DIR = dir_name
        _ARROW_CUR_STEP = WH_STEP_M
    return _ARROW_CUR_STEP

# ===================== Keyboard Handling =====================
def sub_keyboard_event(event, *args, **kwargs) -> bool:
    # Add ARM_TELEOP_YAW to the globals list here
    global ARM_TELEOP_ACTIVE, ARM_TELEOP_POS, ARM_TELEOP_ROT, ARM_TELEOP_YAW 
    speed = 1.0 

    if event.type in (carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT):
        # -------- Arrow keys & 1/0: Z1 Arm Teleoperation ----------
        is_arm_key = False
        arm_step = 0.02 # Meters to move per keypress
        target_yaw = None
        
        if event.input.name == 'UP':
            ARM_TELEOP_POS[0] += arm_step
            target_yaw = 0.0
            is_arm_key = True
        elif event.input.name == 'DOWN':
            ARM_TELEOP_POS[0] -= arm_step
            target_yaw = np.pi
            is_arm_key = True
        elif event.input.name == 'LEFT':
            ARM_TELEOP_POS[1] += arm_step
            target_yaw = np.pi / 2.0
            is_arm_key = True
        elif event.input.name == 'RIGHT':
            ARM_TELEOP_POS[1] -= arm_step
            target_yaw = -np.pi / 2.0
            is_arm_key = True
        elif event.input.name in ['1', 'NUMPAD_1']:
            ARM_TELEOP_POS[2] += arm_step
            is_arm_key = True
        elif event.input.name in ['0', 'NUMPAD_0']:
            ARM_TELEOP_POS[2] -= arm_step
            is_arm_key = True
            
        if is_arm_key:
            ARM_TELEOP_ACTIVE = True
            
            # --- UPDATE YAW (LINEAR DAMPENING) ---
            if target_yaw is not None:
                # Find the shortest angular distance to the target direction
                diff = (target_yaw - ARM_TELEOP_YAW + np.pi) % (2 * np.pi) - np.pi
                
                # Proportional gain: Determines how aggressively it slows down.
                # A value of 0.08 means the step covers 8% of the remaining distance per frame.
                p_gain = 0.08 
                max_step_rad = np.radians(10.0) # Hard cap at 10 degrees per frame so the base joint can keep up
                
                # Calculate the linear step size and clamp it to our safe maximum
                step_rad = np.clip(diff * p_gain, -max_step_rad, max_step_rad)
                
                # Apply the dampened step
                ARM_TELEOP_YAW += step_rad
                    
                # Convert the incrementally updated yaw back to a quaternion
                ARM_TELEOP_ROT[0] = float(np.cos(ARM_TELEOP_YAW / 2.0)) # w
                ARM_TELEOP_ROT[1] = 0.0                                 # x 
                ARM_TELEOP_ROT[2] = 0.0                                 # y 
                ARM_TELEOP_ROT[3] = float(np.sin(ARM_TELEOP_YAW / 2.0)) # z
                
            # --- SAFEGUARDS ---
            # 1. Floor & Robot Body Z-limit (Don't dig into the dog)
            if ARM_TELEOP_POS[2] < 0.05:
                ARM_TELEOP_POS[2] = 0.05
                
            # 2. Prevent colliding with its own base (Bounding Box around origin)
            if -0.15 < ARM_TELEOP_POS[0] < 0.15 and -0.15 < ARM_TELEOP_POS[1] < 0.15:
                if ARM_TELEOP_POS[2] < 0.2:
                    ARM_TELEOP_POS[2] = 0.2
                    
            # 3. Max reach (Prevent IK singularities / stretching beyond hardware limits)
            dist = np.sqrt(ARM_TELEOP_POS[0]**2 + ARM_TELEOP_POS[1]**2 + ARM_TELEOP_POS[2]**2)
            max_reach = 0.6
            if dist > max_reach:
                ARM_TELEOP_POS[0] *= (max_reach / dist)
                ARM_TELEOP_POS[1] *= (max_reach / dist)
                ARM_TELEOP_POS[2] *= (max_reach / dist)

            # --- DEBUG: Print the bounded target position AND yaw ---
            yaw_deg = np.degrees(ARM_TELEOP_YAW)
            print(f"[TELEOP DEBUG] Arm Target: X={ARM_TELEOP_POS[0]:.3f}, Y={ARM_TELEOP_POS[1]:.3f}, Z={ARM_TELEOP_POS[2]:.3f} | Yaw: {yaw_deg:.1f}°")

        # -------- Keep your WASD/QE teleop ----------
        if len(custom_rl_env.base_command) > 0:
            if event.input.name == 'W':
                custom_rl_env.base_command["0"] = [speed, 0, 0]
            if event.input.name == 'S':
                custom_rl_env.base_command["0"] = [-speed, 0, 0]
            if event.input.name == 'A':
                custom_rl_env.base_command["0"] = [0, speed, 0]
            if event.input.name == 'D':
                custom_rl_env.base_command["0"] = [0, -speed, 0]
            if event.input.name == 'Q':
                custom_rl_env.base_command["0"] = [0, 0, speed]
            if event.input.name == 'E':
                custom_rl_env.base_command["0"] = [0, 0, -speed]

        if len(custom_rl_env.base_command) > 1:
            if event.input.name == 'I':
                custom_rl_env.base_command["1"] = [speed, 0, 0]
            if event.input.name == 'K':
                custom_rl_env.base_command["1"] = [-speed, 0, 0]
            if event.input.name == 'J':
                custom_rl_env.base_command["1"] = [0, speed, 0]
            if event.input.name == 'L':
                custom_rl_env.base_command["1"] = [0, -speed, 0]
            if event.input.name == 'U':
                custom_rl_env.base_command["1"] = [0, 0, speed]
            if event.input.name == 'O':
                custom_rl_env.base_command["1"] = [0, 0, -speed]

    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        # Reset teleop commands on key release
        for i in range(len(custom_rl_env.base_command)):
            custom_rl_env.base_command[str(i)] = [0, 0, 0]
        # (Do NOT reset arrow acceleration here; requirement is to reset only when another direction is pressed.)

    return True

# ===================== Your existing helpers =====================
def setup_custom_env():

    # --- NEW: Skip default USDs if we are using the procedural maze ---
    if args_cli.custom_env == "maze":
        print("[Environment] Skipping static USD load for procedural maze.")
        return
    # ------------------------------------------------------------------
    print(f"[DEBUG] >>> Entering setup_custom_env()")
    print(f"[DEBUG]     args_cli.custom_env = '{args_cli.custom_env}'")
    print(f"[DEBUG]     args_cli.terrain    = '{args_cli.terrain}'")

    world_type = "rotate1"
    try:
        if world_type == "simple":
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/flatter.usd")
            cfg_scene.func("/World/warehouse", cfg_scene, translation=(-160.0, 12.0, 0.0))
        if world_type == "rotate":
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/rotate.usd")
            cfg_scene.func("/World/warehouse", cfg_scene, translation=(30.0, -30.0, 0.0))
        if world_type == "rotate1":
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/rotate1.usd")
            cfg_scene.func("/World/warehouse", cfg_scene, translation=(30.0, -30.0, 0.0))
        if world_type == "double":
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/double.usd")
            cfg_scene.func("/World/warehouse", cfg_scene, translation=(0.0, 0.0, 0.0))
    except Exception:
        print("Error loading custom environment")

def cmd_vel_cb(msg, num_robot):
    x = msg.linear.x
    y = msg.linear.y
    z = msg.angular.z
    custom_rl_env.base_command[str(num_robot)] = [x, y, z]

def add_cmd_sub(num_envs):
    node_test = rclpy.create_node('position_velocity_publisher')
    for i in range(num_envs):
        node_test.create_subscription(Twist, f'robot{i}/cmd_vel', lambda msg, i=i: cmd_vel_cb(msg, str(i)), 10)
    thread = threading.Thread(target=rclpy.spin, args=(node_test,), daemon=True)
    thread.start()

def specify_cmd_for_robots(numv_envs):
    for i in range(numv_envs):
        custom_rl_env.base_command[str(i)] = [0, 0, 0]

# ===================== Main run =====================
def run_sim():
    global _ENV_REF
    global _ROS_NODE_REF
    # subscribe to keyboard
    _input = carb.input.acquire_input_interface()
    _appwindow = omni.appwindow.get_default_app_window()
    _keyboard = _appwindow.get_keyboard()
    _sub_keyboard = _input.subscribe_to_keyboard_events(_keyboard, sub_keyboard_event)

    # configure env
    env_cfg = UnitreeGo2CustomEnvCfg()
    if args_cli.robot == "g1":
        env_cfg = G1RoughEnvCfg()
    env_cfg.scene.num_envs = args_cli.robot_amount

    # --- NEW: NATIVE ORBIT ARM INTEGRATION ---
    dest_usd_path = generate_arm_usd()
    
    from omni.isaac.orbit.assets import ArticulationCfg
    from omni.isaac.orbit.actuators import ImplicitActuatorCfg
    
    # We natively inject the arm into the Orbit configuration
    env_cfg.scene.arm = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Arm",
        spawn=sim_utils.UsdFileCfg(
            usd_path=dest_usd_path,
            # Disable gravity so the arm floats perfectly before we teleport it
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.4),
        ),
        actuators={
            "arm_joints": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                stiffness=400.0,
                damping=40.0,
            )
        }
    )
    # -----------------------------------------

    # ROS camera graph(s)
    for i in range(env_cfg.scene.num_envs):
        create_front_cam_omnigraph(i)

    specify_cmd_for_robots(env_cfg.scene.num_envs)

    agent_cfg: RslRlOnPolicyRunnerCfg = unitree_go2_agent_cfg
    if args_cli.robot == "g1":
        agent_cfg = unitree_g1_agent_cfg

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)
    _ENV_REF = env  # expose to checkpoint helpers

    # load policy
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg["experiment_name"])
    log_root_path = os.path.abspath(log_root_path)
    resume_path = get_checkpoint_path(log_root_path, agent_cfg["load_run"], agent_cfg["load_checkpoint"])
    
    ppo_runner = OnPolicyRunner(env, agent_cfg, log_dir=None, device=agent_cfg["device"])
    ppo_runner.load(resume_path)
    policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)

    # first obs
    obs, _ = env.get_observations()

    # sensors + env USD
    annotator_lst = add_rtx_lidar(env_cfg.scene.num_envs, args_cli.robot, False)    
    camera_lst = add_camera(env_cfg.scene.num_envs, args_cli.robot)
    setup_custom_env()

    stage = omni.usd.get_context().get_stage()
    for path in ("/World/warehouse/Landscape", "/World/groundPlane", "/World/defaultGroundPlane"):
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            prim.SetActive(False)
            print(f"[GROUND] Hid {path}")

    init_wh_vis_offset_once()

    # ROS2
    rclpy.init()
    base_node = RobotBaseNode(env, env_cfg.scene.num_envs)
    _ROS_NODE_REF = base_node

    add_cmd_sub(env_cfg.scene.num_envs)

    executor = MultiThreadedExecutor()
    executor.add_node(base_node)
    threading.Thread(target=executor.spin, daemon=True).start()

    save_vis_checkpoint()

    # Setup the IK Controller
    diff_ik_cfg = DifferentialIKControllerCfg(
        command_type="pose",
        ik_method="dls", # Damped Least Squares is highly stable
    )
    diff_ik_controller = DifferentialIKController(diff_ik_cfg, num_envs=env_cfg.scene.num_envs, device=env.unwrapped.device)

    # Find the body index for the end-effector ("link06") 
    arm = env.unwrapped.scene["arm"]
    ee_body_idx = arm.find_bodies("link06")[0][0] # Get the index of link06

    # Add these trackers just before the while loop
    current_rel_pos = torch.zeros((env_cfg.scene.num_envs, 3), device=env.unwrapped.device, dtype=torch.float32)
    current_rel_rot = torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=env.unwrapped.device, dtype=torch.float32).repeat(env_cfg.scene.num_envs, 1)

    # --- OPTIMIZATION VARIABLES ---
    ros_step_counter = 0
    ros_decimation = 1
    next_deadline = time.monotonic() + 0.1

    # Initialize the camera's smoothed yaw to the robot's starting orientation
    init_quat = env.unwrapped.scene["robot"].data.root_state_w[0, 3:7]
    camera_smooth_yaw = torch.atan2(
        2.0 * (init_quat[0] * init_quat[3] + init_quat[1] * init_quat[2]), 
        1.0 - 2.0 * (init_quat[2] * init_quat[2] + init_quat[3] * init_quat[3])
    ).clone()
    
    while simulation_app.is_running():

        with torch.inference_mode():
            
            # --- 1. Pin the Arm using Native Orbit API ---
            robot = env.unwrapped.scene["robot"]
            arm = env.unwrapped.scene["arm"]
            
            # Grab the current base position and orientation
            root_pose = robot.data.root_state_w[:, :7].clone()
            
            # A. Local offset: 30cm forward (+0.3 X), flush height (0.0 Z)
            local_offset = torch.tensor([[0.0, 0.0, 0.0]], device=env.unwrapped.device, dtype=torch.float32)
            local_offset = local_offset.repeat(env_cfg.scene.num_envs, 1)
            
            # B. Rotate this local offset into the world frame using the dog's orientation
            world_offset = quat_apply(root_pose[:, 3:7], local_offset)
            
            # C. Add the rotated offset to the robot's world position
            root_pose[:, :3] += world_offset
            
            # D. Write the teleport command directly to PhysX
            arm.write_root_pose_to_sim(root_pose)
            
            # E. KILL MOMENTUM: Force the arm's velocity to zero so physics doesn't drag it
            zero_vel = torch.zeros_like(robot.data.root_state_w[:, 7:13])
            arm.write_root_velocity_to_sim(zero_vel)
            # ---------------------------------------------

            # --- 2. Teleop OR ROS 2 controls the Arm joints via Inverse Kinematics ---
            target_pos_np = None
            target_rot_np = None

            # Prioritize keyboard teleoperation if it has been activated
            if ARM_TELEOP_ACTIVE:
                target_pos_np = np.array(ARM_TELEOP_POS, dtype=np.float32)
                target_rot_np = np.array(ARM_TELEOP_ROT, dtype=np.float32)
            # Fall back to ROS 2 topic commands
            elif _ROS_NODE_REF is not None and getattr(_ROS_NODE_REF, 'arm_pose_target_rel', None) is not None:
                target_pos_np, target_rot_np = _ROS_NODE_REF.arm_pose_target_rel

            # Only run the IK solver if we have a valid target
            if target_pos_np is not None and target_rot_np is not None:
                
                # A. Prepare the relative target pos/rot 
                target_rel_pos = torch.tensor(target_pos_np, device=env.unwrapped.device, dtype=torch.float32).repeat(env_cfg.scene.num_envs, 1)
                target_rel_rot = torch.tensor(target_rot_np, device=env.unwrapped.device, dtype=torch.float32).repeat(env_cfg.scene.num_envs, 1)
                
                # B. SMOOTHING (Lerp the target so the arm moves fluidly)
                alpha = 0.05  # Adjust this: Lower = slower/smoother, Higher = faster/snappier
                current_rel_pos = current_rel_pos + alpha * (target_rel_pos - current_rel_pos)
                
                # Lerp quaternions and re-normalize
                current_rel_rot = current_rel_rot + alpha * (target_rel_rot - current_rel_rot)
                current_rel_rot = current_rel_rot / torch.norm(current_rel_rot, dim=-1, keepdim=True)
                
                # C. Convert Smoothed Relative Target -> World Target
                robot_root_pos = robot.data.root_state_w[:, :3]
                robot_root_quat = robot.data.root_state_w[:, 3:7]
                
                target_world_pos = robot_root_pos + quat_apply(robot_root_quat, current_rel_pos)
                target_world_rot = quat_mul(robot_root_quat, current_rel_rot)

                # Fetch current EE state before diff_ik_controller.compute
                ee_pos = arm.data.body_pos_w[:, ee_body_idx, :]
                ee_quat = arm.data.body_quat_w[:, ee_body_idx, :]
                
                # D. Set the Target Command for the IK Solver
                target_pose_tensor = torch.cat([target_world_pos, target_world_rot], dim=-1)
                diff_ik_controller.set_command(target_pose_tensor)
                
                # 1. Fetch Jacobians directly from the PhysX view
                physx_jacobians = arm.root_physx_view.get_jacobians()
                
                # 2. Adjust Jacobian index based on base fix status
                ee_jacobi_idx = ee_body_idx - 1 if arm.is_fixed_base else ee_body_idx
                jacobian = physx_jacobians[:, ee_jacobi_idx, :, :]
                
                # 3. Slice out root body columns if floating base
                if not arm.is_fixed_base:
                    jacobian = jacobian[:, :, 6:]
                
                current_joint_pos = arm.data.joint_pos
                
                # E. Compute required joint targets using IK
                ik_joint_targets = diff_ik_controller.compute(
                    ee_pos=ee_pos,
                    ee_quat=ee_quat,
                    jacobian=jacobian,
                    joint_pos=current_joint_pos
                )
                
                # F. Apply the calculated joint targets to the physics engine
                arm.set_joint_position_target(ik_joint_targets)


            # 1. RL Policy controls the Dog
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)

            # --- 3. Video Game Camera Follow (Velocity Yaw Tracking) ---
            # Get the first robot's position and orientation
            base_pos = robot.data.root_state_w[0, :3]
            base_quat = robot.data.root_state_w[0, 3:7] # [w, x, y, z]

            # Grab the world linear velocity of the robot
            lin_vel_w = robot.data.root_lin_vel_w[0, :3]
            vx, vy = lin_vel_w[0], lin_vel_w[1]
            
            # Calculate the current ground speed
            speed = torch.sqrt(vx*vx + vy*vy)
            
            # If the robot is moving fast enough, target the direction of travel.
            # If it stops, smoothly drift the camera back to the robot's physical forward facing direction.
            if speed > 0.2:
                target_cam_yaw = torch.atan2(vy, vx)
            else:
                w, x, y, z = base_quat[0], base_quat[1], base_quat[2], base_quat[3]
                target_cam_yaw = torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            
            # Calculate shortest angular distance between current camera yaw and target yaw
            pi_tensor = torch.tensor(np.pi, device=env.unwrapped.device)
            diff = (target_cam_yaw - camera_smooth_yaw + pi_tensor) % (2 * pi_tensor) - pi_tensor
            
            # Apply EMA smoothing. 
            # A value of 0.02 to 0.05 gives a great "heavy/smooth" ~1 second tracking delay.
            alpha_cam = 0.03 
            camera_smooth_yaw += diff * alpha_cam
            
            # Create a yaw-only quaternion from our smoothed angle
            yaw_quat = torch.tensor([
                [torch.cos(camera_smooth_yaw / 2.0), 0.0, 0.0, torch.sin(camera_smooth_yaw / 2.0)]
            ], device=env.unwrapped.device)
            
            # Define camera offsets (Upper Behind)
            local_eye = torch.tensor([[-2.0, 0.0, 1.5]], device=env.unwrapped.device)
            local_lookat = torch.tensor([[1.0, 0.0, 0.0]], device=env.unwrapped.device) 
            
            # Rotate offsets by the smoothed yaw and add to dog's global position
            world_eye = base_pos + quat_apply(yaw_quat, local_eye)[0]
            world_lookat = base_pos + quat_apply(yaw_quat, local_lookat)[0]

            # Update the Omniverse viewport camera
            set_camera_view(
                eye=world_eye.cpu().numpy(),
                target=world_lookat.cpu().numpy()
            )
            
            # --- OPTIMIZED ROS PUBLISHING ---
            ros_step_counter += 1
            if ros_step_counter >= ros_decimation:
                next_deadline = pub_robo_data_ros2(
                    args_cli.robot, 
                    env_cfg.scene.num_envs, 
                    base_node, 
                    env, 
                    annotator_lst, 
                    next_deadline,
                    camera_lst=camera_lst
                )
                ros_step_counter = 0
            # --------------------------------
            
    env.close()
