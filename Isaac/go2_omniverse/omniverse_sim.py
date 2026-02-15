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
for _ext in ["omni.isaac.sensor", "omni.isaac.sensors", "omni.isaac.range_sensor"]:
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
    speed = 1.0  # existing teleop speed (kept for your WASD mapping)

    if event.type in (carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT):
        # -------- Arrow keys: visual slide with accelerating step (in meters) ----------
        if event.input.name == 'UP':
            step = _arrow_dir_pressed('UP')
            nudge_warehouse(dx=+step)         # robot appears to move +X
        if event.input.name == 'DOWN':
            step = _arrow_dir_pressed('DOWN')
            nudge_warehouse(dx=-step)         # robot appears to move -X
        if event.input.name == 'RIGHT':
            step = _arrow_dir_pressed('RIGHT')
            nudge_warehouse(dz=-step)         # robot appears to move +Y
        if event.input.name == 'LEFT':
            step = _arrow_dir_pressed('LEFT')
            nudge_warehouse(dz=+step)         # robot appears to move -Y

         # -------- Panel Rotation Hotkeys (M / N) ----------
        if event.input.name == 'M':
            # Rotate Positive X
            rotate_panels_local_x(1)
        if event.input.name == 'N':
            # Rotate Negative X
            rotate_panels_local_x(-1)

        # -------- Checkpoint hotkeys ----------
        if event.input.name == 'C':
            save_vis_checkpoint()
        if event.input.name == 'R':
            load_vis_checkpoint()

        # --- NEW: Return to Base Hotkey ---
        if event.input.name == 'B':
            if _ROS_NODE_REF is not None:
                _ROS_NODE_REF.trigger_return_to_base()

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
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    resume_path = get_checkpoint_path(log_root_path, agent_cfg["load_run"], agent_cfg["load_checkpoint"])
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")

    ppo_runner = OnPolicyRunner(env, agent_cfg, log_dir=None, device=agent_cfg["device"])
    ppo_runner.load(resume_path)
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")

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

    # --- OPTIMIZATION VARIABLES ---
    ros_step_counter = 0
    ros_decimation = 1
    next_deadline = time.monotonic() + 0.1
    
    while simulation_app.is_running():

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)
            
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
