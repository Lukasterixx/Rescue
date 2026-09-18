"""The rescue sim: the Go2 with its D1 and wrist RealSense, the competition's lanes and D1Training's cup demo.

    ./run_sim.sh                              # the Shifty Gravel lane, windowed
    ./run_sim.sh --level cup                  # start in the cup demo
    ./run_sim.sh --headless --smoke-steps 400 # load every level and stand up, lie down; then exit

Levels load at runtime from the "Rescue sim" window or F1..Fn. WASD/QE drive the robot (or robot0/cmd_vel), and
the viewport is Isaac's own: move it yourself, or pick "asset root" in Isaac Lab's viewer settings to follow the
robot. The arm is not driven by anything in here; see `d1_arm.py` for how to drive it and what it does.
"""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

HERE = Path(__file__).resolve().parent            # rescue_sim/
SIM_DIR = HERE.parent                             # Isaac/go2_omniverse: the policy logs, the D1 URDF, d1_sdk
GENERATED = HERE / "generated"
DRIVE_KEYS = ("W", "A", "S", "D", "Q", "E")


def parse_args(argv=None):
    from isaaclab.app import AppLauncher

    import cli_args

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--level", default="gravel",
                        help="level to start in: a competition lane key (gravel, krails, square, ...) or cup")
    parser.add_argument("--smoke-steps", type=int, default=0,
                        help="run this many policy steps, loading every level on the way, then exit (>= 300)")
    parser.add_argument("--smoke-shots", default=None, metavar="DIR",
                        help="with --smoke-steps in a window: save the viewport to DIR after each level loads")
    parser.add_argument("--realtime", action=argparse.BooleanOptionalAction, default=True,
                        help="never run faster than real time (default on): the arm's 10 Hz bridge runs on "
                             "the wall clock")
    parser.add_argument("--seed", type=int, default=42)
    # The competition's geometry.
    from competition.build import add_geometry_args

    add_geometry_args(parser)
    # The arm.
    parser.add_argument("--d1_domain_id", type=int, default=0, help="DDS domain of the simulated D1 (the real one: 0)")
    parser.add_argument("--arm_mass", type=float, default=3.152, metavar="KG",
                        help="D1 mass in kg; Unitree's published D1-550 figure, as D1Training uses")
    # The camera.
    parser.add_argument("--camera", default="calibration",
                        help="'calibration' (the bench D435i, default), a preset (d435, d405, d455), or a "
                             "calibration file")
    parser.add_argument("--mount", default=None, metavar="FILE",
                        help="wrist mount file (d1training.wrist_mount/1); default assets/mounts/wrist_mount.json")
    parser.add_argument("--camera-body", action=argparse.BooleanOptionalAction, default=True,
                        help="draw the D435's case on the wrist (visual only)")
    parser.add_argument("--depth-noise", action=argparse.BooleanOptionalAction, default=True,
                        help="stereo noise on the published depth (range limits apply either way)")
    parser.add_argument("--camera-hz", type=float, default=15.0, help="wrist camera publishing rate on ROS")
    parser.add_argument("--profile", action="store_true", help="print where each step's time goes, every 10 s")
    cli_args.add_rsl_rl_args(parser)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args(argv)
    if args.smoke_steps and args.smoke_steps < 300:
        parser.error("--smoke-steps must be at least 300 (or 0 for interactive use): every level, then standing up")
    # Isaac Lab's Camera needs the RTX sensor pipeline.
    args.enable_cameras = True
    return args


def _quiet_sim_time_warnings():
    """Every RTX sensor frame asks the simulation manager for the sim time at that frame and logs two warnings when it
    cannot interpolate one, which it never can here: Isaac Lab steps physics itself. The fallback it takes, the
    current sim time, is the right stamp; only the log suffers. Errors on the channel still print."""
    import carb

    carb.settings.get_settings().set_string("/log/channels/isaacsim.core.simulation_manager.plugin", "error")


def _enable_extensions():
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    for name in ("isaacsim.core.nodes", "isaacsim.sensors.rtx", "isaacsim.sensors.camera",
                 "isaacsim.sensors.physics", "isaacsim.sensors.physx"):
        try:
            manager.set_extension_enabled_immediate(name, True)
        except Exception:
            pass


def _urdf_arm_limits(urdf_path) -> tuple[list[float], list[float]]:
    import xml.etree.ElementTree as ET

    from rescue_sim import d1_model

    joints = {j.get("name"): j for j in ET.parse(urdf_path).getroot().findall("joint")}
    lower, upper = [], []
    for name in d1_model.ARM_JOINTS:
        limit = joints[name].find("limit")
        lower.append(float(limit.get("lower")))
        upper.append(float(limit.get("upper")))
    return lower, upper


def _load_policy(env, agent_cfg, device):
    """The walking policy, loaded for inference. Any rsl_rl checkpoint of the right shape loads: parameters the model
    does not have (a training-time action std, say) are dropped, and a shape mismatch still fails loudly."""
    import torch
    from isaaclab_tasks.utils import get_checkpoint_path
    from rsl_rl.runners import OnPolicyRunner

    root = os.path.abspath(os.path.join(str(SIM_DIR), "logs", "rsl_rl", agent_cfg["experiment_name"]))
    path = get_checkpoint_path(root, agent_cfg["load_run"], agent_cfg["load_checkpoint"])
    print(f"[policy] walking checkpoint: {path}", flush=True)
    runner = OnPolicyRunner(env, agent_cfg, log_dir=None, device=agent_cfg["device"])
    try:
        runner.load(path, load_optimizer=False)
    except TypeError:
        checkpoint = torch.load(path, map_location=agent_cfg["device"])
        for module, key in ((runner.alg.actor, "actor_state_dict"), (runner.alg.critic, "critic_state_dict")):
            own = module.state_dict()
            usable = {k: v for k, v in checkpoint[key].items() if k in own}
            dropped = sorted(set(checkpoint[key]) - set(usable))
            if dropped:
                print(f"[policy] {key}: ignoring {dropped} (not in this model)", flush=True)
            module.load_state_dict(usable, strict=False)
    return runner, runner.get_inference_policy(device=device)


def run(args, simulation_app):
    import numpy as np
    import torch

    _quiet_sim_time_warnings()
    _enable_extensions()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

    from agent_cfg import unitree_go2_agent_cfg as agent_cfg
    from arm_weld import build_welded_robot_usd
    from competition.build import TERRAIN_PATH, export_scene, options_from_args
    from competition.runtime import GravelReset

    from rescue_sim import d1_model, env_cfg as envs, levels as lv
    from rescue_sim.camera_asset import build_camera_usd
    from rescue_sim.cup_asset import build_cup_usd
    from rescue_sim.d1_drive import SimulatedD1
    from rescue_sim.realsense import (DEFAULT_MOUNT, WristCameraPublisher, load_mount, rendered_intrinsics,
                                      resolve_camera)
    from rescue_sim.ros_io import SimNode, add_lidar
    from rescue_sim.runtime import LevelManager

    GENERATED.mkdir(parents=True, exist_ok=True)
    # --- the world ----------------------------------------------------------------------------------------
    options = options_from_args(args)
    scene_usd = GENERATED / f"competition_{os.getpid()}.usdc"
    lanes = export_scene(scene_usd, options)
    levels = lv.catalogue(lanes)
    keys = [level.key for level in levels]
    if args.level not in keys:
        raise SystemExit(f"--level must be one of {keys}")
    start = keys.index(args.level)
    level = levels[start]

    # --- the robot, the arm, the cup, the camera -----------------------------------------------------------
    urdf = SIM_DIR / "d1_arm" / "d1.urdf"
    weld = build_welded_robot_usd(go2_usd_path=UNITREE_GO2_CFG.spawn.usd_path, d1_urdf_path=str(urdf),
                                  out_usd_path=str(GENERATED / "go2_d1.usd"), mount_pos=(0.0, 0.0, 0.08),
                                  arm_mass_kg=args.arm_mass)
    lower, upper = _urdf_arm_limits(urdf)
    rest_q = d1_model.rest_joint_rad(lower, upper)
    rest_travel = d1_model.units_to_travel(d1_model.REST_GRIPPER_UNITS)
    cup = build_cup_usd(HERE / "assets" / "cup" / "High-Resolution_3D_Cup_Model_FBX.usdz", GENERATED,
                        diameter_m=lv.CUP_DIAMETER_M, height_m=lv.CUP_HEIGHT_M, mass_kg=lv.CUP_MASS_KG)
    camera = resolve_camera(args.camera)
    mount = load_mount(args.mount or DEFAULT_MOUNT)
    body = build_camera_usd(GENERATED)["usd_path"] if args.camera_body else None
    print(f"[camera] {camera.name}: {camera.width}x{camera.height}, fx {camera.fx:.2f} fy {camera.fy:.2f} "
          f"cx {camera.cx:.2f} cy {camera.cy:.2f}; depth {camera.min_depth_m:.3f}-{camera.max_depth_m:.1f} m "
          f"({camera.source})", flush=True)
    print(f"[camera] wrist mount: {mount.source}", flush=True)

    cfg = envs.make_env_cfg(
        terrain_usd=str(scene_usd), scan_mesh=f"{TERRAIN_PATH}/WalkableScan", robot_usd=weld.usd_path,
        cup_usd=cup["usd_path"], cup_pose=lv.cup_pose(lv.CUP_DEMO), camera=camera, mount=mount,
        camera_body_usd=body, spawn=level.spawn, spawn_rotation=level.spawn_rotation, arm_q=rest_q,
        finger_travel=rest_travel, device=args.device, seed=args.seed,
        camera_period_s=max(1, round(1.0 / (args.camera_hz * 0.02))) * 0.02)
    envs.base_command["0"] = [0.0, 0.0, 0.0]
    env = RslRlVecEnvWrapper(ManagerBasedRLEnv(cfg=cfg), clip_actions=agent_cfg.get("clip_actions"))
    core = env.unwrapped
    robot = core.scene["robot"]
    names = list(robot.data.joint_names)
    missing = [n for n in d1_model.ARM_JOINTS + d1_model.FINGER_JOINTS if n not in names]
    if missing:
        raise RuntimeError(f"The weld did not take: {missing} are not in the Go2's articulation")
    print(f"[robot] {len(names)} joints, {float(robot.root_physx_view.get_masses()[0].sum()):.3f} kg", flush=True)
    _, policy = _load_policy(env, agent_cfg, core.device)

    # --- sensors that need the timeline, then a reset for them to take ----------------------------------------
    add_lidar(1, debug=False)
    core.sim.reset()

    # --- the arm's firmware, ROS, the levels ------------------------------------------------------------------
    d1 = SimulatedD1(env, domain_id=args.d1_domain_id, seed=args.seed)
    core.action_manager.get_term("d1_arm").arm = d1
    print(f"[D1] simulated arm on DDS domain {args.d1_domain_id}: commands held {d1.firmware.hold_steps} steps, "
          f"feedback every {d1.firmware.feedback_period_steps} steps, resting at "
          f"{[round(math.degrees(q), 1) for q in rest_q]} deg", flush=True)

    rclpy.init()

    def on_cmd_vel(i, vx, vy, wz):
        if manager is None or manager.posture.walking:
            envs.base_command[str(i)] = [vx, vy, wz]

    manager = None

    def on_level(word):
        if manager is not None:
            manager.request(word)

    ros = SimNode(1, on_cmd_vel, on_level)
    executor = SingleThreadedExecutor()
    executor.add_node(ros.node)
    threading.Thread(target=executor.spin, daemon=True).start()
    intrinsics, note = rendered_intrinsics(camera, core.scene["wrist_cam"].data.intrinsic_matrices[0].cpu().numpy())
    print(f"[camera] {note}", flush=True)
    camera_pub = WristCameraPublisher(ros.node, camera, np.random.default_rng(args.seed) if args.depth_noise else None,
                                      intrinsics=intrinsics)

    gravel = GravelReset(lanes) if options.gravel == "dynamic" else None
    if gravel is not None:
        gravel.bind()
    manager = LevelManager(env, levels, d1, ros, start=start, gravel=gravel, seed=args.seed, headless=args.headless)
    manager.build_window()
    keyboard = None
    if not args.headless:
        keyboard = _subscribe_keys(manager, envs.base_command)

    # --- the loop -----------------------------------------------------------------------------------------
    wrist = core.scene["wrist_cam"]
    cup_object = core.scene["cup"]
    step_dt = core.step_dt
    camera_every = max(1, round(1.0 / (args.camera_hz * step_dt)))
    smoke = _Smoke(args.smoke_steps, len(levels), None if args.headless else args.smoke_shots) if args.smoke_steps else None
    steps = 0
    obs = env.get_observations()
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())    # the website's Dev tab stops the sim with SIGTERM
    rate = _Rate(step_dt, args.profile)
    try:
        while simulation_app.is_running() and not stop.is_set():
            started = time.perf_counter()
            with torch.inference_mode():
                if smoke is not None:
                    smoke.drive(steps, manager)
                manager.update()
                if manager.needs_observation:
                    obs = env.get_observations()
                    manager.needs_observation = False
                legs = manager.leg_actions(step_dt)
                actions = policy(obs) if legs is None else legs
                rate.mark("policy")
                obs, _, _, _ = env.step(actions)
                rate.mark("env.step")
                d1.end_step()
                steps += 1
                stamp = ros.publish(core.common_step_counter * step_dt, robot, cup_object)
                rate.mark("ros")
                if steps % camera_every == 0:
                    rgb = wrist.data.output["rgb"][0, ..., :3].detach().cpu().numpy().astype(np.uint8)
                    depth = wrist.data.output["distance_to_image_plane"][0, ..., 0].detach().cpu().numpy()
                    camera_pub.publish(rgb, depth, stamp)
                    ros.save_images(rgb, 1.0 / (camera_every * step_dt))
                    rate.mark("camera")
                if smoke is not None:
                    smoke.check(steps, core, manager, d1, camera_pub)
                    if smoke.done(steps):
                        break
            if args.realtime:
                time.sleep(max(0.0, step_dt - (time.perf_counter() - started)))
            rate.step()
    finally:
        if keyboard is not None:
            keyboard()
        ros.close()
        executor.shutdown()
        for leftover in (scene_usd, scene_usd.with_suffix(".json")):
            leftover.unlink(missing_ok=True)
        env.close()
    if smoke is not None:
        smoke.report(steps)


class _Rate:
    """How fast the sim runs against the wall clock, printed every 10 s; with `profile`, where the time goes."""

    def __init__(self, step_dt: float, profile: bool, every_s: float = 10.0):
        self.step_dt, self.profile, self.every_s = step_dt, profile, every_s
        self.window_start = self.last = time.perf_counter()
        self.steps = 0
        self.spent: dict[str, float] = {}

    def mark(self, name: str) -> None:
        if self.profile:
            now = time.perf_counter()
            self.spent[name] = self.spent.get(name, 0.0) + now - self.last
            self.last = now

    def step(self) -> None:
        self.steps += 1
        now = time.perf_counter()
        self.last = now
        elapsed = now - self.window_start
        if elapsed < self.every_s:
            return
        line = f"[rate] {self.steps / elapsed:.1f} steps/s, {self.steps * self.step_dt / elapsed:.2f}x real time"
        if self.profile and self.steps:
            line += "; ms/step " + ", ".join(f"{k} {1000 * v / self.steps:.1f}" for k, v in self.spent.items())
        print(line, flush=True)
        self.window_start, self.steps, self.spent = now, 0, {}


def _subscribe_keys(manager, base_command):
    import carb

    try:
        import omni.appwindow
    except ModuleNotFoundError:
        return None
    iface = carb.input.acquire_input_interface()
    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    press = (carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT)

    def on_key(event, *args, **kwargs):
        if event.type in press:
            # Repeats only matter for driving; everything else acts once per press.
            if event.type == carb.input.KeyboardEventType.KEY_REPEAT and event.input.name not in DRIVE_KEYS:
                return True
            manager.on_key(event.input.name, True, carb.input, base_command)
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            manager.on_key(event.input.name, False, carb.input, base_command)
        return True

    subscription = iface.subscribe_to_keyboard_events(keyboard, on_key)
    return lambda: iface.unsubscribe_to_keyboard_events(keyboard, subscription)


class _Smoke:
    """`--smoke-steps`: load every level in turn, stand the lying robot up and lay it back down, and check that
    nothing goes non-finite, that loads land where they should, and that the arm and the camera report."""

    def __init__(self, total: int, count: int, shots=None):
        self.total, self.count = total, count
        # Room after the last load for the stand-up ramp (75 steps) and some walking.
        self.gap = max(10, (total - 100) // (count + 1))
        self.failures = []
        self.checked_loads = 0
        self.shots = Path(shots) if shots else None
        self.shot_at = None

    def drive(self, step: int, manager) -> None:
        if step and step % self.gap == 0:
            index = step // self.gap
            if index < self.count:
                manager.request_level(index)
            elif index == self.count:
                manager.request_posture_toggle()     # the last level is the cup demo: stand it up

    def check(self, step: int, core, manager, d1, camera_pub) -> None:
        import torch

        self.arm_feedback = d1.firmware.step
        self.frames = camera_pub.frames
        self.walking = manager.posture.walking
        robot = core.scene["robot"]
        if not torch.isfinite(robot.data.root_state_w).all():
            self.failures.append(f"non-finite robot state at step {step}")
        if manager.loads > self.checked_loads:
            self.checked_loads = manager.loads
            spawn = manager.level.spawn
            got = robot.data.root_pos_w[0, :2].detach().cpu().tolist()
            if math.dist(got, spawn[:2]) > 0.05:
                self.failures.append(f"load of {manager.level.key} put the robot at {got}, not {spawn[:2]}")
            print(f"[smoke] loaded {manager.level.key}: robot at ({got[0]:.2f}, {got[1]:.2f})", flush=True)
            self.shot_at = (step + 40, manager.level.key)
        if self.shots is not None and self.shot_at is not None and step == self.shot_at[0]:
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

            self.shots.mkdir(parents=True, exist_ok=True)
            path = self.shots / f"{len(list(self.shots.glob('*.png'))):02d}_{self.shot_at[1]}.png"
            capture_viewport_to_file(get_active_viewport(), str(path))
            try:
                # The whole window, the level window included.
                import omni.renderer_capture

                omni.renderer_capture.acquire_renderer_capture_interface().capture_next_frame_swapchain(
                    str(path.with_name(path.stem + "_window.png")))
            except Exception as exc:
                print(f"[smoke] no window capture: {exc}", flush=True)
            print(f"[smoke] viewport -> {path}", flush=True)

    def done(self, step: int) -> bool:
        return step >= self.total

    def report(self, steps: int) -> None:
        if steps < self.total:
            self.failures.append(f"stopped after {steps}/{self.total} steps")
        if self.checked_loads < self.count:
            self.failures.append(f"only {self.checked_loads}/{self.count} levels loaded")
        if not getattr(self, "frames", 0):
            self.failures.append("the wrist camera published nothing")
        if not getattr(self, "walking", False):
            self.failures.append("the robot never stood back up to walk")
        if self.failures:
            print("[smoke] FAIL: " + "; ".join(self.failures), flush=True)
            raise SystemExit(1)
        print(f"[smoke] PASS: {steps} steps, {self.count} levels loaded", flush=True)


def main(argv=None):
    sys.path.insert(0, str(SIM_DIR))
    os.chdir(SIM_DIR)
    args = parse_args(argv)
    from isaaclab.app import AppLauncher

    app = AppLauncher(args).app
    try:
        run(args, app)
    except BaseException:
        # SimulationApp.close() ends the process, and with it any traceback not yet printed.
        import traceback

        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        raise
    finally:
        app.close()
