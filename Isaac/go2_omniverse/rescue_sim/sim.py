"""The rescue sim: the Go2 with its D1 and wrist RealSense, the competition's lanes and D1Training's cup demo.

    ./run_sim.sh                              # the Shifty Gravel lane, windowed
    ./run_sim.sh --level cup                  # start in the cup demo
    ./run_sim.sh --headless --smoke-steps 800 # load every level, lie down and stand up; then exit

Levels load at runtime from the "Rescue sim" window's buttons or /sim/level. WASD/QE drive the robot (or
robot0/cmd_vel), and the viewport is Isaac's own: move it yourself, or pick "asset root" in Isaac Lab's viewer settings to follow the
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
# The walking policy the sim loads unless `--policy` names another: the K-Rail Square policy, trained on this sim's
# robot and fine-tuned to stand and stop on four feet (`policies/go2_krail_2026-10-13/README.md`; the 2026-10-12 one
# before it handstood when stopped). `--policy agent_cfg` walks with agent_cfg.py's rsl_rl checkpoint, the default
# until 2026-10-12.
DEFAULT_POLICY = "policies/go2_krail_2026-10-13"
AGENT_CFG_POLICY = "agent_cfg"


def parse_args(argv=None):
    from isaaclab.app import AppLauncher

    import cli_args

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--level", default="gravel",
                        help="level to start in: a competition lane key (gravel, krails, krails_slopes, "
                             "krails_obstacles, ...) or cup; rescue_sim/README.md lists them")
    parser.add_argument("--smoke-steps", type=int, default=0,
                        help="run this many policy steps, loading every level on the way, then exit (>= 300)")
    parser.add_argument("--smoke-shots", default=None, metavar="DIR",
                        help="with --smoke-steps: save the wrist camera's image (and in a window the viewport) to DIR "
                             "after each level loads")
    parser.add_argument("--realtime", action=argparse.BooleanOptionalAction, default=True,
                        help="never run faster than real time (default on): the arm's 10 Hz bridge runs on "
                             "the wall clock")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--policy", default=DEFAULT_POLICY, metavar="DIR|agent_cfg",
                        help="the walking policy: an exported one, a directory holding policy.pt and params/deploy.yaml, "
                             "absolute or relative to the sim directory (default %(default)s, the K-Rail Square "
                             "policy; others: policies/go2_policies_2026-09-18/flat or .../rough), or 'agent_cfg' "
                             "for agent_cfg.py's rsl_rl checkpoint, the default before 2026-10-12")
    # The competition's geometry.
    from competition.build import add_geometry_args

    add_geometry_args(parser)
    # The arm.
    parser.add_argument("--d1_domain_id", type=int, default=0, help="DDS domain of the simulated D1 (the real one: 0)")
    parser.add_argument("--arm-bridge", default="auto", metavar="auto|off|PATH",
                        help="start VIP-Rescue's maps/arm_bridge.py --sim with the sim, so the behaviour tree and the "
                             "website reach the arm on 127.0.0.1:8084 as on the robot (default auto: the one under "
                             "$VIP_RESCUE_ROOT or ~/VIP-Rescue; skipped if something already listens on the port)")
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
    parser.add_argument("--wrist-light", action="store_true",
                        help="start with the lamp beside the wrist camera on (the window toggles it)")
    parser.add_argument("--world-lidar", action=argparse.BooleanOptionalAction, default=True,
                        help="trace the lidar in the world and publish each scan in livox_frame at the TF's pose "
                             "for its stamp (default on); --no-world-lidar is Isaac's own sensor-frame cloud")
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


def _load_exported(bundle, core):
    """An exported walking policy (`--policy`), once the sim is checked to run the legs as it trained: in the Isaac
    Lab joint order, from its default pose, with its gains, at its control rate, into an observation of its width.
    Every one of those is wrong silently, so a mismatch stops the sim instead."""
    from isaaclab.managers import SceneEntityCfg

    from go2_actuators import GO2_LEG_ACTUATOR_GROUP
    from rescue_sim import env_cfg as envs, exported_policy as xp

    contract = bundle.contract
    robot = core.scene["robot"]
    legs = SceneEntityCfg("robot", joint_names=envs.LEG_JOINTS)      # as the leg observation terms resolve them
    legs.resolve(core.scene)
    ids = list(range(robot.num_joints)) if isinstance(legs.joint_ids, slice) else list(legs.joint_ids)
    names = [robot.data.joint_names[i] for i in ids]
    problems = xp.leg_problems(contract, names, robot.data.default_joint_pos[0, ids].tolist())
    acted = list(core.action_manager.get_term("joint_pos")._joint_names)
    if acted != names:
        problems.append(f"the action term drives {acted}, but the observation reads {names}")
    width = core.observation_manager.group_obs_dim["policy"][0]
    if width != contract.obs_dim:
        problems.append(f"the observation is {width} wide, the contract's {contract.obs_dim}")
    if abs(core.step_dt - contract.step_dt) > 1e-9:
        problems.append(f"the sim steps the policy every {core.step_dt} s, the contract every {contract.step_dt} s")
    actuator = robot.actuators[GO2_LEG_ACTUATOR_GROUP]
    try:
        for what, have, want in (("stiffness", actuator.stiffness, contract.kp),
                                 ("damping", actuator.damping, contract.kd)):
            if not _all_near(have, want):
                problems.append(f"leg {what} is {sorted(set(have.flatten().tolist()))}, trained {want}")
    except xp.DeployContractError as exc:
        problems.append(str(exc))
    if problems:
        raise SystemExit(f"[policy] {bundle.directory} cannot drive this robot:\n  " + "\n  ".join(problems))
    policy = xp.ExportedPolicy(bundle, core.device)
    print(f"[policy] walking policy: {bundle.weights}", flush=True)
    print(f"[policy] {bundle.describe()}", flush=True)
    print(f"[policy] legs as trained: Isaac Lab order, default pose, kp {contract.kp:g} kd {contract.kd:g}, "
          f"{contract.control_rate:g} Hz", flush=True)
    if bundle.scanner is not None:
        # The scan as the policy will first see it: a masked scan misplaced by the scanner's offset sees far fewer.
        start = sum(contract.obs_widths[t] for t in contract.obs_terms[:contract.obs_terms.index("height_scan")])
        scan = core.observation_manager.compute_group("policy")[0, start:start + bundle.scanner.num_rays]
        seen = scan != contract.height_scan_unobserved if bundle.mask else scan.isfinite()
        print(f"[policy] height scan at the start: {int(seen.sum())} of {scan.numel()} cells seen"
              + (" (97 on flat ground when standing)" if bundle.mask else "")
              + (f", median {float(scan[seen].median()):+.2f}" if bool(seen.any()) else ""), flush=True)
    return policy


def _all_near(values, target: float, tol: float = 1e-6) -> bool:
    return bool(((values - target).abs() <= tol).all())


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
    from arm_weld import D1_MOUNT_POS, build_welded_robot_usd
    from competition.build import TERRAIN_PATH, export_scene, options_from_args
    from competition.runtime import loose_resets

    from rescue_sim import d1_model, env_cfg as envs, levels as lv
    from rescue_sim.camera_asset import build_camera_usd
    from rescue_sim.bridge import ArmBridge
    from rescue_sim.cup_asset import build_cup_usd
    from rescue_sim.d1_drive import SimulatedD1
    from rescue_sim.realsense import (DEFAULT_MOUNT, WristCameraPublisher, load_mount, rendered_intrinsics,
                                      resolve_camera)
    from rescue_sim.ros_io import SimNode, add_lidar
    from rescue_sim.runtime import LevelManager

    GENERATED.mkdir(parents=True, exist_ok=True)
    exported = None
    if args.policy and args.policy != AGENT_CFG_POLICY:
        from rescue_sim.exported_policy import Bundle, ExportedPolicyError, resolve

        try:
            exported = Bundle.load(resolve(args.policy, SIM_DIR))
        except ExportedPolicyError as exc:
            raise SystemExit(f"[policy] {exc}")
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
                                  out_usd_path=str(GENERATED / "go2_d1.usd"), mount_pos=D1_MOUNT_POS,
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
        camera_period_s=max(1, round(1.0 / (args.camera_hz * 0.02))) * 0.02, policy=exported)
    envs.base_command["0"] = [0.0, 0.0, 0.0]
    envs.command_limits = exported.contract.command_ranges if exported else None
    # agent_cfg's clip is the sim's own policy's training value. An exported policy's clip, if it has one, is on its
    # action term (`apply_exported_policy`), where its training put it.
    env = RslRlVecEnvWrapper(ManagerBasedRLEnv(cfg=cfg), clip_actions=None if exported else agent_cfg.get("clip_actions"))
    core = env.unwrapped
    robot = core.scene["robot"]
    names = list(robot.data.joint_names)
    missing = [n for n in d1_model.ARM_JOINTS + d1_model.FINGER_JOINTS if n not in names]
    if missing:
        raise RuntimeError(f"The weld did not take: {missing} are not in the Go2's articulation")
    print(f"[robot] {len(names)} joints, {float(robot.root_physx_view.get_masses()[0].sum()):.3f} kg", flush=True)
    policy = _load_exported(exported, core) if exported else _load_policy(env, agent_cfg, core.device)[1]

    # --- sensors that need the timeline, then a reset for them to take ----------------------------------------
    scans = add_lidar(1, debug=False, world_frame=args.world_lidar)
    core.sim.reset()

    # --- the arm's firmware, ROS, the levels ------------------------------------------------------------------
    d1 = SimulatedD1(env, domain_id=args.d1_domain_id, seed=args.seed)
    core.action_manager.get_term("d1_arm").arm = d1
    print(f"[D1] simulated arm on DDS domain {args.d1_domain_id}: commands held {d1.firmware.hold_steps} steps, "
          f"feedback every {d1.firmware.feedback_period_steps} steps, resting at "
          f"{[round(math.degrees(q), 1) for q in rest_q]} deg", flush=True)

    bridge, why = ArmBridge.resolve(args.arm_bridge, domain_id=args.d1_domain_id)
    if bridge is None:
        print(f"[arm_bridge] {why}", flush=True)
    else:
        bridge.start()

    rclpy.init()

    def on_cmd_vel(i, vx, vy, wz):
        if manager is None or manager.posture.walking:
            envs.base_command[str(i)] = [vx, vy, wz]

    manager = None

    def on_level(word):
        if manager is not None:
            manager.request(word)

    def on_teleport(frame, position, orientation):
        if manager is not None:
            manager.request_teleport(frame, position, orientation)

    ros = SimNode(1, on_cmd_vel, on_level, on_teleport, scans)
    executor = SingleThreadedExecutor()
    executor.add_node(ros.node)
    threading.Thread(target=executor.spin, daemon=True).start()
    intrinsics, note = rendered_intrinsics(camera, core.scene["wrist_cam"].data.intrinsic_matrices[0].cpu().numpy())
    print(f"[camera] {note}", flush=True)
    camera_pub = WristCameraPublisher(ros.node, camera, np.random.default_rng(args.seed) if args.depth_noise else None,
                                      intrinsics=intrinsics)

    loose = loose_resets(lanes, options)
    for family in loose:
        family.bind()
    import isaaclab.sim as sim_utils

    light = WristLight(sim_utils.find_matching_prim_paths(core.scene["wrist_cam"].cfg.prim_path)[0],
                       on=args.wrist_light)
    manager = LevelManager(env, levels, d1, ros, start=start, loose=loose, light=light, seed=args.seed,
                           headless=args.headless)
    manager.build_window()
    keyboard = None
    if not args.headless:
        keyboard = _subscribe_keys(manager, envs.base_command)

    # --- the loop -----------------------------------------------------------------------------------------
    wrist = core.scene["wrist_cam"]
    cup_object = core.scene["cup"]
    contacts = core.scene["contact_forces"]
    trunk_ids, _ = contacts.find_bodies("base")
    step_dt = core.step_dt
    status_sent = (-1, -math.inf)            # (manager.changes, sim time) of the last /sim/status
    camera_every = max(1, round(1.0 / (args.camera_hz * step_dt)))
    smoke = _Smoke(args.smoke_steps, levels, args.smoke_shots, args.headless) if args.smoke_steps else None
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
                sim_time = core.common_step_counter * step_dt
                manager.watch(float(robot.data.projected_gravity_b[0, 2]),
                              float(contacts.data.net_forces_w[0, trunk_ids].norm(dim=-1).max()))
                stamp = ros.publish(sim_time, robot, cup_object)
                if manager.changes != status_sent[0] or sim_time - status_sent[1] >= 1.0:
                    ros.publish_status(manager.status(sim_time))
                    status_sent = (manager.changes, sim_time)
                rate.mark("ros")
                if steps % camera_every == 0:
                    rgb = wrist.data.output["rgb"][0, ..., :3].detach().cpu().numpy().astype(np.uint8)
                    depth = wrist.data.output["distance_to_image_plane"][0, ..., 0].detach().cpu().numpy()
                    camera_pub.publish(rgb, depth, stamp)
                    ros.save_images(rgb, 1.0 / (camera_every * step_dt))
                    rate.mark("camera")
                if smoke is not None:
                    smoke.check(steps, core, manager, d1, camera_pub, wrist, actions if legs is None else None)
                    if smoke.done(steps):
                        break
            if args.realtime:
                time.sleep(max(0.0, step_dt - (time.perf_counter() - started)))
            rate.step()
    finally:
        if keyboard is not None:
            keyboard()
        if bridge is not None:
            bridge.stop()
        ros.close()
        executor.shutdown()
        for leftover in (scene_usd, scene_usd.with_suffix(".json")):
            leftover.unlink(missing_ok=True)
        env.close()
    if smoke is not None:
        smoke.report(steps)


class WristLight:
    """A lamp beside the wrist camera, off unless asked for.

    D1Training's camera has no lamp, so with it off the images are D1Training's. The maze needs it: its tarp leaves
    the inside dark. It is a small spot on the camera prim, which looks down its -Z with +Y up (USD's camera axes, as
    a light's are). It sits 3 cm above the lens and 2 cm ahead of it, so the case does not shade it and the image
    does not see it.
    """

    INTENSITY = 3.2e7          # the old sim's end-effector flashlight (5e5 at 2 stops over a 4 cm bulb), on 1 cm
    CONE_DEG = 40.0

    def __init__(self, camera_prim_path: str, on: bool = False):
        import omni.usd
        from pxr import Gf, UsdGeom, UsdLux

        stage = omni.usd.get_context().get_stage()
        light = UsdLux.SphereLight.Define(stage, camera_prim_path + "/wrist_light")
        light.CreateRadiusAttr(0.01)
        light.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.85))
        shaping = UsdLux.ShapingAPI.Apply(light.GetPrim())
        shaping.CreateShapingConeAngleAttr(self.CONE_DEG)
        shaping.CreateShapingConeSoftnessAttr(0.3)
        UsdGeom.Xformable(light.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.03, -0.02))
        self._intensity = light.CreateIntensityAttr(0.0)
        self.path = str(light.GetPath())
        self.on = False
        self.set(on)

    def set(self, on: bool) -> None:
        self._intensity.Set(self.INTENSITY if on else 0.0)
        self.on = bool(on)


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
    """`--smoke-steps`: load every level in turn, then go back to the first, lie down, stand up again and walk forward
    for the steps that are left. Checks that nothing goes non-finite, that loads land where they should, that a level
    with a fixed posture keeps it when asked to change it, that the arm and the camera report, and that the walk
    ends on its feet. The wrist light is on for the maze and off again after. Reports how far the walk got and the
    largest action the walking policy gave: normal walking stays around 10 at most (agent_cfg.py), and a wrong
    observation saturates well past it (28 in the 2026-09-18 bundle's README)."""

    SHOT_DELAY = 40            # steps after a load: the robot has settled and the camera has published
    WALK = (0.5, 0.0, 0.0)     # the closing walk's command: vx, vy, wz
    WALK_STEPS = 150           # at least 3 s of it, when --smoke-steps leaves room
    FALLEN_GRAVITY_Z = -0.5    # the trunk tilted past 60 degrees

    def __init__(self, total: int, levels, shots=None, headless: bool = True):
        self.total, self.levels, self.count = total, levels, len(levels)
        # Every level, then back to the first, lie down and stand up: room after that for the stand-up ramp (75 steps)
        # and the closing walk.
        self.gap = max(10, (total - 100 - self.WALK_STEPS) // (self.count + 3))
        self.failures = []
        self.checked_loads = 0
        self.shots = Path(shots) if shots else None
        self.headless = headless
        self.shot_at = None
        self.walk_start = None      # (step, x, y, yaw) once the closing walk begins
        self.walk_end = None
        self.fell_at = None
        self.max_action = 0.0
        if self.shots is not None and self.gap <= self.SHOT_DELAY:
            print(f"[smoke] loads are {self.gap} steps apart, too close for shots {self.SHOT_DELAY} steps after each; "
                  f"--smoke-steps {100 + self.WALK_STEPS + (self.count + 3) * (self.SHOT_DELAY + 1)} leaves room",
                  flush=True)

    def drive(self, step: int, manager) -> None:
        if step and step % self.gap == 0:
            index = step // self.gap
            if index < self.count:
                manager.request_level(index)
                manager.request_light(self.levels[index].key == "maze")
            elif index == self.count:
                manager.request_level(0)
            elif index in (self.count + 1, self.count + 2):
                manager.request_posture_toggle()     # lie down, then stand up again
        elif step % self.gap == self.gap // 2 and manager.level.posture_fixed:
            manager.request_posture_toggle()         # refused: the level keeps its posture
        if self.walk_start is not None:
            from rescue_sim import env_cfg

            env_cfg.base_command["0"] = list(self.WALK)

    def check(self, step: int, core, manager, d1, camera_pub, wrist, actions=None) -> None:
        """`actions`: what the walking policy gave this step, or None when it did not have the legs."""
        import torch

        self.arm_feedback = d1.firmware.step
        self.frames = camera_pub.frames
        self.walking = manager.posture.walking
        robot = core.scene["robot"]
        if not torch.isfinite(robot.data.root_state_w).all():
            self.failures.append(f"non-finite robot state at step {step}")
        if actions is not None:
            self.max_action = max(self.max_action, float(actions.abs().max()))
        if self.walk_start is None and self.walking and step > (self.count + 2) * self.gap:
            x, y = robot.data.root_pos_w[0, :2].tolist()
            w, qx, qy, qz = robot.data.root_quat_w[0].tolist()
            self.walk_start = (step, x, y, math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
            print(f"[smoke] walking forward at {self.WALK[0]} m/s from ({x:.2f}, {y:.2f})", flush=True)
        if self.walk_start is not None:
            self.walk_end = (step, *robot.data.root_pos_w[0, :2].tolist())
            if self.fell_at is None and float(robot.data.projected_gravity_b[0, 2]) > self.FALLEN_GRAVITY_Z:
                self.fell_at = step
                self.failures.append(f"fell over at step {step}, {step - self.walk_start[0]} steps into the walk")
        level = manager.level
        if level.posture_fixed and (manager.posture.posture != level.posture or manager.posture.walking):
            self.failures.append(f"{level.key} left its fixed posture at step {step}")
        if manager.loads > self.checked_loads:
            self.checked_loads = manager.loads
            spawn = manager.level.spawn
            got = robot.data.root_pos_w[0, :2].detach().cpu().tolist()
            if math.dist(got, spawn[:2]) > 0.05:
                self.failures.append(f"load of {manager.level.key} put the robot at {got}, not {spawn[:2]}")
            print(f"[smoke] loaded {manager.level.key}: robot at ({got[0]:.2f}, {got[1]:.2f})", flush=True)
            self.shot_at = (step + self.SHOT_DELAY, manager.level.key)
        if self.shots is not None and self.shot_at is not None and step == self.shot_at[0]:
            self._shoot(self.shot_at[1], manager, wrist)

    def _shoot(self, key: str, manager, wrist) -> None:
        import numpy as np
        from PIL import Image

        self.shots.mkdir(parents=True, exist_ok=True)
        stem = f"{len(list(self.shots.glob('*_wrist.png'))):02d}_{key}"
        rgb = wrist.data.output["rgb"][0, ..., :3].detach().cpu().numpy().astype(np.uint8)
        Image.fromarray(rgb).save(self.shots / f"{stem}_wrist.png")
        light = manager.light is not None and manager.light.on
        print(f"[smoke] wrist camera -> {self.shots / (stem + '_wrist.png')} (light {'on' if light else 'off'}, "
              f"mean {rgb.mean():.1f})", flush=True)
        if self.headless:
            return
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

        path = self.shots / f"{stem}.png"
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
        if self.walk_start is not None and self.walk_end is not None:
            (s0, x0, y0, yaw), (s1, x1, y1) = self.walk_start, self.walk_end
            dx, dy = x1 - x0, y1 - y0
            ahead = dx * math.cos(yaw) + dy * math.sin(yaw)
            aside = -dx * math.sin(yaw) + dy * math.cos(yaw)
            seconds = (s1 - s0) * 0.02
            print(f"[smoke] walked {ahead:.2f} m forward and {aside:+.2f} m sideways in {seconds:.1f} s "
                  f"({ahead / seconds if seconds else 0.0:.2f} m/s for {self.WALK[0]} commanded)", flush=True)
        print(f"[smoke] largest walking action {self.max_action:.2f}", flush=True)
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
        from rescue_sim import bridge

        bridge.stop_all()
        app.close()
