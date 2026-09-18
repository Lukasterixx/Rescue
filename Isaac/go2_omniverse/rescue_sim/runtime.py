"""Loading levels at runtime, standing the robot down and up, and the window and keys that ask for both.

UI buttons and keys only queue a request; `LevelManager.update()`, called by the sim loop between policy steps, is
the only thing that writes robot, arm, cup or gravel state. Loading a level teleports the robot to its start with
velocities zeroed, puts the legs in the level's posture and the arm back at its folded rest (the firmware parks
there, so it does not lunge back to an old target), resets the gravel, re-zeroes odometry, and clears the policy's
action history so its next input is fresh. A controller outside the sim (nav2, the behaviour tree) is not reset: it
can send new commands straight after.
"""
from __future__ import annotations

import math
import threading

import numpy as np
import torch

from . import levels as lv


class LevelManager:
    def __init__(self, env, levels, d1, ros, start: int = 0, gravel=None, seed: int = 0, headless: bool = False,
                 on_status=None):
        self.core = env.unwrapped
        self.env = env
        self.levels = levels
        self.d1 = d1
        self.ros = ros
        self.gravel = gravel
        self.headless = headless
        self.selection = lv.Selection(len(levels), start)
        self.selection.pending = start          # the first update() loads the start level
        self.posture = lv.PostureRamp()
        self.robot = self.core.scene["robot"]
        self.cup = self.core.scene["cup"]
        self.device = self.core.device
        self._lock = threading.Lock()
        self._new_cup = False
        self._toggle_posture = False
        self._cup_rng = np.random.default_rng(seed + 1)
        self.needs_observation = False
        self.loads = 0
        self.on_status = on_status
        self.window = None
        self._buttons = []
        self._status = None

        term = self.core.action_manager.get_term("joint_pos")
        ids = term._joint_ids
        self.leg_ids = list(range(self.robot.num_joints))[ids] if isinstance(ids, slice) else list(ids)
        self._offset = term._offset if torch.is_tensor(term._offset) else torch.full(
            (1, len(self.leg_ids)), float(term._offset), device=self.device)
        self._scale = term._scale if torch.is_tensor(term._scale) else torch.full(
            (1, len(self.leg_ids)), float(term._scale), device=self.device)
        names = list(self.robot.data.joint_names)
        lying = lv.leg_pose(names, lv.LYING_LEG_POSE)
        self.lying_legs = np.array([lying[i] for i in self.leg_ids])
        self.standing_legs = self.robot.data.default_joint_pos[0, self.leg_ids].detach().cpu().numpy()

    # ------------------------------------------------------------------------------------------ requests
    def request_level(self, index: int) -> None:
        with self._lock:
            self.selection.select(index)

    def request_cycle(self, direction: int) -> None:
        with self._lock:
            self.selection.cycle(direction)

    def request_reset(self) -> None:
        with self._lock:
            self.selection.select(self.selection.current)

    def request_new_cup(self) -> None:
        with self._lock:
            self._new_cup = True

    def request_posture_toggle(self) -> None:
        with self._lock:
            self._toggle_posture = True

    def request(self, word: str) -> None:
        """What `/sim/level` asks for: a level's key, reset, new_cup, lie or stand."""
        keys = [level.key for level in self.levels]
        if word in keys:
            self.request_level(keys.index(word))
        elif word == "reset":
            self.request_reset()
        elif word == "new_cup":
            self.request_new_cup()
        elif word in ("lie", "stand"):
            if (word == "lie") == (self.posture.posture == lv.STANDING):
                self.request_posture_toggle()
        else:
            print(f"[levels] /sim/level: '{word}' is not one of {keys + ['reset', 'new_cup', 'lie', 'stand']}",
                  flush=True)

    @property
    def level(self) -> lv.Level:
        return self.levels[self.selection.current]

    # ------------------------------------------------------------------------------------------ the loop
    def update(self) -> None:
        with self._lock:
            load = self.selection.pending is not None
            index = self.selection.consume()
            new_cup, self._new_cup = self._new_cup, False
            toggle, self._toggle_posture = self._toggle_posture, False
        if load:
            self._load(self.levels[index])
        elif new_cup and self.level.key == lv.CUP_DEMO.key:
            (xy, yaw) = lv.random_cup(self._cup_rng)
            self._place_cup(xy, yaw)
        if toggle:
            self._toggle()

    def leg_actions(self, dt: float):
        """The legs' actions while lying or changing posture; None when the walking policy has them."""
        targets = self.posture.targets(dt)
        if targets is None:
            return None
        targets = torch.as_tensor(targets, dtype=torch.float32, device=self.device).reshape(1, -1)
        return (targets - self._offset) / self._scale

    # ------------------------------------------------------------------------------------------ loading
    def _load(self, level: lv.Level) -> None:
        robot = self.robot
        pose = torch.tensor([[*level.spawn, *level.spawn_rotation]], dtype=torch.float32, device=self.device)
        robot.write_root_pose_to_sim(pose)
        robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=self.device))
        joint_pos = robot.data.default_joint_pos.clone()
        if level.posture == lv.LYING:
            joint_pos[0, self.leg_ids] = torch.as_tensor(self.lying_legs, dtype=joint_pos.dtype, device=self.device)
        joint_pos[0, self.d1.arm_ids] = torch.as_tensor(self.d1.rest_q, dtype=joint_pos.dtype, device=self.device)
        joint_pos[0, self.d1.finger_ids] = torch.tensor([self.d1.rest_travel, -self.d1.rest_travel],
                                                        dtype=joint_pos.dtype, device=self.device)
        robot.write_joint_state_to_sim(joint_pos, torch.zeros_like(joint_pos))
        robot.set_joint_position_target(joint_pos)
        robot.set_joint_velocity_target(torch.zeros_like(joint_pos))
        self.d1.hold_rest()
        if level.key == lv.CUP_DEMO.key:
            self._place_cup(lv.CUP_XY_B, lv.CUP_YAW_DEG)
        if self.gravel is not None:
            self.gravel.reset()
        from . import env_cfg

        for key in env_cfg.base_command:
            env_cfg.base_command[key] = [0.0, 0.0, 0.0]
        if level.posture == lv.LYING:
            self.posture.set(lv.LYING, hold=self.lying_legs)
        else:
            self.posture.set(lv.STANDING)
        core = self.core
        core.scene.reset()
        core.action_manager.reset()
        core.observation_manager.reset()
        core.episode_length_buf.zero_()
        core.scene.write_data_to_sim()
        core.scene.update(core.physics_dt)
        core.scene["height_scanner"].update(0.0, force_recompute=True)
        if self.ros is not None:
            self.ros.rezero_odom()
        self.needs_observation = True
        self.loads += 1
        self._view(level)
        self._refresh_ui()
        self.say(f"{self.selection.current + 1}/{len(self.levels)} {level.title}: "
                 f"{'lying down' if level.posture == lv.LYING else 'standing'}, arm folded at rest")

    def _place_cup(self, xy_b, yaw_b_deg) -> None:
        pose = lv.cup_pose(self.level, xy_b, yaw_b_deg)
        self.cup.write_root_pose_to_sim(torch.tensor([pose], dtype=torch.float32, device=self.device))
        self.cup.write_root_velocity_to_sim(torch.zeros(1, 6, device=self.device))
        self.say(f"cup at ({xy_b[0]:.3f}, {xy_b[1]:.3f}) m from the robot, handle yaw {yaw_b_deg:.0f} deg")

    def _toggle(self) -> None:
        current = self.robot.data.joint_pos[0, self.leg_ids].detach().cpu().numpy()
        if self.posture.posture == lv.LYING:
            self.posture.begin(lv.STANDING, current, self.standing_legs)
            self.say("standing up")
        else:
            from . import env_cfg

            for key in env_cfg.base_command:
                env_cfg.base_command[key] = [0.0, 0.0, 0.0]
            self.posture.begin(lv.LYING, current, self.lying_legs)
            self.say("lying down")
        self._refresh_ui()

    def _view(self, level: lv.Level) -> None:
        """Point the viewport at the new level once. It is yours to move after that."""
        if self.headless:
            return
        try:
            from isaacsim.core.utils.viewports import set_camera_view
        except ImportError:
            return
        yaw = math.radians(level.yaw_deg)
        c, s = math.cos(yaw), math.sin(yaw)
        if level.posture == lv.LYING:
            eye_b, target_b = (0.95, -0.95, 0.70), (0.30, 0.0, 0.10)     # D1Training's pick overview
        else:
            eye_b, target_b = (-2.5, 0.0, 1.8), (1.0, 0.0, 0.0)
        to_world = lambda p: [level.spawn[0] + c * p[0] - s * p[1], level.spawn[1] + s * p[0] + c * p[1],
                              (0.0 if level.posture == lv.LYING else level.spawn[2] - 0.42) + p[2]]
        set_camera_view(eye=to_world(eye_b), target=to_world(target_b))

    # ------------------------------------------------------------------------------------------ the window
    def say(self, line: str) -> None:
        print(f"[levels] {line}", flush=True)
        if self._status is not None:
            self._status.text = line
        if self.on_status is not None:
            self.on_status(line)

    def build_window(self) -> None:
        if self.headless:
            return
        import omni.ui as ui

        # Docked beside the Stage panel, and shown, so it covers none of the viewport.
        self.window = ui.Window("Rescue sim", width=330, height=190 + 33 * len(self.levels),
                                flags=ui.WINDOW_FLAGS_NO_SCROLLBAR)
        self.window.deferred_dock_in("Stage", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        with self.window.frame:
            with ui.VStack(spacing=5, height=0):
                self._status = ui.Label("", word_wrap=True, height=36)
                ui.Label("Levels", height=18)
                self._buttons = []
                for i, level in enumerate(self.levels):
                    self._buttons.append(ui.Button("", height=28, clicked_fn=lambda i=i: self.request_level(i),
                                                   tooltip=level.subtitle))
                with ui.HStack(spacing=5, height=28):
                    ui.Button("Reset level", clicked_fn=self.request_reset, tooltip="Home or R")
                    self._posture_button = ui.Button("", clicked_fn=self.request_posture_toggle, tooltip="L")
                self._cup_button = ui.Button("New cup position", height=28, clicked_fn=self.request_new_cup,
                                             tooltip="Somewhere else in front of the lying robot (cup demo only)")
                ui.Label("Drive: W A S D Q E    Lidar points: T    Click the viewport first", height=18,
                         style={"color": 0xFF909090})
        self._refresh_ui()

    def _refresh_ui(self) -> None:
        if not self._buttons:
            return
        for i, (button, level) in enumerate(zip(self._buttons, self.levels)):
            marker = "> " if i == self.selection.current else "   "
            key = f"F{i + 1}" if i < 12 else ""
            button.text = f"{marker}{key}  {level.title}"
        self._posture_button.text = "Stand up" if self.posture.posture == lv.LYING else "Lie down"
        self._cup_button.enabled = self.level.key == lv.CUP_DEMO.key

    # ------------------------------------------------------------------------------------------ keys
    def on_key(self, name: str, pressed: bool, carb_input, base_command) -> bool:
        """Returns True when the key was ours. Drive keys set the velocity command while held."""
        if not pressed:
            if name in ("W", "S", "A", "D", "Q", "E"):
                base_command["0"] = [0.0, 0.0, 0.0]
            return name in ("W", "S", "A", "D", "Q", "E")
        f_keys = {f"F{i + 1}": i for i in range(min(12, len(self.levels)))}
        if name in f_keys:
            self.request_level(f_keys[name])
        elif name in ("PAGE_UP", "PAGE_DOWN"):
            self.request_cycle(-1 if name == "PAGE_UP" else 1)
        elif name in ("HOME", "R"):
            self.request_reset()
        elif name == "L":
            self.request_posture_toggle()
        elif name == "T":
            from .ros_io import toggle_lidar_debug_draw

            toggle_lidar_debug_draw()
        elif name in ("W", "S", "A", "D", "Q", "E"):
            if not self.posture.walking:
                return True
            speed = 1.0
            base_command["0"] = {"W": [speed, 0, 0], "S": [-speed, 0, 0], "A": [0, speed, 0],
                                 "D": [0, -speed, 0], "Q": [0, 0, speed], "E": [0, 0, -speed]}[name]
        else:
            return False
        return True
