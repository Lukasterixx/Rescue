"""Process-local adapter to the rescue sim; never modifies its source files.

The existing simulator owns the robot, D1, policy, cameras, ROS and main loop.
We replace its configuration factory, environment setup, keyboard callback and
reset callback only in the competition launcher process.
"""

from __future__ import annotations

from dataclasses import dataclass

from .build import TERRAIN_PATH


@dataclass
class Selection:
    """A queued selection: input callbacks never write physics tensors."""

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


class GravelReset:
    def __init__(self, lanes):
        self.stones = [stone for lane in lanes for stone in lane.stones]
        self.view = None

    def bind(self):
        from isaacsim.core.prims import RigidPrim

        self.view = RigidPrim(
            prim_paths_expr=f"{TERRAIN_PATH}/Gravel/stone_.*",
            name="competition_gravel",
            reset_xform_properties=False,
            # These bodies already exist in a running physics scene. The
            # default adds PhysxRigidBodyAPI/sleep attributes to every stone,
            # invalidating their GPU body indices while the view is binding.
            prepare_contact_sensors=False,
        )
        self.view.initialize()
        if self.view.count != len(self.stones):
            raise RuntimeError(
                f"Expected {len(self.stones)} gravel bodies; found {self.view.count}"
            )
        # PhysX view order need not match USD traversal or lexicographic order.
        self.ordered_stones = [
            self.stones[int(path.rsplit("_", 1)[1])] for path in self.view.prim_paths
        ]

    def reset(self):
        import numpy as np
        import torch

        positions = np.array(
            [s.position for s in self.ordered_stones], dtype=np.float32
        )
        rotations = np.array(
            [s.rotation for s in self.ordered_stones], dtype=np.float32
        )
        velocities = np.zeros((len(self.stones), 6), dtype=np.float32)
        if self.view._backend == "torch":
            positions, rotations, velocities = [
                torch.as_tensor(v, device=self.view._device)
                for v in (positions, rotations, velocities)
            ]
        self.view.set_world_poses(positions, rotations)
        self.view.set_velocities(velocities)


class CompetitionRuntime:
    def __init__(self, sim, lanes, usd_path, options, start=0, smoke_steps=0):
        self.sim, self.lanes, self.usd_path, self.options = (
            sim,
            lanes,
            str(usd_path),
            options,
        )
        self.selection = Selection(len(lanes), start)
        self.smoke_steps = smoke_steps
        self.steps = 0
        self.needs_observation = False
        self.status_label = None
        self.window = None
        self.gravel = GravelReset(lanes) if options.gravel == "dynamic" else None

    def install(self):
        sim = self.sim
        original_config = sim.UnitreeGo2CustomEnvCfg
        original_keyboard = sim.sub_keyboard_event
        original_reset = sim.reset_robot_and_arm
        original_runner = sim.OnPolicyRunner
        original_wrapper = sim.RslRlVecEnvWrapper
        runtime = self

        def config():
            from isaaclab.terrains import TerrainImporterCfg

            cfg = original_config()
            cfg.scene.num_envs = 1
            cfg.scene.terrain = TerrainImporterCfg(
                prim_path="/World/competition",
                terrain_type="usd",
                usd_path=self.usd_path,
                env_spacing=20.0,
                debug_vis=False,
            )
            lane = self.lanes[self.selection.current]
            cfg.scene.robot.init_state.pos = lane.spawn
            cfg.scene.robot.init_state.rot = lane.spawn_rotation
            cfg.scene.height_scanner.mesh_prim_paths = [f"{TERRAIN_PATH}/WalkableScan"]
            # No corruption during manual evaluation, and no random pushes.
            cfg.observations.policy.enable_corruption = False
            for name in ("push_robot", "base_external_force_torque"):
                if hasattr(cfg.events, name):
                    setattr(cfg.events, name, None)
            return cfg

        def keyboard(event, *args, **kwargs):
            name = event.input.name
            f_keys = tuple(f"F{i + 1}" for i in range(self.selection.count))
            if name in f_keys + ("PAGE_UP", "PAGE_DOWN", "HOME", "R"):
                if event.type == sim.carb.input.KeyboardEventType.KEY_PRESS:
                    if name in f_keys:
                        self.selection.select(int(name[1:]) - 1)
                    elif name in ("PAGE_UP", "PAGE_DOWN"):
                        self.selection.cycle(-1 if name == "PAGE_UP" else 1)
                    else:
                        self.selection.select(self.selection.current)
                    sim.RESET_REQUESTED = True
                return True  # Ignore repeat/release, preserving held-key behaviour.
            return original_keyboard(event, *args, **kwargs)

        def reset(env, d1_controller=None):
            import torch

            lane = self.lanes[self.selection.consume()]
            robot = env.unwrapped.scene["robot"]
            pose = torch.tensor(
                [(*lane.spawn, *lane.spawn_rotation)],
                dtype=robot.data.root_state_w.dtype,
                device=env.unwrapped.device,
            )
            sim.ROBOT_START_POS = list(lane.spawn)
            sim.ROBOT_START_ROT = list(lane.spawn_rotation)
            sim.ROBOT_RESET_ROOT_POSE = pose
            if self.gravel is not None:
                self.gravel.reset()
            # Existing reset restores the joints, D1 targets/power, velocities,
            # command buffer and ROS odometry, including the separate arm mode.
            original_reset(env, d1_controller)
            core = env.unwrapped
            core.scene.reset()
            core.action_manager.reset()
            core.observation_manager.reset()
            core.episode_length_buf.zero_()
            robot.set_joint_position_target(robot.data.joint_pos.clone())
            robot.set_joint_velocity_target(torch.zeros_like(robot.data.joint_vel))
            core.scene.write_data_to_sim()
            core.scene.update(core.physics_dt)
            core.scene["height_scanner"].update(0.0, force_recompute=True)
            self.needs_observation = True
            self.show_status()
            if self.smoke_steps:
                torch.testing.assert_close(robot.data.root_state_w[:, :7], pose)
                assert torch.count_nonzero(robot.data.root_state_w[:, 7:13]) == 0
                assert torch.count_nonzero(core.action_manager.action) == 0
                print(f"[competition][smoke] Teleport and reset verified: {lane.key}")

        class Runner(original_runner):
            def get_inference_policy(self, *args, **kwargs):
                policy = super().get_inference_policy(*args, **kwargs)

                def infer(obs):
                    if runtime.needs_observation:
                        # The host loop retains its previous obs across manual
                        # resets. Recompute BEFORE the first policy inference.
                        obs = runtime.sim._ENV_REF.get_observations()
                        actor = getattr(self.alg, "actor", None)
                        if actor is not None and hasattr(actor, "reset"):
                            import torch

                            actor.reset(
                                torch.ones(
                                    1,
                                    dtype=torch.bool,
                                    device=runtime.sim._ENV_REF.device,
                                )
                            )
                        runtime.needs_observation = False
                    return policy(obs)

                return infer

        class Wrapper(original_wrapper):
            def step(self, actions):
                result = super().step(actions)
                runtime.steps += 1
                if runtime.smoke_steps:
                    # Run both jumps and a reset, without a window or keyboard.
                    if runtime.steps in (10, 20):
                        runtime.selection.cycle(1)
                        sim.RESET_REQUESTED = True
                    elif runtime.steps == 30:
                        sim.RESET_REQUESTED = True
                    import torch

                    if not torch.isfinite(
                        self.unwrapped.scene["robot"].data.root_state_w
                    ).all():
                        raise RuntimeError(
                            "Non-finite robot state during competition smoke test"
                        )
                return result

        sim.UnitreeGo2CustomEnvCfg = config
        sim.sub_keyboard_event = keyboard
        sim.reset_robot_and_arm = reset
        sim.OnPolicyRunner = Runner
        sim.RslRlVecEnvWrapper = Wrapper
        sim.setup_custom_env = self.setup
        # The host's visual warehouse offsets are unrelated to competition USD.
        sim.init_wh_vis_offset_once = lambda: None
        if self.smoke_steps:
            app = sim.simulation_app

            class LimitedApp:
                def is_running(self):
                    return runtime.steps < runtime.smoke_steps and app.is_running()

            sim.simulation_app = LimitedApp()

    def setup(self):
        if self.gravel is not None:
            self.gravel.bind()
        # Used for automatic resets too if a termination is added to the host.
        self.needs_observation = True
        if not self.sim.args_cli.headless:
            import omni.ui as ui

            self.window = ui.Window("Competition", width=560, height=165)
            with self.window.frame:
                with ui.VStack(spacing=6):
                    self.status_label = ui.Label("", word_wrap=True, height=45)
                    ui.Label(
                        "     ".join(
                            f"F{i + 1}  {lane.title}" for i, lane in enumerate(self.lanes)
                        )
                    )
                    ui.Label("Page Down / Page Up  Next / previous lane")
                    ui.Label("Home or R  Reset robot, arm and gravel")
                    ui.Label("Click the viewport to use the keyboard.")
        self.show_status()

    def show_status(self):
        lane = self.lanes[self.selection.current]
        message = f"{self.selection.current+1}/{len(self.lanes)}  {lane.title} | {self.options.difficulty} | gravel: {self.options.gravel}"
        if self.status_label is not None:
            self.status_label.text = message
        print(
            f"[competition] {message}\n[competition] F1-F{len(self.lanes)}: select | PageDown/PageUp: cycle | Home/R: reset"
        )
