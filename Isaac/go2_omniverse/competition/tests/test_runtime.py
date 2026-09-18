"""Exercise adapter control flow with real CPU tensors and a small host double.

This checks queued input, state reset and fresh policy observations. Actual
PhysX/ROS integration is covered by --headless --smoke-steps 60 on an Isaac GPU.
"""

from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

try:
    import torch
except ImportError:
    torch = None

from competition.geometry import Options, build_lanes
from competition.runtime import CompetitionRuntime


@unittest.skipIf(torch is None, "PyTorch unavailable in this interpreter")
class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.lanes = build_lanes(Options(gravel="static"))
        robot = NS(
            data=NS(
                root_state_w=torch.ones((1, 13)),
                joint_pos=torch.ones((1, 20)),
                joint_vel=torch.ones((1, 20)),
            ),
            set_joint_position_target=Mock(),
            set_joint_velocity_target=Mock(),
        )

        class Scene(dict):
            reset = Mock()
            update = Mock()
            write_data_to_sim = Mock()

        action = torch.ones((1, 12))
        scene = Scene(robot=robot, height_scanner=NS(update=Mock()))
        core = NS(
            scene=scene,
            device="cpu",
            physics_dt=0.005,
            episode_length_buf=torch.ones(1),
            action_manager=NS(action=action, reset=lambda: action.zero_()),
            observation_manager=NS(reset=Mock()),
        )
        self.env = NS(
            unwrapped=core,
            device="cpu",
            get_observations=Mock(return_value={"policy": torch.zeros((1, 235))}),
        )

        class Runner:
            def __init__(self):
                self.alg = NS(actor=NS(reset=Mock()))

            def get_inference_policy(self):
                return lambda obs: obs

        self.original_key = Mock(return_value=False)
        self.original_config = Mock()
        self.sim = NS(
            UnitreeGo2CustomEnvCfg=self.original_config,
            sub_keyboard_event=self.original_key,
            OnPolicyRunner=Runner,
            RslRlVecEnvWrapper=type("Wrapper", (), {}),
            carb=NS(input=NS(KeyboardEventType=NS(KEY_PRESS="press"))),
            args_cli=NS(headless=True),
            RESET_REQUESTED=False,
            _ENV_REF=self.env,
        )

        def base_reset(env, controller=None):
            robot.data.root_state_w[:, :7] = self.sim.ROBOT_RESET_ROOT_POSE
            robot.data.root_state_w[:, 7:] = 0
            self.sim.RESET_REQUESTED = False

        self.sim.reset_robot_and_arm = Mock(side_effect=base_reset)
        self.original_reset = self.sim.reset_robot_and_arm
        self.runtime = CompetitionRuntime(
            self.sim, self.lanes, "/tmp/test.usdc", Options(gravel="static")
        )
        self.runtime.install()

    def key(self, name, kind="press"):
        return self.sim.sub_keyboard_event(NS(input=NS(name=name), type=kind))

    def test_jump_queues_then_resets_and_refreshes_before_policy(self):
        initial = self.env.unwrapped.scene["robot"].data.root_state_w.clone()
        self.key("F2")
        self.assertTrue(self.sim.RESET_REQUESTED)
        self.assertEqual(self.runtime.selection.current, 0)
        torch.testing.assert_close(
            self.env.unwrapped.scene["robot"].data.root_state_w, initial
        )
        self.sim.reset_robot_and_arm(self.env)
        self.assertEqual(self.runtime.selection.current, 1)
        self.original_reset.assert_called_once()
        torch.testing.assert_close(
            self.sim.ROBOT_RESET_ROOT_POSE[0, :3], torch.tensor(self.lanes[1].spawn)
        )
        self.assertEqual(
            torch.count_nonzero(self.env.unwrapped.action_manager.action), 0
        )
        self.assertEqual(self.env.unwrapped.episode_length_buf.item(), 0)
        self.env.unwrapped.scene["height_scanner"].update.assert_called_once_with(
            0.0, force_recompute=True
        )
        runner = self.sim.OnPolicyRunner()
        policy = runner.get_inference_policy()
        stale = {"policy": torch.ones((1, 235))}
        self.assertIs(policy(stale), self.env.get_observations.return_value)
        runner.alg.actor.reset.assert_called_once()
        self.assertIs(policy(stale), stale)

    def test_press_repeat_release_and_existing_controls(self):
        self.key("PAGE_DOWN")
        self.key("PAGE_DOWN", "repeat")
        self.key("PAGE_DOWN", "release")
        self.assertEqual(self.runtime.selection.pending, 1)
        self.key("PAGE_DOWN")
        self.assertEqual(self.runtime.selection.pending, 2)
        self.key("PAGE_DOWN")
        self.assertEqual(self.runtime.selection.pending, 0)
        self.key("F3")
        self.assertEqual(self.runtime.selection.pending, 2)
        self.assertFalse(self.key("W"))
        self.original_key.assert_called_once()

    def test_reset_stays_in_selected_arena(self):
        self.runtime.selection.current = 1
        self.sim.reset_robot_and_arm(self.env)
        self.assertEqual(self.runtime.selection.current, 1)
        self.assertEqual(self.sim.ROBOT_START_POS, list(self.lanes[1].spawn))

    def test_configuration_installs_explicit_scanner_and_start(self):
        config = NS(
            scene=NS(robot=NS(init_state=NS()), height_scanner=NS()),
            observations=NS(policy=NS()),
            events=NS(push_robot=object()),
        )
        self.original_config.return_value = config
        terrain_module = NS(TerrainImporterCfg=lambda **kwargs: NS(**kwargs))
        with patch.dict("sys.modules", {"isaaclab.terrains": terrain_module}):
            result = self.sim.UnitreeGo2CustomEnvCfg()
        self.assertEqual(result.scene.terrain.terrain_type, "usd")
        self.assertEqual(result.scene.terrain.usd_path, "/tmp/test.usdc")
        self.assertEqual(
            result.scene.height_scanner.mesh_prim_paths,
            ["/World/competition/terrain/WalkableScan"],
        )
        self.assertEqual(result.scene.robot.init_state.pos, self.lanes[0].spawn)
        self.assertIsNone(result.events.push_robot)


if __name__ == "__main__":
    unittest.main()
