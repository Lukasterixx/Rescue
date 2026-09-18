"""The sim's copies of D1Training's arm and camera models, held to what D1Training computes (fixtures/d1training.json,
written by make_fixtures.py). Needs torch and numpy; no Isaac."""
import json
import math
from pathlib import Path
import unittest

import numpy as np

try:
    import torch
except ImportError:
    torch = None

from rescue_sim import d1_model, realsense

FIXTURE = json.loads((Path(__file__).resolve().parent / "fixtures" / "d1training.json").read_text())


class MotorModelTests(unittest.TestCase):
    def test_limits_and_timing_match(self):
        motor = FIXTURE["motor"]
        self.assertEqual([d1_model.EFFORT_LIMIT_NM[j] for j in d1_model.ARM_JOINTS], motor["effort_limit_nm"])
        self.assertEqual([d1_model.VELOCITY_LIMIT_RAD_S[j] for j in d1_model.ARM_JOINTS], motor["velocity_limit_rad_s"])
        trajectory = motor["trajectory"]
        self.assertEqual(d1_model.COMMAND_DEAD_TIME_S, trajectory["dead_time_s"])
        self.assertEqual(d1_model.PLAN_ACCEL_RAD_S2, trajectory["accel_rad_s2"])
        self.assertEqual(d1_model.PLAN_DECEL_RAD_S2, trajectory["decel_rad_s2"])
        self.assertEqual(d1_model.REPLAN_VELOCITY_RETENTION, trajectory["replan_velocity_retention"])
        self.assertEqual(d1_model.FEEDBACK_HZ, motor["feedback_hz"])
        self.assertEqual(d1_model.COMMAND_HZ, motor["command_hz"])
        steps = d1_model.interface_steps(50.0)
        self.assertEqual(steps["command_hold_steps"], motor["timing_50hz"]["arm_command_hold_steps"])
        self.assertEqual(steps["feedback_period_steps"], motor["timing_50hz"]["arm_feedback_period_steps"])


@unittest.skipIf(torch is None, "PyTorch unavailable in this interpreter")
class PlannerTests(unittest.TestCase):
    def test_every_script_reproduces_d1training(self):
        for name, recorded in FIXTURE["trajectories"].items():
            with self.subTest(script=name):
                planner = d1_model.make_planner()
                planner.reset(None, torch.zeros(1, 6))
                script = {int(k): v for k, v in recorded["script"].items()}
                positions, velocities = [], []
                for policy_step in range(60):
                    if policy_step in script:
                        planner.command(torch.ones(1, dtype=torch.bool), torch.tensor([script[policy_step]]))
                    for _ in range(4):
                        planner.step(0.005)
                        positions.append(planner.position[0].tolist())
                        velocities.append(planner.velocity[0].tolist())
                np.testing.assert_allclose(positions, recorded["position"], atol=1e-6)
                np.testing.assert_allclose(velocities, recorded["velocity"], atol=1e-6)

    def test_a_single_step_cruises_at_the_ceiling(self):
        planner = d1_model.make_planner()
        planner.reset(None, torch.zeros(1, 6))
        planner.command(torch.ones(1, dtype=torch.bool), torch.full((1, 6), 1.0))
        peak = torch.zeros(6)
        for _ in range(400):
            planner.step(0.005)
            peak = torch.maximum(peak, planner.velocity[0].abs())
        ceiling = torch.tensor([d1_model.VELOCITY_LIMIT_RAD_S[j] for j in d1_model.ARM_JOINTS])
        torch.testing.assert_close(peak, ceiling)
        torch.testing.assert_close(planner.position, torch.full((1, 6), 1.0))


class CameraTests(unittest.TestCase):
    def test_the_calibration_gives_d1trainings_camera(self):
        model = realsense.camera_model(realsense.load_calibration(realsense.DEFAULT_CALIBRATION))
        for key, value in FIXTURE["camera_model"].items():
            self.assertEqual(getattr(model, key), value, key)
        self.assertEqual(realsense.resolve_camera("calibration"), model)

    def test_the_d435_preset_matches(self):
        preset = realsense.PRESETS["d435"]
        for key, value in FIXTURE["d435_preset"].items():
            self.assertAlmostEqual(getattr(preset, key), value, places=9, msg=key)

    def test_the_mount_matches(self):
        mount = realsense.load_mount(realsense.DEFAULT_MOUNT)
        np.testing.assert_allclose(mount.pose, FIXTURE["mount"]["pose"], atol=1e-12)
        np.testing.assert_allclose(mount.quat_wxyz(), FIXTURE["mount"]["quat_wxyz"], atol=1e-12)
        self.assertIn("aligned by eye", mount.source)

    def test_depth_limits_and_noise_match(self):
        model = realsense.resolve_camera("calibration")
        depth = np.asarray(FIXTURE["depth"]["true"], dtype=np.float32)
        depth[tuple(FIXTURE["depth"]["nan_at"])] = np.nan
        np.testing.assert_array_equal(realsense.realsense_depth(model, depth, None), FIXTURE["depth"]["clean"])
        np.testing.assert_array_equal(realsense.realsense_depth(model, depth, np.random.default_rng(7)),
                                      np.asarray(FIXTURE["depth"]["noisy_seed_7"], dtype=np.float32))

    def test_near_clip_and_case_registration_match(self):
        from rescue_sim import camera_asset

        self.assertEqual(realsense.NEAR_CLIP_M, FIXTURE["near_clip_m"])
        np.testing.assert_array_equal(camera_asset.MESH_TO_OPTICAL, FIXTURE["mesh_to_optical"])


if __name__ == "__main__":
    unittest.main()
