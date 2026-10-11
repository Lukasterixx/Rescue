"""The K-Rail Square policy's bundle (`policies/go2_krail_2026-10-12`, from `training/export_krail.py`): the contract
is the stock rough layout with the 5 cm scan, the sim's legs and the training envelope; the files are the ones the
export recorded; and the policy stands still when standing still is what it sees."""
import hashlib
import json
from pathlib import Path
import unittest

import torch

from rescue_sim import exported_policy as xp

BUNDLE = Path(__file__).resolve().parents[2] / "policies" / "go2_krail_2026-10-12"
TERMS = ["base_lin_vel", "base_ang_vel", "projected_gravity", "velocity_commands", "joint_pos_rel", "joint_vel_rel",
         "last_action", "height_scan"]


class KRailBundleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle = xp.Bundle.load(BUNDLE)

    def test_the_contract(self):
        c = self.bundle.contract
        self.assertEqual(c.obs_terms, TERMS)
        self.assertEqual(c.obs_dim, 48 + 693)
        self.assertTrue(all(s is None for s in c.obs_scales.values()))
        self.assertEqual(c.obs_clips["height_scan"], (-1.0, 1.0))
        self.assertEqual(c.obs_history, {t: 1 for t in TERMS})
        self.assertEqual(c.command_ranges, ([-1.0, -0.8, -1.5], [1.0, 0.8, 1.5]))
        self.assertIsNone(c.action_clip)

    def test_the_sims_legs(self):
        c = self.bundle.contract
        self.assertEqual(c.step_dt, 0.02)
        self.assertEqual(c.action_scale, 0.25)
        self.assertEqual((c.kp, c.kd), (25.0, 0.5))
        self.assertEqual(c.default_joint_pos, [0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5])

    def test_a_full_scan_centred_on_the_base_at_5_cm(self):
        s = self.bundle.scanner
        self.assertEqual((s.offset, s.resolution, s.size, s.ray_alignment), ((0.0, 0.0, 20.0), 0.05, (1.6, 1.0), "yaw"))
        self.assertEqual(s.num_rays, 693)
        self.assertIsNone(self.bundle.mask)
        self.assertEqual(self.bundle.scan_offset, 0.5)

    def test_the_files_are_the_exported_ones(self):
        record = json.loads((BUNDLE / "params" / "export.json").read_text())
        for name in ("policy.pt", "policy.onnx"):
            self.assertEqual(hashlib.sha256((BUNDLE / name).read_bytes()).hexdigest(), record["sha256"][name])
        self.assertLess(record["onnx_vs_rebuilt_max_abs"], 1e-4)

    def test_standing_still_on_flat_ground(self):
        policy = xp.ExportedPolicy(self.bundle, "cpu")
        obs = torch.zeros(1, 741)
        obs[0, 8] = -1.0                 # gravity straight down
        obs[0, 48:] = 0.30 - 0.5         # flat ground 30 cm under the base
        with torch.inference_mode():
            action = policy({"policy": obs})
        self.assertEqual(tuple(action.shape), (1, 12))
        self.assertLess(float(action.abs().max()), 3.0)

    def test_it_is_the_sims_default(self):
        from rescue_sim import sim

        self.assertEqual(xp.resolve(sim.DEFAULT_POLICY, sim.SIM_DIR), BUNDLE.resolve())

    def test_the_action_is_clamped_as_training_clamped_it(self):
        policy = xp.ExportedPolicy(self.bundle, "cpu")
        with torch.inference_mode():
            action = policy(torch.full((1, 741), 1.0e3))
        self.assertLessEqual(float(action.abs().max()), 20.0)


if __name__ == "__main__":
    unittest.main()
