"""The Go2-W's K-Rail Square policy (the bundle `sim.DEFAULT_POLICIES["go2w"]` names, from `training/export_krail.py`):
the contract is the Go2 bundle's layout with the wheels after the legs, the sim's Go2-W and the training envelope;
the files are the ones the export recorded; and the policy stands still when standing still is what it sees."""
import hashlib
import json
import unittest

import torch

from rescue_sim import exported_policy as xp
from rescue_sim import sim

TERMS = ["base_lin_vel", "base_ang_vel", "projected_gravity", "velocity_commands", "joint_pos_rel", "joint_vel_rel",
         "last_action", "height_scan"]
WIDTH = 48 + 8 + 693


class Go2WKRailBundleTest(unittest.TestCase):
    BUNDLE = xp.resolve(sim.DEFAULT_POLICIES["go2w"], sim.SIM_DIR)

    @classmethod
    def setUpClass(cls):
        cls.bundle = xp.Bundle.load(cls.BUNDLE)

    def test_it_drives_the_go2w(self):
        self.assertEqual((self.bundle.robot, self.bundle.num_actions), ("go2w", 16))

    def test_the_contract(self):
        c = self.bundle.contract
        self.assertEqual(c.obs_terms, TERMS)
        self.assertEqual(c.obs_dim, WIDTH)
        self.assertEqual((c.obs_widths["joint_pos_rel"], c.obs_widths["joint_vel_rel"], c.obs_widths["last_action"]),
                         (12, 16, 16))
        self.assertTrue(all(s is None for s in c.obs_scales.values()))
        self.assertEqual(c.obs_clips["height_scan"], (-1.0, 1.0))
        self.assertEqual(c.command_ranges, ([-1.0, -0.5, -1.5], [1.0, 0.5, 1.5]))
        self.assertIsNone(c.action_clip)

    def test_the_sims_legs_and_wheels(self):
        c, w = self.bundle.contract, self.bundle.wheels
        self.assertEqual(c.step_dt, 0.02)
        self.assertEqual(c.action_scale, 0.25)
        self.assertEqual((c.kp, c.kd), (25.0, 0.5))
        self.assertEqual(c.default_joint_pos, [0.0] * 4 + [0.8] * 4 + [-1.5] * 4)
        self.assertEqual((w.joint_names, w.action_scale, w.stiffness, w.damping), (xp.WHEEL_ORDER, 5.0, 0.0, 0.5))

    def test_a_full_scan_centred_on_the_base_at_5_cm(self):
        s = self.bundle.scanner
        self.assertEqual((s.offset, s.resolution, s.size, s.ray_alignment), ((0.0, 0.0, 20.0), 0.05, (1.6, 1.0), "yaw"))
        self.assertIsNone(self.bundle.mask)

    def test_the_files_are_the_exported_ones(self):
        record = json.loads((self.BUNDLE / "params" / "export.json").read_text())
        for name in ("policy.pt", "policy.onnx"):
            self.assertEqual(hashlib.sha256((self.BUNDLE / name).read_bytes()).hexdigest(), record["sha256"][name])
        self.assertLess(record["onnx_vs_rebuilt_max_abs"], 1e-4)

    def test_standing_still_on_flat_ground(self):
        policy = xp.ExportedPolicy(self.bundle, "cpu")
        obs = torch.zeros(1, WIDTH)
        obs[0, 8] = -1.0                 # gravity straight down
        obs[0, 56:] = 0.41 - 0.5         # flat ground 41 cm under the base, the Go2-W's standing height
        with torch.inference_mode():
            action = policy({"policy": obs})
        self.assertEqual(tuple(action.shape), (1, 16))
        self.assertLess(float(action[0, :12].abs().max()), 3.0)
        self.assertLess(float(action[0, 12:].abs().max()), 1.0)    # under 5 rad/s at the wheels: it does not roll off

    def test_the_action_is_clamped_as_training_clamped_it(self):
        policy = xp.ExportedPolicy(self.bundle, "cpu")
        with torch.inference_mode():
            action = policy(torch.full((1, WIDTH), 1.0e3))
        self.assertLessEqual(float(action.abs().max()), 20.0)


if __name__ == "__main__":
    unittest.main()
