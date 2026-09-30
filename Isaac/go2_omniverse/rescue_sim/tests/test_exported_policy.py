"""The exported-policy path (`--policy`): the 2026-09-18 bundles load and describe themselves as their README does,
the rebuilt camera mask sees what the README says it sees, and the checks refuse what would run silently wrong."""
import copy
import math
from pathlib import Path
import tempfile
import unittest

import torch
import yaml

from rescue_sim import exported_policy as xp
from rescue_sim.deploy_contract import DeployContract

BUNDLES = Path(__file__).resolve().parents[2] / "policies" / "go2_policies_2026-09-18"


def grid_hits(scanner, base_xy=(0.0, 0.0), yaw=0.0, ground_z=0.0):
    """Where a yaw-aligned downward grid (Isaac Lab's grid_pattern, `xy` ordering) hits flat ground."""
    sx, sy = scanner.size
    x = torch.arange(-sx / 2, sx / 2 + 1e-9, scanner.resolution)
    y = torch.arange(-sy / 2, sy / 2 + 1e-9, scanner.resolution)
    gx, gy = torch.meshgrid(x, y, indexing=scanner.ordering)
    px = gx.flatten() + scanner.offset[0]
    py = gy.flatten() + scanner.offset[1]
    c, s = math.cos(yaw), math.sin(yaw)
    hits = torch.stack([base_xy[0] + c * px - s * py, base_xy[1] + s * px + c * py, torch.full_like(px, ground_z)], -1)
    return hits[None]


def base_pose(z, yaw=0.0, xy=(0.0, 0.0)):
    return (torch.tensor([[xy[0], xy[1], z]]),
            torch.tensor([[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]]))


class BundleTest(unittest.TestCase):
    def test_flat_is_the_scaled_45_wide_layout(self):
        bundle = xp.Bundle.load(BUNDLES / "flat")
        c = bundle.contract
        self.assertEqual(c.obs_dim, 45)
        self.assertEqual(c.obs_terms, ["base_ang_vel", "projected_gravity", "velocity_commands", "joint_pos_rel",
                                       "joint_vel_rel", "last_action"])
        self.assertEqual(c.obs_scales["base_ang_vel"], [0.2] * 3)
        self.assertEqual(c.obs_scales["joint_vel_rel"], [0.05] * 12)
        self.assertEqual(c.obs_clips["base_ang_vel"], (-100.0, 100.0))
        self.assertEqual(c.command_ranges, ([-1.5] * 3, [1.5] * 3))
        self.assertIsNone(bundle.scanner)
        self.assertIsNone(bundle.mask)

    def test_rough_is_unscaled_with_a_masked_scan_ahead(self):
        bundle = xp.Bundle.load(BUNDLES / "rough")
        c = bundle.contract
        self.assertEqual(c.obs_dim, 232)
        self.assertEqual(c.obs_terms[-1], "height_scan")
        self.assertTrue(all(s is None for s in c.obs_scales.values()))
        self.assertEqual(c.obs_clips["height_scan"], (-1.0, 1.0))
        self.assertEqual(c.height_scan_unobserved, 0.0)
        self.assertEqual(c.command_ranges, ([-1.0] * 3, [1.0] * 3))
        self.assertEqual(bundle.scanner.offset, (0.6, 0.0, 20.0))
        self.assertEqual(bundle.scanner.num_rays, 187)
        self.assertEqual(bundle.scanner.ray_alignment, "yaw")
        self.assertEqual(bundle.scan_offset, 0.5)
        self.assertIsInstance(bundle.mask, xp.CameraMask)

    def test_both_share_the_sims_legs(self):
        for name in ("flat", "rough"):
            c = xp.Bundle.load(BUNDLES / name).contract
            self.assertEqual(c.step_dt, 0.02)
            self.assertEqual(c.action_scale, 0.25)
            self.assertEqual((c.kp, c.kd), (25.0, 0.5))
            # env_cfg.STANDING_LEG_POSE, in Isaac Lab order.
            self.assertEqual(c.default_joint_pos, [0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5])

    def test_the_torchscript_policy_runs_on_the_wrappers_observations(self):
        for name, width in (("flat", 45), ("rough", 232)):
            policy = xp.ExportedPolicy(xp.Bundle.load(BUNDLES / name), "cpu")
            obs = torch.zeros(1, width)
            obs[0, 5] = -1.0                                   # upright
            with torch.inference_mode():
                a = policy({"policy": obs})
                self.assertEqual(tuple(a.shape), (1, 12))
                self.assertTrue(torch.equal(a, policy(obs)))
            # The bundle README's sanity check: at rest the action stays small.
            self.assertLess(float(a.abs().max()), 2.0)

    def test_resolve_is_relative_to_the_sim_directory(self):
        sim_dir = BUNDLES.parents[1]
        self.assertEqual(xp.resolve("policies/go2_policies_2026-09-18/flat", sim_dir), BUNDLES / "flat")
        with self.assertRaises(xp.ExportedPolicyError):
            xp.resolve("policies/nowhere", sim_dir)


class CameraMaskTest(unittest.TestCase):
    def setUp(self):
        self.bundle = xp.Bundle.load(BUNDLES / "rough")

    def test_97_of_187_cells_on_flat_ground_at_a_standing_height(self):
        # The bundle README: "only the 97 of 187 cells a RealSense D435i could actually see carry real heights".
        for z in (0.28, 0.30, 0.32, 0.34):
            for yaw in (0.0, 1.0, -2.5):
                pos, quat = base_pose(z, yaw, xy=(3.0, -2.0))
                seen = self.bundle.mask.visible(grid_hits(self.bundle.scanner, (3.0, -2.0), yaw), pos, quat)
                self.assertEqual(int(seen.sum()), 97, f"base at {z} m, yaw {yaw}")

    def test_it_sees_from_0_6_m_ahead_and_nothing_behind(self):
        pos, quat = base_pose(0.32)
        hits = grid_hits(self.bundle.scanner)
        seen = self.bundle.mask.visible(hits, pos, quat)[0]
        ahead = hits[0, :, 0]
        self.assertTrue(bool(seen[ahead > 0.65].all()))            # 88 cells from 0.7 m on
        self.assertEqual(int(seen[(ahead > 0.55) & (ahead < 0.65)].sum()), 9)   # the 0.6 m row loses its corners
        self.assertFalse(bool(seen[ahead < 0.55].any()))

    def test_scan_values(self):
        pos, quat = base_pose(0.35)
        hits = grid_hits(self.bundle.scanner)
        hits[0, -1] = float("inf")                            # a ray that hit nothing, in the far corner
        h = xp.height_scan(pos, quat, hits, 0.5, self.bundle.mask, 0.0)
        seen = self.bundle.mask.visible(hits, pos, quat)[0]
        self.assertTrue(torch.allclose(h[0, seen], torch.full_like(h[0, seen], -0.15)))
        self.assertTrue(bool((h[0, ~seen] == 0.0).all()))
        self.assertEqual(float(h[0, -1]), 0.0)
        self.assertEqual(int(seen.sum()), 96)
        # Without a mask it is stock Isaac Lab's height_scan: every cell, misses included.
        stock = xp.height_scan(pos, quat, hits, 0.5, None, 0.0)
        self.assertTrue(torch.allclose(stock[0, :-1], torch.full_like(stock[0, :-1], -0.15)))
        self.assertEqual(float(stock[0, -1]), -float("inf"))


class RefusalTest(unittest.TestCase):
    def contract(self, name="flat"):
        data = yaml.safe_load((BUNDLES / name / "params" / "deploy.yaml").read_text())
        return data, DeployContract(copy.deepcopy(data), source=name)

    def test_the_legs_must_be_in_isaac_lab_order_from_the_default_pose(self):
        _, c = self.contract()
        names = list(xp.ISAAC_LAB_LEG_ORDER)
        self.assertEqual(xp.leg_problems(c, names, c.default_joint_pos), [])
        swapped = names[:]
        swapped[0], swapped[2] = swapped[2], swapped[0]            # FL_hip <-> RL_hip
        self.assertTrue(xp.leg_problems(c, swapped, c.default_joint_pos))
        pose = list(c.default_joint_pos)
        pose[4] = 1.0                                              # FL_thigh at the rear's angle
        self.assertTrue(any("default leg pose" in p for p in xp.leg_problems(c, names, pose)))

    def test_terms_the_sim_does_not_build_are_refused(self):
        data, _ = self.contract()
        data["observations"]["gait_phase"] = {"params": {"period": 0.5}, "scale": [1.0, 1.0], "clip": None}
        data["observation_order"].append("gait_phase")
        problems = xp.contract_problems(DeployContract(data, source="gait"))
        self.assertTrue(any("gait_phase" in p for p in problems))

    def test_stacked_history_is_refused(self):
        data, _ = self.contract()
        data["observations"]["base_ang_vel"]["history_length"] = 3
        self.assertTrue(xp.contract_problems(DeployContract(data, source="history")))

    def test_env_yaml_is_read_without_running_it(self):
        text = "a: !!python/tuple [1, 2]\nb: !!python/object/apply:os.system ['echo ran']\nc: !!python/name:os.getcwd ''\n"
        data = yaml.load(text, Loader=xp._EnvYamlLoader)
        self.assertEqual(data, {"a": (1, 2), "b": None, "c": None})
        # SafeLoader itself is left alone.
        with self.assertRaises(yaml.constructor.ConstructorError):
            yaml.safe_load(text)

    def test_a_scan_function_the_sim_does_not_know_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "rough"
            (root / "params").mkdir(parents=True)
            (root / "policy.pt").write_bytes((BUNDLES / "rough" / "policy.pt").read_bytes())
            (root / "params" / "deploy.yaml").write_text((BUNDLES / "rough" / "params" / "deploy.yaml").read_text())
            env = (BUNDLES / "rough" / "params" / "env.yaml").read_text()
            (root / "params" / "env.yaml").write_text(
                env.replace("camera_rough_env_cfg:height_scan_camera_fov", "camera_rough_env_cfg:height_scan_lidar"))
            with self.assertRaises(xp.ExportedPolicyError) as caught:
                xp.Bundle.load(root)
            self.assertIn("height_scan_lidar", str(caught.exception))


if __name__ == "__main__":
    unittest.main()
