"""The level catalogue, the cup demo's placement and the posture ramp. No Isaac."""
import math
import unittest

import numpy as np

from competition.geometry import Options, build_lanes, ground_mesh
from rescue_sim import levels as lv


class CatalogueTests(unittest.TestCase):
    def setUp(self):
        self.lanes = build_lanes(Options(gravel="static"))
        self.levels = lv.catalogue(self.lanes)

    def test_every_lane_is_a_level_and_the_cup_demo_comes_last(self):
        self.assertEqual([level.key for level in self.levels[:-1]], [lane.key for lane in self.lanes])
        self.assertEqual(self.levels[-1], lv.CUP_DEMO)
        self.assertEqual(len({level.key for level in self.levels}), len(self.levels))
        for level, lane in zip(self.levels, self.lanes):
            self.assertEqual(level.spawn, tuple(lane.spawn))
            self.assertEqual(level.posture, lv.STANDING)
        self.assertEqual(lv.CUP_DEMO.posture, lv.LYING)
        # The cup demo's robot stays lying; every lane lets it lie down and stand up.
        self.assertTrue(lv.CUP_DEMO.posture_fixed)
        self.assertFalse(any(level.posture_fixed for level in self.levels[:-1]))

    def test_the_cup_demo_sits_on_the_hall_floor_clear_of_every_lane(self):
        floor = ground_mesh(self.lanes)
        low, high = floor.vertices.min(axis=0), floor.vertices.max(axis=0)
        cup = lv.cup_pose(lv.CUP_DEMO)
        for x, y in (lv.CUP_DEMO.spawn[:2], cup[:2]):
            self.assertTrue(low[0] + 1.0 < x < high[0] and low[1] + 1.0 < y < high[1] - 1.0, (x, y))
        # Nothing of any lane within a metre of the robot or the cup.
        structure = np.vstack([mesh.vertices for lane in self.lanes for mesh in lane.meshes if mesh.collision])
        for point in (np.array(lv.CUP_DEMO.spawn[:2]), np.array(cup[:2])):
            self.assertGreater(np.min(np.linalg.norm(structure[:, :2] - point, axis=1)), 1.0)

    def test_clear_and_debris_stairs_have_separate_selectable_levels(self):
        levels = {level.key: level for level in self.levels}
        clear, debris = levels["stairs"], levels["stair_debris"]
        self.assertNotEqual(clear.title, debris.title)
        self.assertGreater(debris.spawn[0] - clear.spawn[0], 7.0)
        np.testing.assert_allclose(clear.spawn[1:], debris.spawn[1:])

    def test_the_window_gives_each_arena_one_row_of_settings(self):
        rows = lv.rows(self.levels)
        by_group = {group: [self.levels[i] for i in indices] for group, indices in rows}
        self.assertEqual(len(rows), len(by_group))
        self.assertEqual(sorted(i for _, indices in rows for i in indices), list(range(len(self.levels))))
        # Arenas in hall order, the copies at the far end of the hall in their arena's row.
        self.assertEqual([group for group, _ in rows], [
            "Shifty Gravel", "Diagonal K-Rails", "K-Rail Square", "Half-Cubic Stepfields", "Pitch/Roll Ramps",
            "Center in Alleys", "Pallets & Pipes", "Push/Pull Doors", "Avoid Holes/Posts", "Stairs | Pallet Climb",
            "Search & Map Maze", "Cup demo"])
        for group in ("Diagonal K-Rails", "Half-Cubic Stepfields", "Pitch/Roll Ramps"):
            self.assertEqual([level.label for level in by_group[group]], ["Flat", "Slopes 15°", "Obstacles"])
        self.assertEqual([level.key for level in by_group["Pitch/Roll Ramps"]],
                         ["ramps", "ramps_slopes", "ramps_obstacles"])
        self.assertEqual([level.label for level in by_group["Center in Alleys"]], ["Flat", "Slopes 15°"])
        self.assertEqual([level.label for level in by_group["Stairs | Pallet Climb"]], ["Clear", "Debris"])
        # An arena with one level gets a single button with no setting on it.
        self.assertEqual([level.label for level in by_group["Search & Map Maze"]], [""])
        self.assertEqual(by_group["Cup demo"], [lv.CUP_DEMO])

    def test_the_cup_is_where_d1training_puts_it_relative_to_the_robot(self):
        x, y, z, *quat = lv.cup_pose(lv.CUP_DEMO)
        yaw = math.radians(lv.CUP_DEMO.yaw_deg)
        dx, dy = x - lv.CUP_DEMO.spawn[0], y - lv.CUP_DEMO.spawn[1]
        forward = dx * math.cos(yaw) + dy * math.sin(yaw)
        left = -dx * math.sin(yaw) + dy * math.cos(yaw)
        self.assertAlmostEqual(forward, 0.42, places=9)
        self.assertAlmostEqual(left, 0.03, places=9)
        self.assertAlmostEqual(z, 0.001)
        cup_yaw = math.degrees(2 * math.atan2(quat[3], quat[0]))
        self.assertAlmostEqual((cup_yaw - lv.CUP_DEMO.yaw_deg + 180) % 360 - 180, 0.0, places=9)

    def test_random_cups_stay_in_d1trainings_band(self):
        rng = np.random.default_rng(3)
        for _ in range(200):
            (x, y), handle = lv.random_cup(rng)
            self.assertTrue(0.36 <= x <= 0.44 and -0.10 <= y <= 0.10)
            away = math.degrees(math.atan2(y, x))
            off = (handle - away + 180) % 360 - 180
            self.assertTrue(abs(off) <= 45 or abs(abs(off) - 180) <= 45, off)

    def test_the_lying_pose_resolves_by_name(self):
        names = [f"{leg}_{part}_joint" for part in ("hip", "thigh", "calf") for leg in ("FL", "FR", "RL", "RR")]
        names.insert(5, "Joint1")
        pose = lv.leg_pose(names, lv.LYING_LEG_POSE)
        self.assertNotIn(5, pose)
        by_name = {names[i]: v for i, v in pose.items()}
        self.assertEqual(by_name["RR_hip_joint"], -0.2)
        self.assertEqual(by_name["RL_hip_joint"], 0.2)
        self.assertEqual(by_name["FL_thigh_joint"], 1.36)
        self.assertEqual(by_name["RR_calf_joint"], -2.65)
        self.assertEqual(len(pose), 12)


class SelectionTests(unittest.TestCase):
    def test_requests_queue_until_the_loop_takes_them(self):
        selection = lv.Selection(4)
        self.assertEqual(selection.consume(), 0)
        selection.select(3)
        selection.select(1)
        self.assertEqual(selection.consume(), 1)
        self.assertIsNone(selection.pending)
        self.assertEqual(selection.consume(), 1)
        with self.assertRaises(IndexError):
            selection.select(4)


class PostureTests(unittest.TestCase):
    def test_lying_holds_and_standing_hands_back_to_the_policy(self):
        ramp = lv.PostureRamp(duration_s=1.0)
        lying, standing = np.full(12, -1.0), np.zeros(12)
        ramp.set(lv.LYING, hold=lying)
        self.assertFalse(ramp.walking)
        np.testing.assert_array_equal(ramp.targets(0.02), lying)
        ramp.begin(lv.STANDING, lying, standing)
        halfway = [ramp.targets(0.02) for _ in range(25)][-1]
        np.testing.assert_allclose(halfway, np.full(12, -0.5))
        for _ in range(25):
            out = ramp.targets(0.02)
        np.testing.assert_allclose(out, standing)
        self.assertTrue(ramp.resumed)
        self.assertTrue(ramp.walking)
        self.assertIsNone(ramp.targets(0.02))
        self.assertFalse(ramp.resumed)

    def test_lying_down_ends_holding_the_pose(self):
        ramp = lv.PostureRamp(duration_s=0.5)
        ramp.begin(lv.LYING, np.zeros(12), np.ones(12))
        for _ in range(100):
            out = ramp.targets(0.02)
        np.testing.assert_array_equal(out, np.ones(12))
        self.assertFalse(ramp.walking)
        with self.assertRaises(ValueError):
            ramp.set(lv.LYING)


if __name__ == "__main__":
    unittest.main()
