"""Stair details from printed pp. 51–54, including the PDF viewer's page 53."""

import argparse
import unittest

import numpy as np

from competition.build import add_geometry_args, options_from_args
from competition.geometry import Options
from competition.structures import stair_lane


class StairDetailsTests(unittest.TestCase):
    def meshes(self, angle=45, count=3, debris=True):
        lane = stair_lane(Options(stair_angle=angle, stair_debris=count), (0, 0, 0), debris=debris)
        return lane, {m.name: m for m in lane.meshes}

    def test_two_variants_share_the_structure_and_keep_clear_stairs_clear(self):
        parser = argparse.ArgumentParser()
        add_geometry_args(parser)
        self.assertEqual(options_from_args(parser.parse_args([])).stair_debris, 3)
        for count in range(4):
            clear, clean = self.meshes(count=count, debris=False)
            debris, cluttered = self.meshes(count=count)
            self.assertEqual((clear.key, debris.key), ("stairs", "stair_debris"))
            self.assertEqual([m for m in clean if m.startswith("debris")], [])
            self.assertEqual(len([m for m in cluttered if m.startswith("debris")]), count)
            for name, mesh in clean.items():
                np.testing.assert_allclose(mesh.vertices, cluttered[name].vertices, err_msg=name)
            for name in ("landing_brace_over", "landing_brace_under"):
                self.assertNotIn(name, clean)
                self.assertEqual(cluttered[name].material, "red")
                self.assertTrue(cluttered[name].collision)

    def test_all_four_mitred_rails_touch_the_wall_and_a_grounded_upright(self):
        for angle in (35, 40, 45):
            _, meshes = self.meshes(angle)
            rails = [m for name, m in meshes.items() if name.startswith("stair_rail")]
            self.assertEqual(len(rails), 4)
            for side in range(2):
                wall = meshes[f"stair_wall{side}"].vertices
                post = meshes[f"stair_belay_post{side}"].vertices
                for j in range(2):
                    rail = meshes[f"stair_rail{side}_{j}"].vertices
                    low = rail[np.isclose(rail[:, 1], rail[:, 1].min())]
                    high = rail[np.isclose(rail[:, 1], rail[:, 1].max())]
                    self.assertAlmostEqual(np.linalg.norm(high.mean(axis=0) - low.mean(axis=0)), 1.2)
                    # The low mitre meets the outer OSB face below its top, within its length.
                    wall_x = wall[:, 0].min() if side == 0 else wall[:, 0].max()
                    rail_x = rail[:, 0].max() if side == 0 else rail[:, 0].min()
                    self.assertAlmostEqual(wall_x, rail_x)
                    self.assertLess(low[:, 2].min(), wall[:, 2].max())
                    self.assertGreaterEqual(low[:, 1].min(), wall[:, 1].min())
                    # The high mitre terminates inside the 2.4 m upright, not in free air.
                    self.assertTrue(np.all(high >= post.min(axis=0) - 1e-9))
                    self.assertTrue(np.all(high <= post.max(axis=0) + 1e-9))

    def test_belay_cut_lengths_and_entry_placement_at_each_incline(self):
        for angle in (35, 40, 45):
            _, meshes = self.meshes(angle)
            for name, width, axis in (("stair_belay", 1.0, 0), ("pallet_belay", 1.4, 1)):
                for i in range(2):
                    post = meshes[f"{name}_post{i}"]
                    self.assertAlmostEqual(post.vertices[:, 2].min(), 0.0)
                    self.assertAlmostEqual(post.vertices[:, 2].max(), 2.4)
                cap = meshes[f"{name}_top"]
                self.assertAlmostEqual(np.ptp(cap.vertices[:, axis]), width)
                self.assertAlmostEqual(cap.vertices[:, 2].max(), 2.4)
                self.assertFalse(meshes[f"{name}_rope"].collision)
            self.assertAlmostEqual(meshes["entry_osb"].vertices[:, 1].max(),
                                   meshes["tread1"].vertices[:, 1].min())

    def test_debris_is_rectangular_lumber_seated_on_one_tread_and_the_wall(self):
        for angle in (35, 40, 45):
            _, meshes = self.meshes(angle)
            for k, color in enumerate(("yellow", "orange", "red")):
                debris = meshes[f"debris{k}"]
                self.assertEqual(debris.material, color)
                self.assertTrue(debris.collision)
                self.assertTrue(debris.scan)
                v = debris.vertices
                # A rigid rotation of a 75 x 5 x 9 cm box: three orthogonal edges.
                edges = v[[1, 3, 4]] - v[0]
                np.testing.assert_allclose(edges @ edges.T, np.diag(np.array([0.75, 0.05, 0.09])**2), atol=1e-12)
                self.assertAlmostEqual(np.ptp(v[:, 1]), 0.05)
                self.assertGreater(np.ptp(v[:, 2]), 0.4)
                tread = meshes[f"tread{2 * k + 1}"].vertices
                self.assertAlmostEqual(v[:, 2].min(), tread[:, 2].max())
                foot = v[np.isclose(v[:, 2], v[:, 2].min())]
                self.assertTrue(np.all(foot[:, :2] >= tread[:, :2].min(axis=0) - 1e-9))
                self.assertTrue(np.all(foot[:, :2] <= tread[:, :2].max(axis=0) + 1e-9))
                # The entire board stays over a single tread, not across successive risers.
                self.assertGreaterEqual(v[:, 1].min(), tread[:, 1].min())
                self.assertLessEqual(v[:, 1].max(), tread[:, 1].max())
                right = k % 2 == 0
                wall = meshes["stair_belay_post1" if k == 2 else f"stair_wall{1 if right else 0}"].vertices
                contact_x = v[:, 0].max() if right else v[:, 0].min()
                self.assertAlmostEqual(contact_x, wall[:, 0].min() if right else wall[:, 0].max())
                contact = v[np.isclose(v[:, 0], contact_x)]
                self.assertTrue(np.all(contact >= wall.min(axis=0) - 1e-9))
                self.assertTrue(np.all(contact <= wall.max(axis=0) + 1e-9))

    def test_red_braces_cross_the_upper_and_lower_paths_at_the_landing_centre(self):
        _, meshes = self.meshes()
        post = meshes["landing_brace_post"].vertices
        for name, floor in (("under", 0.0), ("over", 1.0)):
            beam = meshes[f"landing_brace_{name}"]
            self.assertTrue(beam.collision)
            self.assertTrue(beam.scan)
            v = beam.vertices
            # The top-view footprint cuts across the landing's depth at X=0.
            # The old wall decoration instead extended along X at the rear edge.
            self.assertAlmostEqual(v[:, 0].mean(), 0.0)
            self.assertAlmostEqual(np.ptp(v[:, 0]), 0.05)
            self.assertLess(v[:, 1].min(), 0.10)
            self.assertGreater(v[:, 1].max(), 1.05)
            self.assertAlmostEqual(v[:, 2].min(), floor)
            self.assertGreater(np.ptp(v[:, 2]), 0.8)
            # High end meets the front post; the low end rests on the floor.
            high = v[np.isclose(v[:, 1], v[:, 1].min())]
            self.assertTrue(np.all(high >= post.min(axis=0) - 1e-9))
            self.assertTrue(np.all(high <= post.max(axis=0) + 1e-9))


if __name__ == "__main__":
    unittest.main()
