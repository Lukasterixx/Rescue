"""Dimension/topology checks against the fabrication drawings (no Isaac needed)."""

import math
import unittest

import numpy as np

from competition.geometry import (
    ACUITY_TARGETS,
    DECK,
    OSB,
    TASK_ELEVATION,
    Options,
    build_lanes,
    stone_shape,
)
from competition.runtime import Selection


class LaneGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.flat = build_lanes()
        cls.slopes = build_lanes(Options(difficulty="slopes", k_rail_height=0.2))

    def test_four_floors_and_fourteen_railings_per_lane(self):
        for lane in self.flat[:2]:
            self.assertEqual(len(lane.floors), 4)
            self.assertEqual(lane.railing_count, 14)
            self.assertAlmostEqual(sum(np.prod(f.size) for f in lane.floors), 4.8 * 2.4)
            self.assertEqual(lane.floors[0].center, (-1.8, 0.05))
            self.assertEqual(lane.floors[3].center, (1.8, -0.05))

    def test_eight_k_rails_on_eight_square_backings(self):
        meshes = self.flat[1].meshes
        rails = [m for m in meshes if "_k_" in m.name]
        backings = [m for m in meshes if "_backing_" in m.name]
        self.assertEqual(len(rails), 8)
        self.assertEqual(len(backings), 8)
        for rail in rails:
            self.assertAlmostEqual(np.ptp(rail.vertices[:, 2]), 0.1)
            self.assertAlmostEqual(rail.vertices[:, 2].min(), DECK + OSB)
            np.testing.assert_allclose(np.ptp(rail.vertices[:, :2], axis=0), (1.2, 1.2))
        # Both centre lanes converge at the centre of the arena (guide p. 32).
        for rail in (m for m in rails if m.name.startswith(("lower", "upper"))):
            self.assertTrue(
                np.any(np.linalg.norm(rail.vertices[:, :2] - [7.0, 0.0], axis=1) < 1e-8)
            )

    def test_four_gravel_xs_and_unobstructed_end_floors(self):
        rails = [m for m in self.flat[0].meshes if "_x_" in m.name]
        self.assertEqual(len(rails), 8)
        self.assertFalse(
            any(m.name.startswith(("blue_end", "green_end")) for m in rails)
        )
        self.assertGreater(len(self.flat[0].stones), 7000)
        self.assertLess(len(self.flat[0].stones), 12000)

    def test_15_degree_opposing_floors(self):
        for lane in self.slopes[:2]:  # the square's floors stay flat
            self.assertAlmostEqual(lane.floors[1].pitch, math.radians(15))
            self.assertAlmostEqual(lane.floors[2].pitch, -math.radians(15))
            for floor in lane.floors[1:3]:
                points = floor.transform([[-1.2, 0, 0], [1.2, 0, 0]])
                self.assertAlmostEqual(points[:, 2].min(), DECK)
                self.assertAlmostEqual(
                    abs(np.diff(points[:, 2])[0]), 2.4 * math.sin(math.radians(15))
                )
                self.assertAlmostEqual(np.linalg.norm(points[1] - points[0]), 2.4)

    def test_meshes_are_closed_outward_and_non_degenerate(self):
        for lane in self.flat + self.slopes:
            for mesh in lane.meshes:
                v, f = mesh.vertices, mesh.faces
                self.assertTrue(np.isfinite(v).all(), mesh.name)
                tri = v[f]
                self.assertTrue(
                    (
                        np.linalg.norm(
                            np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]),
                            axis=1,
                        )
                        > 1e-10
                    ).all(),
                    mesh.name,
                )
                edges = np.sort(
                    np.concatenate((f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]])), axis=1
                )
                _, counts = np.unique(edges, axis=0, return_counts=True)
                self.assertTrue((counts == 2).all(), mesh.name)
                volume = (
                    np.sum(
                        np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2]))
                    )
                    / 6
                )
                self.assertGreater(volume, 0, mesh.name)

    def test_stones_have_positive_mass_and_no_initial_neighbour_overlaps(self):
        from scipy.spatial import cKDTree

        for variant in range(6):
            shape, mass = stone_shape(variant)
            self.assertGreater(mass, 0.0)
            self.assertLess(mass, 0.2)
            self.assertLessEqual(
                np.max(np.linalg.norm(shape.vertices[:, :2], axis=1)), 0.023
            )
        # Maximum extent is 46 mm; neighbours in a layer must clear this.
        stones = self.flat[0].stones
        for z in (0.024, 0.070):
            positions = np.array(
                [s.position for s in stones if abs(s.position[2] - DECK - z) < 1e-7]
            )
            distance, _ = cKDTree(positions).query(positions, k=2)
            self.assertGreater(distance[:, 1].min(), 0.046)

    def test_railing_horizontals_have_tops_at_30_60_90(self):
        # "Spacing between tops 30 cm" on 90 cm posts, guide p. 22.
        lane = self.flat[0]
        for member, top in (("lower", 0.3), ("middle", 0.6), ("upper", 0.9)):
            mesh = next(m for m in lane.meshes if m.name == f"blue_end_south_{member}")
            self.assertAlmostEqual(mesh.vertices[:, 2].max() - DECK, top)
            self.assertAlmostEqual(np.ptp(mesh.vertices[:, 2]), 0.1)

    def test_square_lane_matches_drawing_and_guide(self):
        square = self.flat[2]
        origin = np.array(square.origin)
        self.assertEqual(square.key, "square")
        self.assertEqual(len(square.floors), 2)
        self.assertEqual(square.railing_count, 6)
        self.assertAlmostEqual(sum(np.prod(f.size) for f in square.floors), 2.4 * 2.4)
        # Four K-Rails on four backings, forming one X through the centre.
        rails = [m for m in square.meshes if "_k_" in m.name]
        self.assertEqual(len(rails), 4)
        self.assertEqual(len([m for m in square.meshes if "_backing_" in m.name]), 4)
        for rail in rails:
            self.assertTrue(
                np.any(np.linalg.norm(rail.vertices[:, :2] - origin[:2], axis=1) < 1e-8)
            )
        # Railings: full north and south edges, the north half of the east edge and the
        # south half of the west edge; the other two half-edges stay open.
        posts = [m for m in square.meshes if m.name.endswith(("post_a", "post_b"))]
        centres = np.array([m.vertices.mean(axis=0)[:2] - origin[:2] for m in posts])
        self.assertEqual(len(posts), 12)
        east = centres[np.isclose(centres[:, 0], 1.2)]
        west = centres[np.isclose(centres[:, 0], -1.2)]
        self.assertTrue(np.all(east[:, 1] > 0) and np.all(west[:, 1] < 0))
        # Two Linear Align/Inspect tasks, p. 70: green 90 cm rail with five capped hollow
        # pipes at 60 cm elevation (p. 69), projecting into the arena, targets from p. 72.
        for name, x_edge in (("east_inspect", 1.2), ("west_inspect", -1.2)):
            rail = next(m for m in square.meshes if m.name == f"{name}_rail")
            self.assertEqual(rail.material, "green")
            np.testing.assert_allclose(np.ptp(rail.vertices, axis=0), (0.05, 0.9, 0.05))
            self.assertAlmostEqual(rail.vertices[:, 2].min() - DECK, TASK_ELEVATION)
            tubes = [m for m in square.meshes if m.name.startswith(f"{name}_pipe") and m.name.endswith("_tube")]
            self.assertEqual(len(tubes), 5)
            for tube in tubes:
                z = tube.vertices[:, 2]
                self.assertAlmostEqual((z.min() + z.max()) / 2 - DECK, TASK_ELEVATION + 0.025)
                self.assertEqual(len(tube.faces), 8 * 16)
            parts = [m for m in square.meshes if m.name.startswith(name)]
            inward = np.concatenate([m.vertices[:, 0] for m in parts]) - origin[0]
            if x_edge > 0:
                self.assertLessEqual(inward.max(), x_edge + 0.025 + 1e-9)
                self.assertLess(inward.min(), x_edge - 0.15)
            else:
                self.assertGreaterEqual(inward.min(), x_edge - 0.025 - 1e-9)
                self.assertGreater(inward.max(), x_edge + 0.15)
        targets = [m for m in square.meshes if m.name.endswith("_target")]
        self.assertEqual(
            sorted(m.material for m in targets),
            sorted(f"target_{key}" for key in ACUITY_TARGETS),
        )
        for target in targets:
            self.assertEqual(target.uv.shape, (len(target.vertices), 2))
            self.assertTrue(np.all(target.uv >= 0) and np.all(target.uv <= 1))
        crate = next(m for m in square.meshes if m.name == "centre_crate")
        np.testing.assert_allclose(crate.vertices.mean(axis=0)[:2], origin[:2])
        self.assertAlmostEqual(crate.vertices[:, 2].min(), DECK + OSB)
        self.assertAlmostEqual(np.ptp(crate.vertices[:, 2]), 0.28)
        # Start on the floor pad outside the south-west corner, facing north.
        self.assertGreater(square.spawn_rotation[3], 0)
        for name, x, y in (("entry_pad", -1.85, -1.2), ("return_pad", 1.85, 0.9)):
            pad = next(m for m in square.meshes if m.name == name)
            np.testing.assert_allclose(pad.vertices.mean(axis=0)[:2] - origin[:2], (x, y))
            self.assertLess(pad.vertices[:, 2].max(), 0.02)

    def test_spawn_on_clear_external_pad_and_separate_lanes(self):
        self.assertAlmostEqual(self.flat[1].origin[0] - self.flat[0].origin[0], 7.0)
        self.assertAlmostEqual(self.flat[2].origin[0] - self.flat[1].origin[0], 7.0)
        for lane in self.flat:
            pad = next(m for m in lane.meshes if m.name == "entry_pad")
            self.assertAlmostEqual(lane.spawn[2] - pad.vertices[:, 2].max(), 0.42)
            lo, hi = pad.vertices.min(axis=0), pad.vertices.max(axis=0)
            self.assertTrue(np.all(np.array(lane.spawn[:2]) - [0.25, 0.40] > lo[:2]))
            self.assertTrue(np.all(np.array(lane.spawn[:2]) + [0.25, 0.40] < hi[:2]))

    def test_invalid_dimensions_fail_early(self):
        for value in (0.05, 0.12, 0.45, float("nan")):
            with self.assertRaises(ValueError):
                Options(k_rail_height=value)


class SelectionTests(unittest.TestCase):
    def test_cycle_wraps_and_accumulates_before_physics_tick(self):
        selection = Selection(2)
        selection.cycle(-1)
        self.assertEqual(selection.pending, 1)
        self.assertEqual(selection.current, 0)
        selection.cycle(1)
        self.assertEqual(selection.consume(), 0)
        selection.select(1)
        self.assertEqual(selection.consume(), 1)
        self.assertIsNone(selection.pending)
        self.assertEqual(selection.consume(), 1)
        with self.assertRaises(IndexError):
            selection.select(2)


if __name__ == "__main__":
    unittest.main()
