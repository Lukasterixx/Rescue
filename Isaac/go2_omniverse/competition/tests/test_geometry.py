"""Dimension/topology checks against the fabrication drawings (no Isaac needed)."""

import math
import unittest

import numpy as np

from competition.geometry import (
    ACUITY_TARGETS,
    DECK,
    LANE_ORIGINS,
    OSB,
    PALLET_TOP,
    TASK_ELEVATION,
    Options,
    build_lanes,
    stone_shape,
)


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


class RemainingLaneTests(unittest.TestCase):
    """The eight lanes added after gravel, K-Rails and the square."""

    @classmethod
    def setUpClass(cls):
        cls.lanes = {lane.key: lane for lane in build_lanes()}
        cls.slopes = {lane.key: lane for lane in build_lanes(Options(difficulty="slopes"))}

    def named(self, key, prefix="", suffix=""):
        return [m for m in self.lanes[key].meshes if m.name.startswith(prefix) and m.name.endswith(suffix)]

    def test_catalogue_order_origins_and_pads(self):
        self.assertEqual(list(self.lanes), list(LANE_ORIGINS))
        xs = [lane.origin[0] for lane in self.lanes.values()]
        self.assertEqual(xs, sorted(xs))
        for key, lane in self.lanes.items():
            pad = next(m for m in lane.meshes if m.name == "entry_pad")
            self.assertAlmostEqual(lane.spawn[2] - pad.vertices[:, 2].max(), 0.42, msg=key)
            lo, hi = pad.vertices.min(axis=0), pad.vertices.max(axis=0)
            self.assertTrue(np.all(np.array(lane.spawn[:2]) - 0.25 > lo[:2]), key)
            self.assertTrue(np.all(np.array(lane.spawn[:2]) + 0.25 < hi[:2]), key)
        # Lanes never overlap along the hall.
        spans = sorted((np.min([m.vertices[:, 0].min() for m in lane.meshes]), np.max([m.vertices[:, 0].max() for m in lane.meshes])) for lane in self.lanes.values())
        for (_, right), (left, _) in zip(spans, spans[1:]):
            self.assertLess(right, left)

    def test_stepfields_three_elevations(self):
        tops = self.named("stepfields", suffix="_top")
        self.assertEqual(len(self.named("stepfields", suffix="_base")), 32)
        heights = sorted({round(float(m.vertices[:, 2].max() - DECK - OSB), 3) for m in tops})
        self.assertEqual(heights, [0.15, 0.30])
        self.assertEqual(len(tops), 16 * 3 + 16)  # quads carry three plateaus, singles one
        self.assertTrue(all(np.allclose(np.ptp(m.vertices[:, :2], axis=0), 0.297) for m in tops))

    def test_ramps_are_fifteen_degree_wedges_in_peaks_and_valleys(self):
        ramps = self.named("ramps", suffix="_r0") + self.named("ramps", suffix="_r1") + self.named("ramps", suffix="_r2") + self.named("ramps", suffix="_r3")
        self.assertEqual(len(ramps), 32)
        for ramp in ramps:
            z = ramp.vertices[:, 2] - DECK
            self.assertAlmostEqual(z.max(), OSB + 0.15)
            self.assertAlmostEqual(z.min(), OSB)  # on the half-panel backing
            self.assertAlmostEqual(math.degrees(math.atan((0.15 - OSB) / 0.594)), 13.2, places=0)
        # A peak's four high edges lie on the element's centre lines; the first element of
        # the first floor is a peak, and the lower floor's first is a valley.
        meshes = self.lanes["ramps"].meshes
        for name, peak in (("blue_end_ramps00", True), ("lower_ramps00", False)):
            backing = next(m for m in meshes if m.name == f"{name}_backing")
            centre = backing.vertices[:, :2].mean(axis=0)
            element = [m for m in ramps if m.name.startswith(name)]
            high = np.concatenate([m.vertices[m.vertices[:, 2] > DECK + 0.1][:, :2] for m in element])
            on_centre_lines = np.abs(high - centre).min(axis=1).max() < 0.01
            self.assertEqual(on_centre_lines, peak, name)

    def test_alleys_three_slalom_doorways(self):
        lane = self.lanes["alleys"]
        # Twelve perimeter railings and three dividers; no standard end barriers (p. 44).
        self.assertEqual(lane.railing_count, 15)
        self.assertFalse([m for m in lane.meshes if "end_barrier" in m.name])
        panels = self.named("alleys", suffix="_panel")
        self.assertEqual(len(panels), 3)
        for k, panel in enumerate(panels):
            y = panel.vertices[:, 1]
            gap = y.min() + 1.2 if k % 2 == 0 else 1.2 - y.max()  # south, north, south
            self.assertAlmostEqual(gap, 0.45)
            self.assertAlmostEqual(np.ptp(panel.vertices[:, 2]), 0.8)
        wide = build_lanes(Options(alley_width=0.8))[5]
        panel = next(m for m in wide.meshes if m.name == "divider0_panel")
        self.assertAlmostEqual(panel.vertices[:, 1].min() + 1.2, 0.8)
        self.assertEqual(self.slopes["alleys"].railing_count, 15)
        # Under slopes the far floors rise together toward the far end (a side slope for
        # the hallways); the door-end floors stay flat.
        floors = {f.name: f for f in self.slopes["alleys"].floors}
        self.assertEqual(floors["far_north"].pitch, floors["far_south"].pitch)
        self.assertGreater(floors["far_north"].pitch, 0)
        self.assertEqual(floors["blue_end"].pitch, floors["second"].pitch)
        self.assertEqual(floors["second"].pitch, 0.0)
        # ...hinged to the flat floor at x = 0, with no slot between them.
        for name in ("far_north", "far_south"):
            low_edge = floors[name].transform([[-1.2, 0.0, 0.0]])[0]
            self.assertAlmostEqual(low_edge[0], 0.0)
            self.assertAlmostEqual(low_edge[2], DECK)

    def test_alleys_every_divider_line_has_its_doorway(self):
        # Regression: the first divider was sealed, its doorway landing on the standard
        # lane's end barrier. Walk along each divider line and find where it is open.
        for lanes in (self.lanes, self.slopes):
            lane = lanes["alleys"]
            ox = lane.origin[0]
            floor_parts = ("_osb", "_frame_long_", "_joist_", "_leg_", "_leg_brace")
            solid = [
                m
                for m in lane.meshes
                if m.collision and not any(part in m.name for part in floor_parts)
            ]
            ys = np.linspace(-1.15, 1.15, 231)
            sides = []
            for x in (-1.2, 0.0, 1.2):
                blocked = np.zeros(len(ys), bool)
                for m in solid:
                    lo, hi = m.vertices.min(axis=0), m.vertices.max(axis=0)
                    if lo[0] - ox <= x + 0.05 and hi[0] - ox >= x - 0.05:
                        blocked |= (ys >= lo[1]) & (ys <= hi[1])
                free = ys[~blocked]
                self.assertTrue(len(free), f"divider at x = {x} is closed")
                self.assertTrue(np.all(np.diff(free) < 0.011), f"x = {x}: gap not contiguous")
                self.assertGreater(np.ptp(free) + 0.05, 0.40, f"x = {x}: doorway too narrow")
                sides.append("north" if free.mean() > 0 else "south")
            self.assertEqual(sides, ["south", "north", "south"])

    def test_pallets_two_raised_cells_with_pipes(self):
        osb = self.named("pallets", suffix="_osb")
        self.assertEqual(len([m for m in osb if "pallet" in m.name]), 10)
        upper = [m for m in osb if m.name.endswith("_upper_osb")]
        self.assertEqual(len(upper), 2)
        for m in upper:
            self.assertAlmostEqual(m.vertices[:, 2].min() - DECK, PALLET_TOP)
        pipes = self.named("pallets", prefix="pipe_", suffix="_pipe0")
        self.assertEqual(len(pipes), 4)
        for pipe in pipes:
            self.assertAlmostEqual(pipe.vertices[:, 2].min() - DECK, PALLET_TOP, places=3)

    def test_door_leaf_is_sprung_and_floor_options_remove_steps(self):
        lane = self.lanes["doors"]
        leaf = next(m for m in lane.meshes if m.name == "door_leaf")
        self.assertTrue(leaf.dynamic)
        self.assertEqual(len(lane.joints), 1)
        hinge = lane.joints[0]
        self.assertEqual((hinge.body0, hinge.body1), ("stud2", "door_leaf"))
        self.assertAlmostEqual(hinge.pivot[1] - lane.origin[1], 1.05)
        self.assertGreater(hinge.stiffness, 0)
        self.assertAlmostEqual(np.ptp(leaf.vertices[:, 2]), 2.0)
        names = {m.name for m in lane.meshes}
        self.assertTrue({"square_step_west", "half_step_east", "base"} <= names)
        for choice, missing in (("square", "square_step_west"), ("half", "half_step_east")):
            lane = build_lanes(Options(door_floor=choice))[7]
            self.assertNotIn(missing, {m.name for m in lane.meshes})
            self.assertIn("base", {m.name for m in lane.meshes})

    def test_avoid_posts_are_loose_pairs_on_a_meander(self):
        lane = self.lanes["avoid"]
        posts = lane.dynamic_meshes()
        self.assertEqual(len(posts), 10)
        self.assertTrue(all(m.name.startswith("post_") and m.mass > 0 for m in posts))
        pairs = [posts[i : i + 2] for i in range(0, 10, 2)]
        for a, b in pairs:
            gap = np.linalg.norm(a.vertices.mean(axis=0)[:2] - b.vertices.mean(axis=0)[:2])
            self.assertAlmostEqual(gap, 0.90)
            self.assertAlmostEqual(a.vertices[:, 2].min(), 0.14)
        self.assertEqual(len(self.named("avoid", suffix="_stringer0")), 10)

    def test_stairs_landing_and_pallet_climb(self):
        lane = self.lanes["stairs"]
        for k in range(1, 6):
            tread = next(m for m in lane.meshes if m.name == f"tread{k}")
            self.assertAlmostEqual(tread.vertices[:, 2].max(), 0.2 * k)
            self.assertAlmostEqual(np.ptp(tread.vertices[:, 0]), 0.9)
        landing = next(m for m in lane.meshes if m.name == "landing_osb")
        self.assertAlmostEqual(landing.vertices[:, 2].max(), 1.0)
        tops = [max(m.vertices[:, 2].max() for m in lane.meshes if m.name.startswith(f"stack{i}_pallet")) for i in range(3)]
        np.testing.assert_allclose(tops, [7 * PALLET_TOP, 4 * PALLET_TOP, PALLET_TOP])
        self.assertEqual(len(self.named("stairs", prefix="stack", suffix="_pipes_pipe2")), 3)
        shallow = build_lanes(Options(stair_angle=35.0, stair_debris=3))[9]
        run = 0.2 / math.tan(math.radians(35.0))
        t1 = next(m for m in shallow.meshes if m.name == "tread1")
        t5 = next(m for m in shallow.meshes if m.name == "tread5")
        self.assertAlmostEqual(t5.vertices[:, 1].min() - t1.vertices[:, 1].min(), 4 * run)
        self.assertEqual(len([m for m in shallow.meshes if m.name.startswith("debris")]), 3)

    def test_maze_walls_fiducials_diagonals_rooms_and_tarp(self):
        lane = self.lanes["maze"]
        walls = self.named("maze", prefix="wall_")
        self.assertEqual(len(walls), 36)
        for wall in walls:
            self.assertAlmostEqual(np.ptp(wall.vertices[:, 2]), 2.2)
            self.assertAlmostEqual(min(np.ptp(wall.vertices[:, :2], axis=0)), 0.01)
        fiducials = self.named("maze", prefix="fiducial_")
        heights = sorted(round(float(m.vertices[:, 2].mean()), 2) for m in fiducials)
        self.assertEqual(heights, [1.0] * 5 + [2.0] * 5)
        self.assertEqual(len(self.named("maze", prefix="diagonal_", suffix="_0")), 19)
        self.assertEqual(len(self.named("maze", prefix="room", suffix="_target")), 10)
        tarp = next(m for m in lane.meshes if m.name == "tarp")
        self.assertFalse(tarp.collision)
        self.assertGreater(tarp.vertices[:, 2].min(), 2.2 - 1e-9)
        # Everything but the tarp stands on the hall floor.
        self.assertAlmostEqual(min(m.vertices[:, 2].min() for m in walls), 0.0)


if __name__ == "__main__":
    unittest.main()
