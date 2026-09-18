"""The lanes' difficulty settings (printed pp. 5, 30, 34, 37, 43): the sloped and Additional
Obstacles copies of the standard lanes, with their pinch points (pp. 86–87), layered K-Rails
(pp. 30, 32) and rotating slip disks (pp. 40–41)."""

from collections import Counter
import math
import re
import unittest

import numpy as np

from competition.geometry import (
    DECK,
    FLAT,
    OBSTACLES,
    OSB,
    PINCH_DEPTH,
    PINCH_POINTS,
    SLOPES,
    Options,
    build_lanes,
    point_polygon_distance,
)
from competition.runtime import loose_resets
from competition.terrains import BOLT_PLAY, DISK_RADIUS, DISK_THICKNESS, SEAT

STANDARD = ("krails", "stepfields", "ramps")


def to_floor(lane, floor, points):
    """World points in `floor`'s own coordinates: the inverse of Floor.transform."""
    p = np.asarray(points, dtype=float) - lane.origin
    c, s = math.cos(floor.pitch), math.sin(floor.pitch)
    x = p[:, 0] - floor.center[0]
    z = p[:, 2] - DECK - abs(s) * floor.size[0] / 2
    return np.column_stack((c * x + s * z, p[:, 1] - floor.center[1], -s * x + c * z))


class SettingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lanes = {lane.key: lane for lane in build_lanes(Options(gravel="static"))}
        cls.tilted = {lane.key: lane for lane in build_lanes(Options(gravel="static", difficulty="slopes"))}

    def named(self, key, prefix="", suffix=""):
        return [m for m in self.lanes[key].meshes if m.name.startswith(prefix) and m.name.endswith(suffix)]

    def test_each_arena_offers_its_settings_once(self):
        settings = {}
        for lane in self.lanes.values():
            settings.setdefault(lane.arena, []).append(lane.setting)
        for arena in ("Diagonal K-Rails", "Half-Cubic Stepfields", "Pitch/Roll Ramps"):
            self.assertEqual(settings[arena], [FLAT, SLOPES, OBSTACLES], arena)
        self.assertEqual(settings["Center in Alleys"], [FLAT, SLOPES])
        self.assertEqual(settings["Stairs | Pallet Climb"], ["Clear", "Debris"])
        self.assertEqual(settings["Shifty Gravel"], [FLAT])
        for arena in ("K-Rail Square", "Push/Pull Doors", "Avoid Holes/Posts", "Search & Map Maze"):
            self.assertEqual(settings[arena], [""], arena)
        self.assertEqual(self.lanes["krails_obstacles"].title, "Diagonal K-Rails · Additional Obstacles")
        self.assertEqual(self.lanes["alleys_slopes"].title, "Center in Alleys · Sloped 15°")

    def test_sloped_settings_tilt_the_centre_floors_opposite_ways(self):
        for base in STANDARD:
            for key, angle in ((base, 0.0), (f"{base}_slopes", 15.0), (f"{base}_obstacles", 15.0)):
                pitches = [math.degrees(f.pitch) for f in self.lanes[key].floors]
                np.testing.assert_allclose(pitches, [0.0, angle, -angle, 0.0], err_msg=key)
        # --difficulty tilts only the lanes without a sloped level of their own.
        for key, lanes in (("gravel", self.tilted), ("pallets", self.tilted)):
            self.assertAlmostEqual(math.degrees(lanes[key].floors[1].pitch), 15.0)
            self.assertEqual(lanes[key].setting, SLOPES)
        for key in STANDARD + ("alleys",):
            self.assertTrue(all(f.pitch == 0.0 for f in self.tilted[key].floors), key)
        far = [f.pitch for f in self.lanes["alleys_slopes"].floors if f.name.startswith("far_")]
        np.testing.assert_allclose(np.degrees(far), [15.0, 15.0])

    def test_sloped_copies_carry_the_same_terrain(self):
        # The same parts by name; tilting adds only the tilt-up legs, and the Additional
        # Obstacles only pinch points, the K-Rails' extra layers and the ramps' disks.
        extras = {
            "slopes": r"(lower|upper)_leg_(0|1|brace)",
            "obstacles": r"(lower|upper)_leg_(0|1|brace)|pinch\d_.*|.*_layer[12]|slip_(disk|seat)_\d\d|disk_washer_\d\d",
        }
        for base in STANDARD:
            flat = {m.name for m in self.lanes[base].meshes}
            for setting, pattern in extras.items():
                names = {m.name for m in self.lanes[f"{base}_{setting}"].meshes}
                self.assertLessEqual(flat, names, (base, setting))
                self.assertTrue(all(re.fullmatch(pattern, n) for n in names - flat), (base, setting, names - flat))

    def test_four_pinch_points_hang_in_each_additional_obstacles_lane(self):
        for key, lane in self.lanes.items():
            panels = [m for m in lane.meshes if re.fullmatch(r"pinch\d_panel\d", m.name)]
            self.assertEqual(len(panels), 8 if lane.setting == OBSTACLES else 0, key)
        # Serpentine: the west wall, then north, south, north.
        self.assertEqual([(floor, facing) for floor, _, facing in PINCH_POINTS],
                         [("blue_end", (1.0, 0.0)), ("upper", (0.0, -1.0)), ("lower", (0.0, 1.0)), ("green_end", (0.0, -1.0))])
        for base in STANDARD:
            lane = self.lanes[f"{base}_obstacles"]
            floors = {f.name: f for f in lane.floors}
            for k, (floor, line, facing) in enumerate(PINCH_POINTS):
                parts = {m.name[len(f"pinch{k}_"):]: m for m in lane.meshes if m.name.startswith(f"pinch{k}_")}
                self.assertEqual(sorted(parts), ["block0", "block1", "panel0", "panel1", "post0", "post1", "spine"])
                local = {name: to_floor(lane, floors[floor], m.vertices) for name, m in parts.items()}
                directions = []
                for name in ("panel0", "panel1"):
                    v = local[name]
                    self.assertEqual(parts[name].material, "red")
                    self.assertTrue(parts[name].collision and not parts[name].scan)
                    # 60 x 60 cm, hanging 35–95 cm above the floor they are hung over.
                    self.assertAlmostEqual(v[:, 2].min(), 0.35)
                    self.assertAlmostEqual(v[:, 2].max(), 0.95)
                    plan = v[:, :2]
                    centre = plan.mean(axis=0)
                    axis = np.linalg.svd(plan - centre)[2][0]
                    self.assertAlmostEqual(np.ptp((plan - centre) @ axis), 0.6, places=6)
                    directions.append(axis)
                # At right angles to each other and 45 degrees to the railing.
                self.assertAlmostEqual(abs(directions[0] @ directions[1]), 0.0)
                self.assertAlmostEqual(abs(directions[0] @ np.r_[facing]), math.sqrt(0.5))
                # The apex juts 42 cm into the lane from 5 mm clear of the railing's inner face,
                # which is 2.5 cm in from the railing line; its outer corner half a board more.
                panel_plan = np.vstack([local["panel0"], local["panel1"]])[:, :2]
                inner = np.dot(line, facing) + 0.025
                self.assertAlmostEqual(np.min(panel_plan @ np.r_[facing]), inner + 0.005 - OSB / 2 / math.sqrt(2))
                self.assertAlmostEqual(np.max(panel_plan @ np.r_[facing]), inner + 0.005 + PINCH_DEPTH + OSB / 2 / math.sqrt(2))
                # The hanging blocks lie on the top rail, whose top is 90 cm up.
                for name in ("block0", "block1"):
                    self.assertAlmostEqual(local[name][:, 2].min(), 0.90)

    def test_pinch_points_hang_clear_of_the_terrain_under_them(self):
        """Nothing walkable (or loose) under a pinch point reaches its panels' lower edge, even
        with the K-Rails layered to their 40 cm maximum: the diagonals meet the walls at the
        cells' corners, and the pinch points hang at the middle of a cell's wall."""
        from scipy.spatial import ConvexHull

        def plan(points):
            return points[ConvexHull(points).vertices]  # counter-clockwise, as the distance needs

        for options in (Options(gravel="static"), Options(gravel="static", layered_k_rail_height=0.40)):
            for lane in build_lanes(options):
                if lane.setting != OBSTACLES:
                    continue
                floors = {f.name: f for f in lane.floors}
                for k, (name, _, _) in enumerate(PINCH_POINTS):
                    floor = floors[name]
                    panels = to_floor(lane, floor, np.vstack(
                        [m.vertices for m in lane.meshes if re.fullmatch(fr"pinch{k}_(panel\d|spine)", m.name)]))
                    # The triangle the panels make with the railing, sampled.
                    triangle = plan(panels[:, :2])
                    lo, hi = triangle.min(axis=0), triangle.max(axis=0)
                    grid = np.stack(np.meshgrid(np.linspace(lo[0], hi[0], 30), np.linspace(lo[1], hi[1], 30)), -1)
                    under = [p for p in grid.reshape(-1, 2) if point_polygon_distance(p, triangle) == 0.0]
                    self.assertGreater(len(under), 200)
                    for m in lane.meshes:
                        if not (m.scan or m.dynamic):
                            continue
                        v = to_floor(lane, floor, m.vertices)
                        if v[:, 2].max() < panels[:, 2].min():
                            continue
                        footprint = plan(v[:, :2])
                        clash = [p for p in under if point_polygon_distance(p, footprint) == 0.0]
                        self.assertFalse(clash, (lane.key, k, m.name))

    def test_layered_k_rails_stack_in_five_centimetre_layers(self):
        for key, height in (("krails", 0.10), ("krails_slopes", 0.10), ("krails_obstacles", 0.20)):
            lane = self.lanes[key]
            end = next(f for f in lane.floors if f.name == "blue_end")
            layers = [m for m in lane.meshes if m.name.startswith("blue_end_k_0_layer")]
            self.assertEqual(len(layers), 1 + round((height - 0.10) / 0.05), key)
            top = max(m.vertices[:, 2].max() for m in layers)
            self.assertAlmostEqual(top - DECK - OSB, height, msg=key)
            self.assertAlmostEqual(np.ptp(layers[0].vertices[:, 2]), 0.10)
            self.assertTrue(all(math.isclose(np.ptp(m.vertices[:, 2]), 0.05) for m in layers[1:]))
            self.assertEqual(end.pitch, 0.0)
        tall = {lane.key: lane for lane in build_lanes(Options(layered_k_rail_height=0.40, gravel="static"))}
        self.assertEqual(len([m for m in tall["krails_obstacles"].meshes if "_k_" in m.name]), 8 * 7)
        self.assertEqual(len([m for m in tall["krails"].meshes if "_k_" in m.name]), 8)
        self.assertIn("40 cm layered K-Rails", tall["krails_obstacles"].subtitle)
        for value in (0.05, 0.12, 0.45):
            with self.assertRaises(ValueError):
                Options(layered_k_rail_height=value)

    def test_a_slip_disk_turns_on_a_loose_bolt_at_the_centre_of_every_ramp(self):
        for key in ("ramps", "ramps_slopes"):
            self.assertFalse(self.lanes[key].joints or self.named(key, "slip_"), key)
        lane = self.lanes["ramps_obstacles"]
        disks = self.named("ramps_obstacles", "slip_disk_")
        seats = {m.name[-2:]: m for m in self.named("ramps_obstacles", "slip_seat_")}
        washers = self.named("ramps_obstacles", "disk_washer_")
        ramps = [m for m in lane.meshes if re.fullmatch(r".*_ramps\d\d_r\d", m.name)]
        self.assertEqual((len(disks), len(seats), len(washers), len(ramps), len(lane.joints)), (32, 32, 32, 32, 32))
        self.assertIn("32 rotating slip disks", lane.subtitle)
        joints = {j.body1: j for j in lane.joints}
        used = Counter()
        for disk in disks:
            self.assertTrue(disk.dynamic and disk.collision and not disk.scan)
            self.assertAlmostEqual(disk.mass, 650 * math.pi * 0.25**2 * 0.003)
            self.assertEqual((disk.material, disk.contact_offset), ("slip_disk", 0.002))
            self.assertEqual(disk.uv.shape, (len(disk.vertices), 2))
            centre = disk.vertices.mean(axis=0)
            joint = joints[disk.name]
            np.testing.assert_allclose(joint.pivot, centre, atol=1e-9)
            self.assertEqual((joint.body0, joint.limits, joint.lift), ("", None, BOLT_PLAY))
            axis = np.asarray(joint.axis) / np.linalg.norm(joint.axis)
            # 50 cm across and 3 mm thick along its bolt.
            radial = disk.vertices - centre
            along = radial @ axis
            np.testing.assert_allclose(np.abs(along), DISK_THICKNESS / 2, atol=1e-9)
            np.testing.assert_allclose(np.linalg.norm(radial - np.outer(along, axis), axis=1), DISK_RADIUS, atol=1e-9)
            # On the ramp under it: parallel to its top, a seat's thickness above it, well inside it.
            ramp = next(r for r in ramps if np.all(r.vertices[:, :2].min(axis=0) < centre[:2])
                        and np.all(r.vertices[:, :2].max(axis=0) > centre[:2]))
            used[ramp.name] += 1
            top = ramp.vertices[4:8]
            normal = np.cross(top[1] - top[0], top[3] - top[0])
            normal /= np.linalg.norm(normal)
            self.assertAlmostEqual(abs(normal @ axis), 1.0)
            bottom = disk.vertices[along < 0]
            np.testing.assert_allclose((bottom - top[0]) @ normal, SEAT, atol=1e-9)
            np.testing.assert_allclose(top.mean(axis=0) + (SEAT + DISK_THICKNESS / 2) * normal, centre, atol=1e-9)
            seat = seats[disk.name[-2:]]
            self.assertEqual(seat.material, "slip_seat")
            self.assertTrue(seat.collision and not seat.dynamic)
            np.testing.assert_allclose(seat.vertices.mean(axis=0), centre - (SEAT + DISK_THICKNESS) / 2 * normal, atol=1e-9)
        self.assertEqual(set(used.values()), {1})
        self.assertTrue(all(not w.collision for w in washers))

    def test_the_disks_come_back_on_a_reset_and_nothing_else_does(self):
        resets = {reset.name: reset for reset in loose_resets(list(self.lanes.values()), Options(gravel="static"))}
        self.assertEqual(sorted(resets), ["competition_avoid_posts", "competition_ramps_obstacles_disks"])
        disks = resets["competition_ramps_obstacles_disks"]
        lane = self.lanes["ramps_obstacles"]
        prims = [m.name for m in lane.meshes] + [j.name for j in lane.joints]
        pattern = disks.pattern.rsplit("/", 1)[1]
        matched = sorted(n for n in prims if re.fullmatch(pattern, n))
        self.assertEqual(matched, [f"slip_disk_{k:02d}" for k in range(32)])
        by_name = {m.name: m for m in lane.meshes}
        for k, (position, rotation) in enumerate(disks.poses):
            np.testing.assert_allclose(position, by_name[f"slip_disk_{k:02d}"].vertices.mean(axis=0))
            self.assertEqual(rotation, (1.0, 0.0, 0.0, 0.0))



if __name__ == "__main__":
    unittest.main()
