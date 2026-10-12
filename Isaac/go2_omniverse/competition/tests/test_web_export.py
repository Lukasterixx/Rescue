"""The arenas as VIP-Rescue's website gets them: glTF models, terrain, the K-Rail Square's course route."""

import base64
import json
from pathlib import Path
import struct
import tempfile
import unittest

import numpy as np

from competition import web_export


def levels_at(terrain, x, y):
    """The terrain's walkable levels at (x, y), as the website reads them."""
    counts = np.frombuffer(base64.b64decode(terrain["levels"]["counts"]), dtype="u1").astype(int)
    z = np.frombuffer(base64.b64decode(terrain["levels"]["z_cm"]), dtype="<i2") / 100.0
    i = int(np.floor((x - terrain["x0"]) / terrain["resolution"]))
    j = int(np.floor((y - terrain["y0"]) / terrain["resolution"]))
    if not (0 <= i < terrain["width"] and 0 <= j < terrain["height"]):
        return [terrain["outside_level"]]
    k = j * terrain["width"] + i
    start = int(counts[:k].sum())
    return [round(float(v), 2) for v in z[start:start + counts[k]]]


class WebExportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.out = Path(cls.tmp.name)
        cls.index = web_export.export(cls.out)
        cls.arenas = {a["id"]: a for a in cls.index["arenas"]}

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_every_lane_has_a_model_walls_and_a_start(self):
        self.assertIn("square", self.arenas)
        self.assertEqual(len(self.arenas), len(self.index["arenas"]))
        for arena in self.index["arenas"]:
            self.assertTrue((self.out / arena["model"]).is_file(), arena["id"])
            self.assertTrue((self.out / arena["terrain"]).is_file(), arena["id"])
            x0, y0, x1, y1 = arena["bounds"]
            self.assertTrue(x0 - 0.5 <= arena["start"]["x"] <= x1 + 0.5, arena["id"])
            self.assertTrue(y0 - 0.5 <= arena["start"]["y"] <= y1 + 0.5, arena["id"])

    def test_the_models_are_valid_gltf_binaries_standing_up(self):
        data = (self.out / "arenas" / "square.glb").read_bytes()
        magic, version, length = struct.unpack("<III", data[:12])
        self.assertEqual((magic, version, length), (0x46546C67, 2, len(data)))
        json_length, json_type = struct.unpack("<II", data[12:20])
        self.assertEqual(json_type, 0x4E4F534A)
        document = json.loads(data[20:20 + json_length])
        positions = [document["accessors"][p["attributes"]["POSITION"]] for p in document["meshes"][0]["primitives"]]
        low = np.min([a["min"] for a in positions], axis=0)
        high = np.max([a["max"] for a in positions], axis=0)
        # +y up: the square stands on the floor and its railings reach 0.9 m above its 12 cm deck.
        self.assertAlmostEqual(low[1], 0.0, places=3)
        self.assertGreater(high[1], 0.95)
        # glTF z is the arena's -y: the square's END pad reaches y = 1.5, its START pad y = -1.8.
        self.assertAlmostEqual(low[2], -1.5, places=3)
        self.assertAlmostEqual(high[2], 1.8, places=3)
        self.assertNotIn("tarp", {m["name"] for m in json.loads(
            (self.out / "arenas" / "maze.glb").read_bytes()[20:20 + struct.unpack("<I", (
                self.out / "arenas" / "maze.glb").read_bytes()[12:16])[0]])["materials"]})

    def test_the_terrain_is_where_the_robot_can_stand(self):
        square = json.loads((self.out / "arenas" / "square.terrain.json").read_text())
        self.assertEqual(square["schema"], "vip-rescue.terrain/1")
        self.assertEqual(levels_at(square, -1.85, -1.2), [0.02])          # the START pad, 1.5 cm
        self.assertEqual(levels_at(square, -0.6, 0.0), [0.12])            # the deck
        self.assertIn(0.22, levels_at(square, -0.6, 0.6))                 # on a K-rail (the NW one's middle)
        self.assertEqual(levels_at(square, 0.0, 0.0), [])                 # the crate: nowhere to stand
        self.assertEqual(levels_at(square, 0.0, 1.2), [])                 # the north railing
        self.assertEqual(levels_at(square, -1.2, 0.6), [0.12])            # the open half of the west edge
        self.assertEqual(levels_at(square, 3.0, -1.6), [0.0])             # the hall floor
        self.assertEqual(levels_at(square, 30.0, 30.0), [0.0])            # off the grid: the hall floor
        # The stairs' landing: on it, or crouched on the floor under it.
        stairs = json.loads((self.out / "arenas" / "stairs.terrain.json").read_text())
        self.assertEqual(levels_at(stairs, -0.6, 0.6), [0.0, 1.0])

    def test_the_route_aligns_to_what_the_robot_sees_from_its_start(self):
        route = json.loads((self.out / "routes" / "krail_square.json").read_text())
        cells = np.array(route["walls"]["cells"])
        self.assertEqual(route["walls"]["seen_from"], "start")
        self.assertGreater(len(cells), 300)
        # Every cell lies on one of the four railing lines (within a railing's half-thickness and a cell)...
        on_line = (
            ((np.abs(np.abs(cells[:, 1]) - 1.2) < 0.06) & (np.abs(cells[:, 0]) < 1.26))
            | ((np.abs(cells[:, 0] - 1.2) < 0.06) & (cells[:, 1] > -0.06))
            | ((np.abs(cells[:, 0] + 1.2) < 0.06) & (cells[:, 1] < 0.06))
        )
        # ... or on the linear inspect tasks: 90 cm rails on the east and west railings, centred 0.6 m along them,
        # their pipes reaching in from the railing.
        tasks = (np.abs(np.abs(cells[:, 0]) - 1.05) < 0.2) & (np.abs(np.abs(cells[:, 1]) - 0.6) < 0.5)
        self.assertTrue(np.all(on_line | tasks), cells[~(on_line | tasks)][:5])
        # The crate (0.4 m tall) and the deck are below the band; the north railing is all there.
        self.assertFalse(np.any((np.abs(cells[:, 0]) < 0.3) & (np.abs(cells[:, 1]) < 0.3)))
        north = cells[np.abs(cells[:, 1] - 1.2) < 0.06]
        self.assertGreaterEqual(len(np.unique(np.round(north[:, 0], 3))), 48)

    def test_the_squares_course_route_is_measure_squares(self):
        route = json.loads((self.out / "routes" / "krail_square.json").read_text())
        self.assertEqual(route["schema"], "vip-rescue.route/1")
        self.assertEqual(route["reference"], {"kind": "arena", "id": "square", "title": "K-Rail Square"})
        self.assertEqual(route["start"], {"x": -1.85, "y": -1.2, "yaw_deg": 90.0})
        self.assertEqual(route["waypoints"][0], [-1.85, 0.6])
        self.assertEqual(route["waypoints"][-1], [1.9, 0.9])
        self.assertEqual(len(route["waypoints"]), 8)
        self.assertEqual(route["walls"]["resolution"], 0.05)


if __name__ == "__main__":
    unittest.main()
