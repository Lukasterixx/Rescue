"""The arenas as VIP-Rescue's website gets them: glTF models, wall cells, the K-Rail Square's course route."""

import json
from pathlib import Path
import struct
import tempfile
import unittest

import numpy as np

from competition import web_export


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
            self.assertTrue((self.out / arena["walls"]).is_file(), arena["id"])
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

    def test_the_squares_walls_are_its_railings_and_nothing_low(self):
        walls = json.loads((self.out / "arenas" / "square.walls.json").read_text())
        cells = np.array(walls["cells"])
        self.assertEqual(walls["resolution"], 0.05)
        # Every cell lies on one of the four railing lines (within a railing's half-thickness and a cell).
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
        self.assertEqual(len(route["walls"]["cells"]), self.arenas["square"]["wall_cells"])


if __name__ == "__main__":
    unittest.main()
