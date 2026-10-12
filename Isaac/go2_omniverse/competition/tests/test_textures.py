"""The textures a sim writes at startup, which another sim may be rendering from."""

import tempfile
from pathlib import Path
import unittest

from competition.textures import write_textures


class TextureTests(unittest.TestCase):
    def test_a_second_writer_leaves_the_files_alone(self):
        with tempfile.TemporaryDirectory() as directory:
            textures = Path(directory) / "textures"
            write_textures(textures)
            written = {path.name: (path.read_bytes(), path.stat().st_mtime_ns) for path in textures.iterdir()}
            self.assertIn("osb.png", written)
            self.assertIn("slip_disk.png", written)
            write_textures(textures)
            again = {path.name: (path.read_bytes(), path.stat().st_mtime_ns) for path in textures.iterdir()}
            self.assertEqual(again, written)

    def test_a_changed_file_is_replaced_whole(self):
        with tempfile.TemporaryDirectory() as directory:
            textures = Path(directory) / "textures"
            write_textures(textures)
            osb = textures / "osb.png"
            good = osb.read_bytes()
            osb.write_bytes(good[: len(good) // 2])
            write_textures(textures)
            self.assertEqual(osb.read_bytes(), good)
            self.assertEqual(sorted(p.name for p in textures.iterdir() if p.name.startswith(".")), [])


if __name__ == "__main__":
    unittest.main()
