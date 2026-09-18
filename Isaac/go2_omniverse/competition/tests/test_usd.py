"""Exercise real USD export and reference composition, without an Isaac GPU."""

import tempfile
from pathlib import Path
import unittest

try:
    from pxr import Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None

from competition.build import export_scene
from competition.geometry import Options


@unittest.skipIf(Usd is None, "USD bindings unavailable in this interpreter")
class UsdExportTests(unittest.TestCase):
    def test_static_and_dynamic_scenes_reference_correctly(self):
        for mode in ("static", "dynamic"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / "lane.usdc"
                lanes = export_scene(output, Options(gravel=mode))
                stage = Usd.Stage.CreateInMemory()
                root = stage.DefinePrim("/World/competition/terrain")
                root.GetReferences().AddReference(str(output))
                scan = stage.GetPrimAtPath(str(root.GetPath()) + "/WalkableScan")
                self.assertTrue(scan.IsA(UsdGeom.Mesh))
                self.assertFalse(scan.HasAPI(UsdPhysics.CollisionAPI))
                self.assertEqual(
                    UsdGeom.Imageable(scan).GetVisibilityAttr().Get(), "invisible"
                )
                gravel = stage.GetPrimAtPath(str(root.GetPath()) + "/Gravel")
                rocks = list(gravel.GetChildren())
                self.assertEqual(len(rocks), len(lanes[0].stones))
                for rock in (rocks[0], rocks[-1]):
                    self.assertTrue(rock.HasAPI(UsdPhysics.CollisionAPI))
                    self.assertEqual(
                        rock.HasAPI(UsdPhysics.RigidBodyAPI), mode == "dynamic"
                    )
                    self.assertEqual(
                        UsdPhysics.MeshCollisionAPI(rock).GetApproximationAttr().Get(),
                        "convexHull",
                    )
                    self.assertTrue(UsdGeom.Mesh(rock).GetPointsAttr().Get())
                # Abstract prototypes must never become phantom colliders.
                self.assertTrue(
                    stage.GetPrimAtPath(
                        str(root.GetPath()) + "/StoneTemplates/Stone0"
                    ).IsAbstract()
                )
                self.assertTrue((output.parent / "textures" / "osb.png").is_file())
                self.assertTrue((output.parent / "textures" / "target_2C.png").is_file())
                self.assertTrue(output.with_suffix(".json").is_file())
                # The square's inspect targets carry their own texture coordinates.
                target = stage.GetPrimAtPath(
                    str(root.GetPath()) + "/Structure/square/east_inspect_pipe0_target"
                )
                self.assertTrue(target.IsA(UsdGeom.Mesh))
                st = UsdGeom.PrimvarsAPI(target).GetPrimvar("st")
                self.assertTrue(st.IsDefined())
                faces = UsdGeom.Mesh(target).GetFaceVertexCountsAttr().Get()
                self.assertEqual(len(st.Get()), 3 * len(faces))


if __name__ == "__main__":
    unittest.main()
