"""Exercise real USD export and reference composition, without an Isaac GPU."""

import tempfile
from pathlib import Path
import unittest

try:
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
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
                # The door leaf is a hull-collided rigid body on a sprung revolute joint.
                leaf = stage.GetPrimAtPath(str(root.GetPath()) + "/Structure/doors/door_leaf")
                self.assertTrue(leaf.HasAPI(UsdPhysics.RigidBodyAPI))
                self.assertEqual(UsdPhysics.MeshCollisionAPI(leaf).GetApproximationAttr().Get(), "convexHull")
                hinge = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(str(root.GetPath()) + "/Structure/doors/door_hinge"))
                self.assertTrue(hinge)
                self.assertEqual([str(p) for p in hinge.GetBody1Rel().GetTargets()], [str(leaf.GetPath())])
                self.assertEqual(hinge.GetUpperLimitAttr().Get(), 100.0)
                self.assertGreater(UsdPhysics.DriveAPI(hinge.GetPrim(), "angular").GetStiffnessAttr().Get(), 0)
                # Metal faces remain on the same moving mesh as the leaf, so the
                # new round handles retain their own finish throughout the swing.
                handle_faces = UsdGeom.Subset(stage.GetPrimAtPath(str(leaf.GetPath()) + "/handle_metal"))
                self.assertTrue(handle_faces)
                count = len(UsdGeom.Mesh(leaf).GetFaceVertexCountsAttr().Get())
                self.assertEqual(list(handle_faces.GetIndicesAttr().Get()), list(range(12, count)))
                finish, _ = UsdShade.MaterialBindingAPI(handle_faces.GetPrim()).ComputeBoundMaterial()
                self.assertEqual(str(finish.GetPath()), str(root.GetPath()) + "/Materials/handle_metal")
                shader = UsdShade.Shader(stage.GetPrimAtPath(str(finish.GetPath()) + "/Surface"))
                self.assertAlmostEqual(shader.GetInput("metallic").Get(), 0.8)
                self.assertLess(shader.GetInput("roughness").Get(), 0.4)
                # Avoid posts are loose bodies with a translate op the reset can drive.
                post = stage.GetPrimAtPath(str(root.GetPath()) + "/Structure/avoid/post_00")
                self.assertTrue(post.HasAPI(UsdPhysics.RigidBodyAPI))
                self.assertTrue(UsdGeom.Xformable(post).GetOrderedXformOps())
                # Static structure keeps exact triangle collision and no rigid body.
                wall = stage.GetPrimAtPath(str(root.GetPath()) + "/Structure/maze/wall_00")
                self.assertFalse(wall.HasAPI(UsdPhysics.RigidBodyAPI))
                self.assertEqual(UsdPhysics.MeshCollisionAPI(wall).GetApproximationAttr().Get(), "none")
                self.assertFalse(stage.GetPrimAtPath(str(root.GetPath()) + "/Structure/maze/tarp").HasAPI(UsdPhysics.CollisionAPI))
                # Both stair variants coexist; the debris and braces are real colliders,
                # while the unused hanging belay ropes stay visual-only.
                for key in ("stairs", "stair_debris"):
                    stairs = str(root.GetPath()) + f"/Structure/{key}"
                    self.assertTrue(stage.GetPrimAtPath(stairs + "/stair_belay_top").HasAPI(UsdPhysics.CollisionAPI))
                    self.assertTrue(stage.GetPrimAtPath(stairs + "/pallet_belay_post0").HasAPI(UsdPhysics.CollisionAPI))
                    self.assertFalse(stage.GetPrimAtPath(stairs + "/stair_belay_rope").HasAPI(UsdPhysics.CollisionAPI))
                    for name in ("debris0", "debris1", "debris2", "landing_brace_over", "landing_brace_under"):
                        prim = stage.GetPrimAtPath(stairs + "/" + name)
                        if key == "stair_debris":
                            self.assertTrue(prim.HasAPI(UsdPhysics.CollisionAPI))
                            self.assertFalse(prim.HasAPI(UsdPhysics.RigidBodyAPI))
                        else:
                            self.assertFalse(prim)
                # A slip disk is a thin, textured rigid body on a loose bolt: a D6 joint that
                # turns about the ramp's normal and slides a little along it, nothing else.
                ramps = str(root.GetPath()) + "/Structure/ramps_obstacles"
                disk = stage.GetPrimAtPath(ramps + "/slip_disk_00")
                self.assertTrue(disk.HasAPI(UsdPhysics.RigidBodyAPI))
                self.assertEqual(UsdPhysics.MeshCollisionAPI(disk).GetApproximationAttr().Get(), "convexHull")
                self.assertAlmostEqual(disk.GetAttribute("physxCollision:contactOffset").Get(), 0.002)
                self.assertIn("PhysxCollisionAPI", disk.GetMetadata("apiSchemas").GetAddedOrExplicitItems())
                self.assertTrue(UsdGeom.PrimvarsAPI(disk).GetPrimvar("st").IsDefined())
                self.assertTrue((output.parent / "textures" / "slip_disk.png").is_file())
                bolt = stage.GetPrimAtPath(ramps + "/disk_bolt_00")
                self.assertEqual(bolt.GetTypeName(), "PhysicsJoint")
                self.assertFalse(UsdPhysics.Joint(bolt).GetBody0Rel().GetTargets())
                self.assertEqual([str(p) for p in UsdPhysics.Joint(bolt).GetBody1Rel().GetTargets()], [str(disk.GetPath())])
                for dof in ("transX", "transY", "rotX", "rotY"):
                    limit = UsdPhysics.LimitAPI(bolt, dof)
                    self.assertGreater(limit.GetLowAttr().Get(), limit.GetHighAttr().Get(), dof)
                play = UsdPhysics.LimitAPI(bolt, "transZ")
                self.assertAlmostEqual(play.GetLowAttr().Get(), -0.005)
                self.assertAlmostEqual(play.GetHighAttr().Get(), 0.005)
                self.assertFalse(bolt.HasAPI(UsdPhysics.LimitAPI, "rotZ"))
                self.assertEqual(UsdPhysics.DriveAPI(bolt, "rotZ").GetStiffnessAttr().Get(), 0.0)
                # The joint's z is the disk's axis, tilted with its ramp.
                rotation = UsdPhysics.Joint(bolt).GetLocalRot0Attr().Get()
                self.assertLess(rotation.GetReal(), 0.9999)
                self.assertEqual(rotation, UsdPhysics.Joint(bolt).GetLocalRot1Attr().Get())
                # The door's hinge stays a plain revolute joint about the vertical.
                self.assertFalse(hinge.GetLocalRot0Attr().HasAuthoredValue())
                # The seat under each disk carries the disk-on-ramp friction, whatever it meets.
                seat = UsdPhysics.MaterialAPI(stage.GetPrimAtPath(str(root.GetPath()) + "/Materials/slip_seat"))
                self.assertAlmostEqual(seat.GetStaticFrictionAttr().Get(), 0.30)
                self.assertEqual(seat.GetPrim().GetAttribute("physxMaterial:frictionCombineMode").Get(), "min")
                # Pinch points collide.
                panel = stage.GetPrimAtPath(str(root.GetPath()) + "/Structure/stepfields_obstacles/pinch0_panel0")
                self.assertTrue(panel.HasAPI(UsdPhysics.CollisionAPI))
                self.assertFalse(panel.HasAPI(UsdPhysics.RigidBodyAPI))


if __name__ == "__main__":
    unittest.main()
