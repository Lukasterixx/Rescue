"""The Go2-W (`go2w.py`, `--robot go2w`) without Isaac: Unitree's model as the sim converts it, a wheeled policy's
contract (read by `exported_policy`, refused by the legs-only `deploy_contract`), the exporter's wheel terms, the
left-right mirror with wheels, and the lying cup-demo spawn."""
import copy
import importlib.util
import math
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import torch
import yaml

import go2w
from rescue_sim import exported_policy as xp
from rescue_sim.deploy_contract import DeployContract, DeployContractError

SIM_DIR = Path(__file__).resolve().parents[2]
GO2_BUNDLE = SIM_DIR / "policies" / "go2_krail_2026-10-13"
sys.path.insert(0, str(SIM_DIR / "training"))
import export_krail  # noqa: E402


def wheeled_contract() -> dict:
    """A Go2-W deploy.yaml as `export_krail.py` writes one: the Go2 bundle's, with the wheels' action term and section,
    and the joint terms 16 wide."""
    data = yaml.safe_load((GO2_BUNDLE / "params" / "deploy.yaml").read_text())
    data["default_joint_pos"] = [0.0] * 4 + [0.8] * 4 + [-1.5] * 4
    data["commands"]["base_velocity"]["ranges"]["lin_vel_y"] = [-0.5, 0.5]
    data["actions"]["wheel_vel"] = {"clip": None, "joint_names": list(xp.WHEEL_ORDER), "scale": [5.0] * 4,
                                    "offset": 0.0, "joint_ids": None}
    data["wheels"] = {"joint_names": list(xp.WHEEL_ORDER), "joint_ids_map": [13, 12, 15, 14], "control": "velocity",
                      "stiffness": [0.0] * 4, "damping": [0.5] * 4, "effort_limit": [23.5] * 4,
                      "velocity_limit": [30.0] * 4}
    data["observations"]["joint_vel"]["scale"] = [1.0] * 16
    data["observations"]["actions"]["scale"] = [1.0] * 16
    return data


class ModelTest(unittest.TestCase):
    def setUp(self):
        self.urdf = ET.fromstring(go2w._urdf_for_conversion())
        self.joints = {j.get("name"): j for j in self.urdf.findall("joint")}

    def test_the_legs_and_wheels_are_named_as_the_policies_name_them(self):
        movable = [n for n, j in self.joints.items() if j.get("type") in ("revolute", "continuous")]
        self.assertEqual(sorted(movable), sorted(list(xp.ISAAC_LAB_LEG_ORDER) + list(xp.WHEEL_ORDER)))
        self.assertEqual(tuple(go2w.WHEEL_JOINTS), xp.WHEEL_ORDER)
        self.assertEqual(export_krail.WHEEL_ORDER, xp.WHEEL_ORDER)
        for wheel in xp.WHEEL_ORDER:
            self.assertEqual(self.joints[wheel].get("type"), "continuous")
            self.assertEqual(self.joints[wheel].find("axis").get("xyz"), "0 1 0")    # all four turn about +y

    def test_each_wheel_collides_as_a_cylinder_the_size_of_its_tyre(self):
        for link in self.urdf.findall("link"):
            if not link.get("name").endswith("_foot"):
                continue
            (collision,) = link.findall("collision")
            cylinder = collision.find("geometry/cylinder")
            self.assertEqual(float(cylinder.get("radius")), go2w.WHEEL_RADIUS_M)
            y = float(collision.find("origin").get("xyz").split()[1])
            self.assertAlmostEqual(y, go2w.WHEEL_OFFSET_M * (1 if link.get("name")[1] == "L" else -1))
        # Everything else is a capsule or a sphere (cheaper on the GPU), as long as Unitree's cylinder.
        leftover = [l.get("name") for l in self.urdf.findall("link")
                    if not l.get("name").endswith("_foot") and l.find("collision/geometry/cylinder") is not None]
        self.assertEqual(leftover, [])
        original = {l.get("name"): l for l in ET.parse(go2w.URDF).getroot().findall("link")}
        calf = self.urdf.find("link[@name='FL_calf']").find("collision/geometry/capsule")
        unitree = original["FL_calf"].find("collision/geometry/cylinder")
        self.assertAlmostEqual(float(calf.get("length")) + 2 * float(calf.get("radius")), float(unitree.get("length")))
        # The meshes are found in this folder, with no ROS package path.
        meshes = [m.get("filename") for m in self.urdf.iter("mesh")]
        self.assertTrue(meshes and all(Path(m).is_file() for m in meshes))

    def test_the_standing_pose_puts_the_base_where_the_docstring_says(self):
        thigh = abs(float(self.joints["FL_calf_joint"].find("origin").get("xyz").split()[2]))
        calf = abs(float(self.joints["FL_foot_joint"].find("origin").get("xyz").split()[2]))
        q_thigh, q_calf = go2w.STANDING_LEG_POSE[".*_thigh_joint"], go2w.STANDING_LEG_POSE[".*_calf_joint"]
        height = thigh * math.cos(q_thigh) + calf * math.cos(q_thigh + q_calf)
        self.assertAlmostEqual(height, go2w.STANDING_BASE_OVER_AXLES_M, places=3)
        ahead = -thigh * math.sin(q_thigh) - calf * math.sin(q_thigh + q_calf)
        self.assertLess(abs(ahead), 0.01)    # each wheel within a centimetre of under its hip


class WheeledContractTest(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())
        (self.dir / "params").mkdir()
        shutil.copy(GO2_BUNDLE / "params" / "env.yaml", self.dir / "params" / "env.yaml")
        (self.dir / "policy.pt").write_bytes(b"")

    def tearDown(self):
        shutil.rmtree(self.dir)

    def write(self, data):
        (self.dir / "params" / "deploy.yaml").write_text(yaml.safe_dump(data, sort_keys=False))

    def test_the_sim_reads_the_wheels(self):
        self.write(wheeled_contract())
        bundle = xp.Bundle.load(self.dir)
        w = bundle.wheels
        self.assertEqual((w.joint_names, w.action_scale, w.stiffness, w.damping, w.sdk_ids),
                         (xp.WHEEL_ORDER, 5.0, 0.0, 0.5, (13, 12, 15, 14)))
        self.assertEqual((bundle.robot, bundle.num_actions), ("go2w", 16))
        self.assertEqual(bundle.contract.obs_dim, 48 + 8 + 693)
        self.assertEqual(bundle.contract.action_scale, 0.25)          # the legs' term, as for the Go2
        self.assertIn("wheels", bundle.describe())

    def test_a_legs_only_reader_refuses_it(self):
        """The verbatim reader (and the robot's node it mirrors) drives joint positions only."""
        self.write(wheeled_contract())
        with self.assertRaises(DeployContractError):
            DeployContract.load(str(self.dir))

    def test_the_go2_bundle_has_no_wheels(self):
        bundle = xp.Bundle.load(GO2_BUNDLE)
        self.assertIsNone(bundle.wheels)
        self.assertEqual((bundle.robot, bundle.num_actions), ("go2", 12))

    def test_half_a_wheel_contract_is_refused(self):
        for drop in ("wheels", "action"):
            data = wheeled_contract()
            if drop == "wheels":
                del data["wheels"]
            else:
                del data["actions"]["wheel_vel"]
            self.write(data)
            with self.assertRaises(xp.ExportedPolicyError, msg=drop):
                xp.Bundle.load(self.dir)

    def test_what_the_sim_cannot_honour_is_refused(self):
        cases = {
            "order": lambda d: d["wheels"].update(joint_names=["FR_foot_joint", "FL_foot_joint", "RL_foot_joint",
                                                                "RR_foot_joint"]),
            "position control": lambda d: d["wheels"].update(control="position"),
            "per-wheel scale": lambda d: d["actions"]["wheel_vel"].update(scale=[5.0, 5.0, 4.0, 5.0]),
            "a clip": lambda d: d["actions"]["wheel_vel"].update(clip=[[-1, 1]] * 4),
            "speeds left out": lambda d: d["observations"]["joint_vel"].update(scale=[1.0] * 12),
            "actions left out": lambda d: d["observations"]["actions"].update(scale=[1.0] * 12),
        }
        for what, edit in cases.items():
            data = wheeled_contract()
            edit(data)
            self.write(data)
            with self.assertRaises(xp.ExportedPolicyError, msg=what):
                xp.Bundle.load(self.dir)


class ExporterTest(unittest.TestCase):
    def test_a_go2w_runs_contract(self):
        """export_krail's contract from a Go2-W run's env.yaml: the legs as the Go2's, the wheels after them."""
        env = yaml.load((GO2_BUNDLE / "params" / "env.yaml").read_text(), Loader=export_krail._EnvYamlLoader)
        env = copy.deepcopy(env)
        legs = list(xp.ISAAC_LAB_LEG_ORDER) + list(xp.WHEEL_ORDER)
        for group in ("policy", "critic"):
            env["observations"][group]["joint_vel"]["params"]["asset_cfg"]["joint_names"] = legs
        env["actions"]["wheel_vel"] = {"joint_names": list(xp.WHEEL_ORDER), "preserve_order": True,
                                       "use_default_offset": True, "scale": 5.0}
        env["scene"]["robot"]["actuators"]["wheels"] = {"stiffness": 0.0, "damping": 0.5, "effort_limit_sim": 23.5,
                                                        "velocity_limit_sim": 30.0}
        contract = export_krail.deploy_contract(env)
        self.assertEqual(len(contract["observations"]["joint_vel"]["scale"]), 16)
        self.assertEqual(len(contract["observations"]["actions"]["scale"]), 16)
        self.assertEqual(len(contract["observations"]["joint_pos"]["scale"]), 12)
        self.assertEqual(contract["actions"]["wheel_vel"]["scale"], [5.0] * 4)
        self.assertEqual(contract["wheels"]["damping"], [0.5] * 4)
        # And the sim reads back what it wrote.
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "params").mkdir()
            shutil.copy(GO2_BUNDLE / "params" / "env.yaml", Path(tmp) / "params" / "env.yaml")
            (Path(tmp) / "policy.pt").write_bytes(b"")
            (Path(tmp) / "params" / "deploy.yaml").write_text(yaml.safe_dump(contract, sort_keys=False))
            self.assertEqual(xp.Bundle.load(tmp).num_actions, 16)


class _FakeEnv:
    """What the mirror reads off an env: the scan's pattern and how many actions there are."""

    class _Pattern:
        size, resolution, ordering = (1.6, 1.0), 0.05, "xy"

    def __init__(self, actions):
        scanner = type("S", (), {"cfg": type("C", (), {"pattern_cfg": self._Pattern})})
        self.unwrapped = type("U", (), {"scene": {"height_scanner": scanner},
                                        "action_manager": type("A", (), {"total_action_dim": actions})})


class MirrorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = SIM_DIR / "training" / "go2_rescue" / "krail_symmetry.py"
        spec = importlib.util.spec_from_file_location("krail_symmetry", path)
        cls.sym = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.sym)

    def test_the_go2w_layout_mirrors_back_to_itself(self):
        env = _FakeEnv(16)
        obs = torch.randn(5, 749)
        self.assertTrue(torch.equal(self.sym.mirror_observation(env, self.sym.mirror_observation(env, obs)), obs))

    def test_wheels_swap_sides_with_their_sign(self):
        env = _FakeEnv(16)
        obs = torch.zeros(1, 749)
        obs[0, 24 + 12:24 + 16] = torch.tensor([1.0, 2.0, 3.0, 4.0])      # wheel speeds FL, FR, RL, RR
        obs[0, 40 + 12:40 + 16] = torch.tensor([5.0, 6.0, 7.0, 8.0])      # wheel actions
        out = self.sym.mirror_observation(env, obs)
        self.assertEqual(out[0, 36:40].tolist(), [2.0, 1.0, 4.0, 3.0])
        self.assertEqual(out[0, 52:56].tolist(), [6.0, 5.0, 8.0, 7.0])
        _, actions = self.sym.compute_symmetric_states(env, actions=torch.arange(16.0).unsqueeze(0))
        self.assertEqual(actions[1, 12:].tolist(), [13.0, 12.0, 15.0, 14.0])
        self.assertEqual(actions[1, :4].tolist(), [-1.0, -0.0, -3.0, -2.0])  # hips: swapped and negated

    def test_the_go2_layout_is_unchanged(self):
        env = _FakeEnv(12)
        obs = torch.randn(3, 741)
        out = self.sym.mirror_observation(env, obs)
        self.assertTrue(torch.equal(out[:, 24:36], self.sym._mirror_legs(obs[:, 24:36])))
        self.assertTrue(torch.equal(out[:, 36:48], self.sym._mirror_legs(obs[:, 36:48])))
        with self.assertRaises(ValueError):
            self.sym.mirror_observation(env, torch.randn(1, 749))


class LyingSpawnTest(unittest.TestCase):
    def test_the_go2w_lies_on_its_wheels(self):
        from competition.geometry import Options, build_lanes
        from rescue_sim import levels as lv

        lanes = build_lanes(Options(gravel="static"))
        go2, wheeled = lv.catalogue(lanes, "go2"), lv.catalogue(lanes, "go2w")
        self.assertEqual(go2[-1], lv.CUP_DEMO)
        self.assertEqual(wheeled[-1].spawn[2], lv.GO2W_LYING_SPAWN_HEIGHT_M)
        self.assertEqual(wheeled[-1].spawn[:2], lv.CUP_DEMO.spawn[:2])
        self.assertEqual(go2[:-1], wheeled[:-1])      # the lanes are the same for both


if __name__ == "__main__":
    unittest.main()
