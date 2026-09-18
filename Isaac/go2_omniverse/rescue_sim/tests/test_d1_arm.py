"""The simulated D1's firmware, driven through its wire protocol with no Isaac and no DDS."""
import math
import unittest

try:
    import torch
except ImportError:
    torch = None

from rescue_sim import d1_model as m

# The D1 URDF's hard limits (d1_arm/d1.urdf), radians.
LOWER = [-2.35, -1.57, -1.57, -2.35, -1.57, -2.35]
UPPER = [2.35, 1.57, 1.57, 2.35, 1.57, 2.35]
POLICY_DT = 0.02


def firmware(seed=0):
    from rescue_sim.d1_arm import D1Firmware

    rest = m.rest_joint_rad(LOWER, UPPER)
    return D1Firmware(POLICY_DT, LOWER, UPPER, rest, m.units_to_travel(m.REST_GRIPPER_UNITS), seed=seed)


def all_angles(servo_deg, gripper=None, mode=0):
    data = {"mode": mode, **{f"angle{i}": a for i, a in enumerate(servo_deg)}}
    if gripper is not None:
        data["angle6"] = gripper
    return {"seq": 1, "address": 1, "funcode": 2, "data": data}


def run(fw, steps, q=None):
    """Advance `steps` policy steps with a perfect drive (the arm is wherever the plan is). Returns the feedbacks."""
    out = []
    for _ in range(steps):
        fw.begin_step(q if q is not None else fw.planner.position[0].tolist())
        for _ in range(4):
            fw.physics_step(0.005)
        fb = fw.end_step(fw.planner.position[0].tolist(), fw.gripper_target_m)
        if fb is not None:
            out.append(fb)
    return out


class WireConventionTests(unittest.TestCase):
    def test_gripper_units_span_the_measured_ends(self):
        self.assertAlmostEqual(m.units_to_travel(-19.8), -0.0076, places=12)      # pads 2 mm apart
        self.assertAlmostEqual(m.units_to_travel(50.2), 0.03, places=12)
        self.assertAlmostEqual(m.units_to_travel(-40.0), m.units_to_travel(-19.8))  # clamps like the arm
        self.assertAlmostEqual(m.units_to_travel(65.0), m.units_to_travel(50.2))
        for units in (-19.8, -3.0, 0.0, 12.5, 40.8, 50.2):
            self.assertAlmostEqual(m.travel_to_units(m.units_to_travel(units)), units, places=9)

    def test_gripper_limits_are_widened_as_d1training_widens_them(self):
        # run_pick_demo.py's formula for a 2 mm shut jaw, 0.9 soft factor, the URDF's 0..30 mm finger.
        shut = (0.002 - 0.0172) / 2.0
        reach = (2 * (shut - 0.0005) - 0.03 * (1 - 0.9)) / (1 + 0.9)
        lower, upper = m.gripper_joint_limits(0.0, 0.03, 0.9)
        self.assertAlmostEqual(lower, reach, places=12)
        self.assertEqual(upper, 0.03)
        soft_lower = (lower + upper) / 2 - (upper - lower) / 2 * 0.9
        self.assertAlmostEqual(soft_lower, shut - 0.0005, places=12)

    def test_servo_signs_invert_servos_0_and_3(self):
        q = m.servo_to_joint_rad([10, 10, 10, 10, 10, 10])
        self.assertEqual([round(math.degrees(v), 9) for v in q], [-10, 10, 10, -10, 10, 10])
        self.assertEqual([round(v, 9) for v in m.joint_to_servo_deg(q)], [10.0] * 6)

    def test_rest_is_f023s_pose_inside_the_urdf(self):
        rest = m.rest_joint_rad(LOWER, UPPER)
        self.assertAlmostEqual(math.degrees(rest[0]), -2.1, places=9)      # servo 0 inverted
        self.assertAlmostEqual(rest[1], LOWER[1] + m.REST_LIMIT_MARGIN_RAD)  # -90.9 is past the limit
        self.assertAlmostEqual(rest[2], UPPER[2] - m.REST_LIMIT_MARGIN_RAD)  # +91.7 is past the limit
        self.assertAlmostEqual(math.degrees(rest[3]), 3.1, places=9)


@unittest.skipIf(torch is None, "PyTorch unavailable in this interpreter")
class FirmwareTests(unittest.TestCase):
    def test_it_starts_parked_at_rest_and_reports_it_in_servo_space(self):
        fw = firmware()
        feedback = run(fw, 12)
        self.assertEqual(len(feedback), 2)                 # every 6 steps: 120 ms at 50 Hz
        servo = feedback[-1].servo_deg
        self.assertAlmostEqual(servo[0], 2.1, places=6)
        self.assertAlmostEqual(servo[1], -89.9, places=6)  # -89.95 deg less the margin, quantised to 0.1
        self.assertAlmostEqual(servo[3], -3.1, places=6)
        self.assertAlmostEqual(feedback[-1].gripper_units, 40.8, places=6)
        for value in servo + [feedback[-1].gripper_units]:
            self.assertAlmostEqual(value * 10, round(value * 10), places=6)

    def test_a_command_moves_the_joints_the_real_arm_would(self):
        fw = firmware()
        self.assertTrue(fw.receive(all_angles([20, 0, 0, 20, 0, 0], gripper=50.2)))
        run(fw, 100)
        goal = [math.degrees(v) for v in fw.planner.position[0].tolist()]
        self.assertAlmostEqual(goal[0], -20.0, places=4)   # servo 0 turns against Joint1
        self.assertAlmostEqual(goal[3], -20.0, places=4)
        self.assertAlmostEqual(goal[1], 0.0, places=4)
        self.assertAlmostEqual(fw.gripper_target_m, 0.03, places=12)

    def test_commands_reach_the_planner_at_10_hz_latest_first(self):
        fw = firmware()
        fw.receive(all_angles([0] * 6))
        self.assertTrue(fw.begin_step())
        fw.end_step(fw.planner.position[0].tolist(), 0.0)
        for step in range(1, 5):                     # inside the 100 ms hold: kept, not passed on
            fw.receive(all_angles([step] * 6))
            self.assertFalse(fw.begin_step(), step)
            fw.end_step(fw.planner.position[0].tolist(), 0.0)
        self.assertTrue(fw.begin_step())             # the fifth step passes the latest
        self.assertAlmostEqual(math.degrees(float(fw.goal_q[0, 1])), 4.0, places=4)
        self.assertEqual(fw.commands_issued, 2)
        self.assertEqual(fw.commands_received, 5)

    def test_a_single_joint_command_leaves_the_others(self):
        fw = firmware()
        before = fw.goal_q.clone()
        fw.receive({"address": 1, "funcode": 1, "data": {"id": 4, "angle": 30.0, "delay_ms": 0}})
        fw.begin_step()
        changed = (fw.goal_q - before).abs()[0] > 1e-6
        self.assertEqual(changed.tolist(), [False, False, False, False, True, False])

    def test_targets_are_held_to_the_joint_limits(self):
        fw = firmware()
        fw.receive(all_angles([0, -150, 150, 0, 0, 0]))
        fw.begin_step()
        self.assertAlmostEqual(float(fw.goal_q[0, 1]), LOWER[1], places=6)
        self.assertAlmostEqual(float(fw.goal_q[0, 2]), UPPER[2], places=6)

    def test_mode_1_slews_at_a_fifth(self):
        fw = firmware()
        fw.park([0.0] * 6, 0.0)
        fw.receive(all_angles([0, 0, 0, 0, 60, 0], mode=1))
        peak = 0.0
        for _ in range(100):
            fw.begin_step()
            for _ in range(4):
                _, velocity = fw.physics_step(0.005)
                peak = max(peak, abs(float(velocity[0, 4])))
            fw.end_step(fw.planner.position[0].tolist(), 0.0)
        self.assertAlmostEqual(peak, m.VELOCITY_LIMIT_RAD_S["Joint5"] * m.MODE1_SPEED_FACTOR, places=5)

    def test_release_goes_limp_and_a_motion_command_reenergises_from_where_it_fell(self):
        fw = firmware()
        fw.receive({"address": 1, "funcode": 5, "data": {"mode": 0}})
        self.assertFalse(fw.energised)
        fw.receive({"address": 1, "funcode": 5, "data": {"mode": 1}})   # enable is implicit: a no-op
        self.assertFalse(fw.energised)
        fw.receive({"address": 1, "funcode": 6, "data": {"power": 1}})  # power commands change nothing
        self.assertFalse(fw.energised)
        fallen = [0.2, -1.0, 1.2, 0.0, 0.3, 0.0]
        fw.receive(all_angles(m.joint_to_servo_deg(fallen)))
        fw.begin_step(fallen)
        self.assertTrue(fw.energised)
        self.assertEqual([round(v, 6) for v in fw.planner.position[0].tolist()], [round(v, 6) for v in fallen])

    def test_power_off_is_ignored_as_on_the_arm(self):
        fw = firmware()
        self.assertTrue(fw.receive({"address": 1, "funcode": 6, "data": {"power": 0}}))
        self.assertTrue(fw.energised)

    def test_return_to_zero_plans_to_the_zero_pose(self):
        fw = firmware()
        fw.receive({"address": 1, "funcode": 7})
        run(fw, 200)
        for value in fw.planner.position[0].tolist():
            self.assertAlmostEqual(value, 0.0, places=6)

    def test_a_level_load_is_not_undone_by_a_streamed_old_target(self):
        fw = firmware()
        out = [30, -20, 40, 0, 20, 0]
        for _ in range(3):
            fw.receive(all_angles(out, gripper=10.0))
            run(fw, 5)
        rest = m.rest_joint_rad(LOWER, UPPER)
        fw.park(rest, m.units_to_travel(m.REST_GRIPPER_UNITS))      # the sim loaded a level
        for _ in range(3):                                         # the bridge keeps repeating its last target
            fw.receive(all_angles(out, gripper=10.0))
            run(fw, 5)
        for got, want in zip(fw.planner.position[0].tolist(), rest):
            self.assertAlmostEqual(got, want, places=5)
        fw.receive(all_angles([0, -20, 40, 0, 20, 0], gripper=10.0))   # a new target moves it again
        run(fw, 100)
        self.assertAlmostEqual(math.degrees(fw.planner.position[0, 1].item()), -20.0, places=3)

    def test_frames_that_are_not_commands_are_ignored(self):
        fw = firmware()
        self.assertFalse(fw.receive({"address": 2, "funcode": 1, "data": {"angle0": 50}}))
        self.assertFalse(fw.receive({"address": 1, "funcode": 99}))
        self.assertFalse(fw.begin_step())

    def test_status_is_sent_at_its_measured_period(self):
        fw = firmware()
        sent = sum(fw.status_due(POLICY_DT) for _ in range(500))     # 10 s
        self.assertIn(sent, (99, 100))


if __name__ == "__main__":
    unittest.main()
