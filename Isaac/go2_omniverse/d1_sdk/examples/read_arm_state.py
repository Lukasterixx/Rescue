"""Print the D1's joint angles at 10 Hz.

Works against the simulator or the real arm -- the only difference is the
constructor. Run it while the sim is up:

    python d1_sdk/examples/read_arm_state.py
    python d1_sdk/examples/read_arm_state.py --interface eth0   # real arm
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from d1_sdk import D1Arm

parser = argparse.ArgumentParser()
parser.add_argument("--interface", default=None, help="NIC of the real arm, e.g. eth0")
parser.add_argument("--domain", type=int, default=0)
args = parser.parse_args()

arm = D1Arm(domain_id=args.domain, network_interface=args.interface)
if not arm.wait_for_state(timeout=5.0):
    sys.exit("No arm found. Is the simulator running (or the arm powered)?")

print("joint angles (deg)                              gripper   status")
try:
    while True:
        arm.poll()
        angles = " ".join(f"{a:7.2f}" for a in arm.get_joint_angles())
        status = arm.get_status()
        print(
            f"{angles}   {arm.get_gripper_mm():5.1f}mm   "
            f"power={status['power_status']} err={status['error_status']}"
        )
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
