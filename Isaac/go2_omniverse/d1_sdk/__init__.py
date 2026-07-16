"""Unitree D1 arm SDK.

A faithful Python implementation of the D1's native CycloneDDS protocol, plus a
simulated arm that speaks it. Nothing in this package imports Isaac Sim, so the
same client drives simulation and hardware:

    from d1_sdk import D1Arm

    arm = D1Arm()                          # simulated arm
    arm = D1Arm(network_interface="eth0")  # real arm at 192.168.123.100

See README.md for the protocol summary and the porting checklist.
"""
from .d1_client import D1Arm
from .d1_msgs import ArmString_, PubServoInfo_, SetServoAngle_, SetServoDumping_
from .d1_sim_server import D1SimServer

from . import d1_protocol as protocol

__all__ = [
    "D1Arm",
    "D1SimServer",
    "ArmString_",
    "PubServoInfo_",
    "SetServoAngle_",
    "SetServoDumping_",
    "protocol",
]
