"""CycloneDDS IDL types for the Unitree D1 arm.

The typenames below must match the real firmware's IDL byte-for-byte -- DDS
derives its type hash from them, and a mismatch means the arm silently ignores
us. They are reproduced from the D1 SDK's generated headers.

The D1 is not a ROS device: these are raw DDS topics, which is why the arm never
appears in `ros2 topic list`.

Do not add `from __future__ import annotations` here: it stringifies the field
annotations and CycloneDDS's IDL introspection cannot resolve them.
"""
from dataclasses import dataclass

from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import float32, int16, int32, uint8, uint16


@dataclass
class ArmString_(IdlStruct, typename="unitree_arm::msg::dds_::ArmString_"):
    """Carries the JSON payload that makes up almost the whole D1 API."""

    data_: str


@dataclass
class PubServoInfo_(IdlStruct, typename="unitree_arm::msg::dds_::PubServoInfo_"):
    """Joint feedback, published by the arm at 10 Hz. Angles in degrees."""

    servo0_data_: float32
    servo1_data_: float32
    servo2_data_: float32
    servo3_data_: float32
    servo4_data_: float32
    servo5_data_: float32
    servo6_data_: float32


@dataclass
class SetServoAngle_(IdlStruct, typename="unitree_arm::msg::dds_::SetServoAngle_"):
    """Internal to the arm's own controller; the external API uses JSON instead."""

    seq_: int32
    id_: uint8
    angle_: float32
    delay_ms_: int16


@dataclass
class SetServoDumping_(IdlStruct, typename="unitree_arm::msg::dds_::SetServoDumping_"):
    """Internal to the arm's own controller; the external API uses JSON instead.

    'Dumping' is the firmware's spelling of damping, kept for fidelity.
    """

    seq_: int32
    id_: uint8
    power_: uint16
