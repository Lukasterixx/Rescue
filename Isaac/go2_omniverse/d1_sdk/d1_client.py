"""Client for the Unitree D1 arm.

This is the object application code talks to. It speaks the real DDS protocol,
so the *same* client drives the simulated arm and the physical one -- porting is
a constructor argument:

    arm = D1Arm()                             # simulated arm, loopback
    arm = D1Arm(network_interface="eth0")     # real arm on that NIC

It mirrors the official SDK's shape (which is a handful of sample .cpp files
rather than a library) and the community D1Py client, so call sites written
against either translate directly.

Units follow the wire protocol, not the URDF: angles are DEGREES and gripper
stroke is MILLIMETRES. Radians stay on the simulator side of the boundary.
"""
from __future__ import annotations

import itertools
import threading
import time

from cyclonedds.core import Policy, Qos
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.pub import DataWriter, Publisher
from cyclonedds.sub import DataReader, Subscriber
from cyclonedds.topic import Topic

from . import d1_protocol as proto
from .d1_msgs import ArmString_, PubServoInfo_

# The arm publishes several frames per 10 Hz tick (acks plus three state
# messages). With the DDS default of KEEP_LAST(1) a polling reader silently
# drops acks between polls. History depth is a local resource policy, so
# deepening it does not affect QoS matching with the real arm.
_READER_QOS = Qos(Policy.History.KeepLast(64))


def _nic_config(network_interface: str | None) -> str | None:
    """XML binding CycloneDDS to one NIC.

    The real SDK selects the arm by NIC name in ChannelFactory::Init(0, "eth0");
    this is the equivalent, and how you would reach an arm at 192.168.123.100.
    """
    if not network_interface:
        return None
    return (
        '<CycloneDDS><Domain><General>'
        f'<Interfaces><NetworkInterface name="{network_interface}"/></Interfaces>'
        '</General></Domain></CycloneDDS>'
    )


class D1Arm:
    """Talks to a D1 arm -- simulated or real -- over its native DDS protocol."""

    def __init__(self, domain_id: int = 0, network_interface: str | None = None):
        self.domain_id = domain_id
        self._seq = itertools.count(1)
        self._lock = threading.Lock()

        # Holding the Domain matters: dropping it tears down the DDS domain.
        self._domain = None
        config = _nic_config(network_interface)
        if config is not None:
            self._domain = Domain(domain_id, config)

        self._dp = DomainParticipant(domain_id)
        pub, sub = Publisher(self._dp), Subscriber(self._dp)

        self._cmd_w = DataWriter(pub, Topic(self._dp, proto.TOPIC_ARM_COMMAND, ArmString_))
        self._fb_r = DataReader(
            sub, Topic(self._dp, proto.TOPIC_ARM_FEEDBACK, ArmString_), qos=_READER_QOS
        )
        self._servo_r = DataReader(
            sub, Topic(self._dp, proto.TOPIC_SERVO_ANGLE, PubServoInfo_), qos=_READER_QOS
        )

        # Latest state, refreshed by poll(). Degrees; index 6 is gripper mm.
        self._angles = [0.0] * proto.NUM_SERVOS
        # Last gripper stroke we *commanded*. Distinct from the measured value
        # in _angles[6]: holding the measured one would re-pin the target to
        # wherever the jaws happen to be and they could never finish closing.
        self._cmd_gripper_mm = 0.0
        self._enable_status = 0
        self._power_status = 0
        self._error_status = 1  # 1 = normal, 0 = abnormal (note the inversion)
        self._motor_status = [1] * proto.NUM_SERVOS
        self._last_recv_ack = 0.0
        self._last_exec_ack = 0.0
        self._got_state = False

    # ------------------------------------------------------------- commands
    def _send(self, address: int, funcode: int, data: dict | None = None) -> int:
        seq = next(self._seq)
        self._cmd_w.write(ArmString_(data_=proto.encode(seq, address, funcode, data)))
        return seq

    def set_joint_angle(self, joint_id: int, angle_deg: float, delay_ms: int = 0) -> int:
        """Move one joint (protocol id 0..5) to an absolute angle in degrees."""
        angle = proto.clamp_joint_deg(joint_id, angle_deg)
        return self._send(
            proto.ADDR_COMMAND,
            proto.FUNC_SET_ANGLE,
            {"id": int(joint_id), "angle": round(angle, 3), "delay_ms": int(delay_ms)},
        )

    def set_all_joint_angles(
        self,
        angles_deg,
        gripper_mm: float | None = None,
        mode: int = proto.SMOOTH_STREAMING,
    ) -> int:
        """Move all six joints at once -- the command a Cartesian controller wants.

        `mode` selects firmware smoothing: SMOOTH_STREAMING for continuous
        setpoints, SMOOTH_TRAJECTORY for discrete waypoints.

        angle6 is the gripper and is millimetres. When `gripper_mm` is None the
        gripper's current commanded stroke is held rather than being driven to
        zero, so a position controller can ignore the gripper entirely.
        """
        data = {"mode": int(mode)}
        for i in range(proto.NUM_ARM_JOINTS):
            data[f"angle{i}"] = round(proto.clamp_joint_deg(i, angles_deg[i]), 3)
        with self._lock:
            if gripper_mm is not None:
                self._cmd_gripper_mm = proto.clamp_gripper_mm(gripper_mm)
            stroke = self._cmd_gripper_mm
        data[f"angle{proto.GRIPPER_ID}"] = round(stroke, 3)
        return self._send(proto.ADDR_COMMAND, proto.FUNC_SET_ALL_ANGLES, data)

    def set_gripper(
        self, stroke_mm: float, delay_ms: int = proto.GRIPPER_DEFAULT_DELAY_MS
    ) -> int:
        """Set jaw opening in MILLIMETRES (0 = closed, 65 = fully open)."""
        stroke = proto.clamp_gripper_mm(stroke_mm)
        with self._lock:
            self._cmd_gripper_mm = stroke
        return self._send(
            proto.ADDR_COMMAND,
            proto.FUNC_SET_ANGLE,
            {
                "id": proto.GRIPPER_ID,
                "angle": round(stroke, 3),
                "delay_ms": int(delay_ms),
            },
        )

    def set_joint_damping(self, joint_id: int, mode: int) -> int:
        """Damping for one joint, 0 (free) .. 80000 (locked)."""
        mode = max(proto.DAMPING_RELEASED, min(proto.DAMPING_LOCKED, int(mode)))
        return self._send(
            proto.ADDR_COMMAND, proto.FUNC_SET_DAMPING, {"id": int(joint_id), "mode": mode}
        )

    def set_all_damping(self, mode: int) -> int:
        mode = max(proto.DAMPING_RELEASED, min(proto.DAMPING_LOCKED, int(mode)))
        return self._send(proto.ADDR_COMMAND, proto.FUNC_SET_ALL_DAMPING, {"mode": mode})

    def set_motor_power(self, on: bool) -> int:
        """Master motor switch. Off is the e-stop: the arm goes limp and falls."""
        return self._send(
            proto.ADDR_COMMAND, proto.FUNC_MOTOR_POWER, {"power": 1 if on else 0}
        )

    def return_to_zero(self) -> int:
        return self._send(proto.ADDR_COMMAND, proto.FUNC_RETURN_TO_ZERO)

    # ---------------------------------------------------------------- state
    @staticmethod
    def _drain(reader):
        """Yield every queued sample, newest last.

        Draining fully matters: the arm publishes ~30 frames per second and a
        single bounded take() would hand back the *oldest* of them, so a loop
        polling slower than the arm would process ever-staler state and never
        catch up.
        """
        while True:
            batch = reader.take(N=64)
            if not batch:
                return
            yield from batch

    def poll(self) -> bool:
        """Drain feedback into the cached state. Returns True if state arrived.

        Non-blocking, so a control loop can call it every tick. The arm uploads
        at 10 Hz, so most calls legitimately return False.
        """
        fresh = False

        for sample in self._drain(self._servo_r):
            with self._lock:
                for i in range(proto.NUM_SERVOS):
                    self._angles[i] = float(getattr(sample, f"servo{i}_data_"))
                self._got_state = True
            fresh = True

        for sample in self._drain(self._fb_r):
            frame = proto.decode(sample.data_)
            if frame is None:
                continue
            self._ingest_feedback(frame)
            fresh = True

        return fresh

    def _ingest_feedback(self, frame: dict) -> None:
        address, funcode = frame.get("address"), frame.get("funcode")
        data = frame.get("data") or {}
        now = time.monotonic()

        with self._lock:
            if address == proto.ADDR_STATE:
                if funcode == proto.FUNC_STATE_ANGLES:
                    for i in range(proto.NUM_SERVOS):
                        if f"angle{i}" in data:
                            self._angles[i] = float(data[f"angle{i}"])
                    self._got_state = True
                elif funcode == proto.FUNC_STATE_STATUS:
                    self._enable_status = int(data.get("enable_status", 0))
                    self._power_status = int(data.get("power_status", 0))
                    self._error_status = int(data.get("error_status", 1))
                elif funcode == proto.FUNC_STATE_MOTOR_STATUS:
                    for i in range(proto.NUM_SERVOS):
                        if f"motor{i}_status" in data:
                            self._motor_status[i] = int(data[f"motor{i}_status"])
            elif address == proto.ADDR_ACK:
                if funcode == proto.FUNC_ACK_RECV and int(data.get("recv_status", 0)) == 1:
                    self._last_recv_ack = now
                elif funcode == proto.FUNC_ACK_EXEC and int(data.get("exec_status", 0)) == 1:
                    self._last_exec_ack = now

    def get_joint_angles(self) -> list[float]:
        """Six joint angles in degrees, as last reported by the arm."""
        with self._lock:
            return list(self._angles[: proto.NUM_ARM_JOINTS])

    def get_gripper_mm(self) -> float:
        with self._lock:
            return self._angles[proto.GRIPPER_ID]

    def get_status(self) -> dict:
        with self._lock:
            return {
                "enable_status": self._enable_status,
                "power_status": self._power_status,
                "error_status": self._error_status,
                "motor_status": list(self._motor_status),
            }

    def is_connected(self) -> bool:
        """True once the arm has reported joint state at least once."""
        with self._lock:
            return self._got_state

    def wait_for_state(self, timeout: float = 5.0) -> bool:
        """Block until the arm reports state. Real arms take a moment to appear
        on the bus after power-on, so call this before commanding motion."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll()
            if self.is_connected():
                return True
            time.sleep(0.01)
        return False
