"""Simulated D1 arm firmware.

Stands in for the arm's onboard controller: consumes `rt/arm_Command`, and
publishes `current_servo_angle` and `rt/arm_Feedback` at 10 Hz exactly as the
real firmware does. A D1Arm client cannot tell this apart from hardware, which
is the whole point -- controllers developed against it port unchanged.

It owns no simulator state. The sim loop pushes measured joint angles in and
pulls commanded targets out; everything else is protocol.
"""
from __future__ import annotations

import threading
import time

from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.pub import DataWriter, Publisher
from cyclonedds.sub import DataReader, Subscriber
from cyclonedds.topic import Topic

from . import d1_protocol as proto
from .d1_client import _READER_QOS, D1Arm, _nic_config
from .d1_msgs import ArmString_, PubServoInfo_


class D1SimServer:
    """Speaks the D1 protocol on behalf of a simulated arm."""

    def __init__(self, domain_id: int = 0, network_interface: str | None = None):
        self._lock = threading.Lock()

        self._domain = None
        config = _nic_config(network_interface)
        if config is not None:
            self._domain = Domain(domain_id, config)

        self._dp = DomainParticipant(domain_id)
        pub, sub = Publisher(self._dp), Subscriber(self._dp)

        # Commands can burst between the 10 Hz drains; queue them rather than
        # silently keeping only the newest.
        self._cmd_r = DataReader(
            sub, Topic(self._dp, proto.TOPIC_ARM_COMMAND, ArmString_), qos=_READER_QOS
        )
        self._fb_w = DataWriter(pub, Topic(self._dp, proto.TOPIC_ARM_FEEDBACK, ArmString_))
        self._servo_w = DataWriter(
            pub, Topic(self._dp, proto.TOPIC_SERVO_ANGLE, PubServoInfo_)
        )

        # Commanded targets (what the host asked for) -- degrees, gripper in mm.
        self._target_deg = [0.0] * proto.NUM_ARM_JOINTS
        self._target_gripper_mm = 0.0
        self._smoothing = proto.SMOOTH_STREAMING

        # Measured state (what the simulator reports) -- degrees, gripper in mm.
        self._measured_deg = [0.0] * proto.NUM_ARM_JOINTS
        self._measured_gripper_mm = 0.0

        self._motors_powered = True
        self._damping = [proto.DAMPING_LOCKED] * proto.NUM_SERVOS
        self._error_status = 1  # 1 = normal

        self._running = False
        self._thread: threading.Thread | None = None
        self._cmd_count = 0

    # ------------------------------------------------------------ lifecycle
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name="d1_sim_server", daemon=True
        )
        self._thread.start()
        print(
            f"[D1] Simulated arm online. cmd='{proto.TOPIC_ARM_COMMAND}' "
            f"state='{proto.TOPIC_SERVO_ANGLE}' @ {proto.FEEDBACK_RATE_HZ:g} Hz"
        )

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        period = 1.0 / proto.FEEDBACK_RATE_HZ
        next_tick = time.monotonic()
        while self._running:
            try:
                self._drain_commands()
                self._publish_state()
            except Exception as exc:  # a firmware thread must not die on one bad frame
                print(f"[D1][WARN] server tick failed: {exc}")
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))

    # ------------------------------------------------------------- commands
    def _drain_commands(self) -> None:
        for sample in D1Arm._drain(self._cmd_r):
            frame = proto.decode(sample.data_)
            if frame is None:
                continue
            if frame.get("address") != proto.ADDR_COMMAND:
                continue  # feedback echoed back to us; ignore

            # The real arm acks receipt before acting, and clients block on it.
            self._ack(proto.FUNC_ACK_RECV, {"recv_status": 1})
            ok = self._apply(frame.get("funcode"), frame.get("data") or {})
            self._ack(proto.FUNC_ACK_EXEC, {"exec_status": 1 if ok else 0})
            self._cmd_count += 1

    def _apply(self, funcode: int, data: dict) -> bool:
        with self._lock:
            if funcode == proto.FUNC_SET_ANGLE:
                joint_id = int(data.get("id", -1))
                angle = float(data.get("angle", 0.0))
                if joint_id == proto.GRIPPER_ID:
                    self._target_gripper_mm = proto.clamp_gripper_mm(angle)
                    return True
                if 0 <= joint_id < proto.NUM_ARM_JOINTS:
                    self._target_deg[joint_id] = proto.clamp_joint_deg(joint_id, angle)
                    return True
                return False

            if funcode == proto.FUNC_SET_ALL_ANGLES:
                self._smoothing = int(data.get("mode", proto.SMOOTH_STREAMING))
                for i in range(proto.NUM_ARM_JOINTS):
                    if f"angle{i}" in data:
                        self._target_deg[i] = proto.clamp_joint_deg(
                            i, float(data[f"angle{i}"])
                        )
                key = f"angle{proto.GRIPPER_ID}"
                if key in data:
                    self._target_gripper_mm = proto.clamp_gripper_mm(float(data[key]))
                return True

            if funcode == proto.FUNC_SET_DAMPING:
                joint_id = int(data.get("id", -1))
                if 0 <= joint_id < proto.NUM_SERVOS:
                    self._damping[joint_id] = int(data.get("mode", proto.DAMPING_LOCKED))
                    return True
                return False

            if funcode == proto.FUNC_SET_ALL_DAMPING:
                mode = int(data.get("mode", proto.DAMPING_LOCKED))
                self._damping = [mode] * proto.NUM_SERVOS
                return True

            if funcode == proto.FUNC_MOTOR_POWER:
                self._motors_powered = int(data.get("power", 1)) == 1
                return True

            if funcode == proto.FUNC_RETURN_TO_ZERO:
                self._target_deg = [0.0] * proto.NUM_ARM_JOINTS
                return True

        return False

    # ------------------------------------------------------------- feedback
    def _ack(self, funcode: int, data: dict) -> None:
        self._fb_w.write(
            ArmString_(data_=proto.encode(proto.FEEDBACK_SEQ, proto.ADDR_ACK, funcode, data))
        )

    def _publish_state(self) -> None:
        with self._lock:
            measured = list(self._measured_deg)
            gripper = self._measured_gripper_mm
            powered = self._motors_powered
            damping = list(self._damping)
            error = self._error_status

        self._servo_w.write(
            PubServoInfo_(*[float(a) for a in measured], float(gripper))
        )

        angles = {f"angle{i}": round(measured[i], 3) for i in range(proto.NUM_ARM_JOINTS)}
        angles[f"angle{proto.GRIPPER_ID}"] = round(gripper, 3)
        self._emit(proto.FUNC_STATE_ANGLES, angles)

        self._emit(
            proto.FUNC_STATE_STATUS,
            {
                "enable_status": 1 if any(d > 0 for d in damping) else 0,
                "power_status": 1 if powered else 0,
                "error_status": error,
            },
        )
        self._emit(
            proto.FUNC_STATE_MOTOR_STATUS,
            {f"motor{i}_status": 1 for i in range(proto.NUM_SERVOS)},
        )

    def _emit(self, funcode: int, data: dict) -> None:
        self._fb_w.write(
            ArmString_(
                data_=proto.encode(proto.FEEDBACK_SEQ, proto.ADDR_STATE, funcode, data)
            )
        )

    # ------------------------------------------------- simulator-side bridge
    def set_measured(self, joint_angles_deg, gripper_mm: float) -> None:
        """Report the simulator's actual joint state, in degrees / millimetres."""
        with self._lock:
            self._measured_deg = [float(a) for a in joint_angles_deg]
            self._measured_gripper_mm = float(gripper_mm)

    def get_targets(self) -> tuple[list[float], float]:
        """Commanded targets for the simulator to drive toward (degrees, mm)."""
        with self._lock:
            return list(self._target_deg), self._target_gripper_mm

    def is_powered(self) -> bool:
        with self._lock:
            return self._motors_powered

    def get_damping(self) -> list[int]:
        with self._lock:
            return list(self._damping)

    def restore_power(self) -> None:
        """Re-energise the motors and re-lock the joints.

        Not part of the wire protocol: the real arm is re-armed by a human at the
        hardware. The simulator calls this on reset so an e-stop from a previous
        run does not leave the arm dead.
        """
        with self._lock:
            self._motors_powered = True
            self._damping = [proto.DAMPING_LOCKED] * proto.NUM_SERVOS

    def sync_targets_to_measured(self) -> None:
        """Snap targets onto the measured state.

        Used on reset: without it the arm would lunge back to a stale target the
        instant the simulator resumes.
        """
        with self._lock:
            self._target_deg = list(self._measured_deg)
            self._target_gripper_mm = self._measured_gripper_mm
