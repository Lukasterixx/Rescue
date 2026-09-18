"""The arm bridge, started by the sim so that nothing else has to be.

On the robot the behaviour tree reaches the D1 through VIP-Rescue's `maps/arm_bridge.py`, in the split stack's
vip-arm container. The tree sends newline-delimited JSON to 127.0.0.1:8084, and only the bridge speaks the arm's
CycloneDDS protocol, through the team's D1 driver (`unitree-d1-control`). The sim runs that same bridge with
`--sim`, as a child process. The tree, the website's arm panel and anything else that drives the arm then reach the
sim's D1 exactly as they reach the real one, with no terminal of its own. The bridge runs on the sim's Python, which
has the cyclonedds binding the driver needs.

Where things are:
- The bridge script: `--arm-bridge PATH`, or by default VIP-Rescue's, under $VIP_RESCUE_ROOT (default ~/VIP-Rescue).
- The driver: $D1_DRIVER_ROOT, or by default VIP-Rescue's submodule `Docker/unitree-d1-control`.

If something already listens on the port (the website's own bridge, say), the sim starts none: that bridge serves the
arm, since it too talks to whatever D1 is on DDS domain 0. The bridge the sim starts dies with the sim, however the sim
ends: closing Isaac's window skips the sim's own cleanup, so the kernel is asked to do it (PR_SET_PDEATHSIG).

Its output goes to a log file (`generated/arm_bridge.log`), never a pipe. A pipe the sim stopped reading would block
the bridge on its next log line, and a blocked bridge stops streaming targets: the arm freezes mid-move.

No Isaac imports.
"""
from __future__ import annotations

import ctypes
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8084
BRIDGE_IN_REPO = Path("Docker/Isaac/go2_ws/src/go2_control_cpp/maps/arm_bridge.py")
DRIVER_IN_REPO = Path("Docker/unitree-d1-control")
DRIVER_MARKER = Path("src/arm_control.py")       # what arm_bridge.py imports D1Arm from
DEFAULT_LOG = Path(__file__).resolve().parent / "generated" / "arm_bridge.log"


def vip_rescue_root() -> Path:
    return Path(os.environ.get("VIP_RESCUE_ROOT") or Path.home() / "VIP-Rescue").expanduser()


def listening(host: str, port: int, timeout: float = 0.3) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


PR_SET_PDEATHSIG = 1


def _die_with_parent() -> None:
    """In the child, before exec: SIGTERM it when the process that started it exits."""
    ctypes.CDLL("libc.so.6", use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGTERM)


def say(line: str) -> None:
    print(f"[arm_bridge] {line}", flush=True)


_started: list["ArmBridge"] = []


def stop_all() -> None:
    """Stop every bridge this process started: the sim's last word, however it ends."""
    while _started:
        _started.pop().stop()


class ArmBridge:
    """VIP-Rescue's arm bridge as a child of the sim: `start()` after the sim's D1 is on DDS, `stop()` on the way out."""

    def __init__(self, script: Path, driver: Path, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT,
                 log_path: Path = DEFAULT_LOG):
        self.script, self.driver, self.host, self.port = Path(script), Path(driver), host, port
        self.log_path = Path(log_path)
        self.process: subprocess.Popen | None = None

    @classmethod
    def resolve(cls, choice: str, port: int = DEFAULT_PORT, domain_id: int = 0):
        """(bridge, None) to start one, or (None, why not). `choice` is 'auto', 'off' or the path of arm_bridge.py."""
        if choice == "off":
            return None, "not started (--arm-bridge off)"
        if domain_id != 0:
            return None, (f"not started: the sim's D1 is on DDS domain {domain_id}, and the D1 driver only talks to "
                          f"domain 0, as the real arm does")
        script = vip_rescue_root() / BRIDGE_IN_REPO if choice == "auto" else Path(choice).expanduser()
        if not script.is_file():
            return None, f"not started: no arm_bridge.py at {script} (set VIP_RESCUE_ROOT or --arm-bridge PATH)"
        driver = Path(os.environ.get("D1_DRIVER_ROOT") or vip_rescue_root() / DRIVER_IN_REPO).expanduser()
        if not (driver / DRIVER_MARKER).is_file():
            return None, (f"not started: no D1 driver at {driver}. In VIP-Rescue: "
                          f"git submodule update --init {DRIVER_IN_REPO} (or set D1_DRIVER_ROOT)")
        return cls(script, driver, port=port), None

    def start(self, wait_s: float = 8.0) -> bool:
        if listening(self.host, self.port):
            say(f"a bridge is already listening on {self.host}:{self.port}: it serves the arm, so the sim starts none")
            return False
        env = dict(os.environ)
        env["D1_DRIVER_ROOT"] = str(self.driver)
        env["PYTHONIOENCODING"] = "utf-8"            # Isaac's locale is ASCII; the bridge's messages are not
        env.pop("CYCLONEDDS_URI", None)
        command = [sys.executable, "-u", str(self.script), "--sim", "--host", self.host, "--port", str(self.port)]
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.log_path, "wb") as log:
            # Its own session: a Ctrl+C in the sim's terminal is the sim's to handle, and stop() ends the bridge after.
            # And it dies with the sim: the kernel sends it SIGTERM if the sim exits first.
            self.process = subprocess.Popen(command, env=env, stdin=subprocess.DEVNULL, stdout=log,
                                            stderr=subprocess.STDOUT, start_new_session=True,
                                            preexec_fn=_die_with_parent)
        _started.append(self)
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                say(f"exited at startup with code {self.process.returncode}; the arm has no bridge. Its log "
                    f"({self.log_path}) ends:\n{self._tail()}")
                return False
            if listening(self.host, self.port):
                say(f"up on {self.host}:{self.port} (pid {self.process.pid}), logging to {self.log_path}")
                return True
            time.sleep(0.2)
        say(f"not listening on {self.host}:{self.port} after {wait_s:.0f} s; carrying on without waiting for it "
            f"(log: {self.log_path})")
        return False

    def _tail(self, lines: int = 15) -> str:
        try:
            text = self.log_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return ""
        return "\n".join(text.splitlines()[-lines:])

    def stop(self, timeout_s: float = 3.0) -> None:
        process, self.process = self.process, None
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=timeout_s)
        except ProcessLookupError:
            pass
        say("stopped")
