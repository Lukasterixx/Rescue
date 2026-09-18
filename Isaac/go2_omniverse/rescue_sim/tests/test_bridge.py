"""The sim's arm bridge child: where it looks, when it declines, and that it starts, serves and stops. No Isaac, and no
VIP-Rescue: a stand-in bridge script plays the real one."""
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import textwrap
import time
import unittest
from unittest import mock

from rescue_sim import bridge as br

FAKE_BRIDGE = textwrap.dedent('''
    import argparse, os, socket
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    args = parser.parse_args()
    assert args.sim, "the sim must start the bridge in --sim mode"
    print("fake bridge: up", flush=True)
    # The real bridge talks a lot, not all of it ASCII: 200 kB of it before listening, more than any pipe holds.
    for i in range(2000):
        print(f"line {i} \u2014 connected to the simulated arm \u2014 manual control " + "x" * 40, flush=True)
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen()
    while True:
        server.accept()[0].close()
''')


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class ResolveTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(tempfile.mkdtemp())
        self.env = mock.patch.dict(os.environ, {"VIP_RESCUE_ROOT": str(self.root)})
        self.env.start()

    def tearDown(self):
        self.env.stop()

    def make_repo(self, driver=True):
        script = self.root / br.BRIDGE_IN_REPO
        script.parent.mkdir(parents=True)
        script.write_text(FAKE_BRIDGE)
        if driver:
            marker = script.parent / br.DRIVER_BESIDE_BRIDGE
            marker.parent.mkdir(parents=True)
            marker.write_text("")
        return script

    def test_off_and_other_domains_start_nothing(self):
        self.make_repo()
        self.assertIsNone(br.ArmBridge.resolve("off")[0])
        bridge, why = br.ArmBridge.resolve("auto", domain_id=5)
        self.assertIsNone(bridge)
        self.assertIn("domain 5", why)

    def test_says_what_is_missing(self):
        bridge, why = br.ArmBridge.resolve("auto")
        self.assertIsNone(bridge)
        self.assertIn("no arm_bridge.py", why)
        self.make_repo(driver=False)
        bridge, why = br.ArmBridge.resolve("auto")
        self.assertIsNone(bridge)
        self.assertIn("no D1 driver", why)

    def test_finds_vip_rescues_bridge_and_driver(self):
        script = self.make_repo()
        bridge, why = br.ArmBridge.resolve("auto")
        self.assertIsNone(why)
        self.assertEqual(bridge.script, script)
        self.assertIsNotNone(br.ArmBridge.resolve(str(script))[0])


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())
        self.script = self.dir / "arm_bridge.py"
        self.script.write_text(FAKE_BRIDGE)
        self.port = free_port()

    def test_starts_listens_and_stops(self):
        bridge = br.ArmBridge(self.script, port=self.port, log_path=self.dir / "bridge.log")
        self.assertTrue(bridge.start())
        self.assertIn("\u2014 connected to the simulated arm", (self.dir / "bridge.log").read_text(encoding="utf-8"))
        self.assertTrue(br.listening("127.0.0.1", self.port))
        pid = bridge.process.pid
        br.stop_all()
        self.assertFalse(br.listening("127.0.0.1", self.port))
        with self.assertRaises(ProcessLookupError):
            os.kill(pid, 0)

    def test_dies_with_the_sim_even_when_the_sim_skips_its_cleanup(self):
        # Closing Isaac's window ends the sim without its `finally`: the bridge must not outlive it.
        parent = textwrap.dedent(f'''
            import os, sys
            sys.path.insert(0, {str(Path(__file__).resolve().parents[2])!r})
            from rescue_sim import bridge as br
            b = br.ArmBridge({str(self.script)!r}, port={self.port}, log_path={str(self.dir / 'bridge.log')!r})
            assert b.start()
            os._exit(0)
        ''')
        subprocess.run([sys.executable, "-c", parent], check=True, timeout=30, capture_output=True)
        for _ in range(50):
            if not br.listening("127.0.0.1", self.port):
                break
            time.sleep(0.1)
        self.assertFalse(br.listening("127.0.0.1", self.port), "the bridge outlived the sim that started it")

    def test_leaves_a_bridge_that_is_already_there(self):
        first = br.ArmBridge(self.script, port=self.port, log_path=self.dir / "bridge.log")
        self.assertTrue(first.start())
        try:
            second = br.ArmBridge(self.script, port=self.port, log_path=self.dir / "bridge.log")
            self.assertFalse(second.start())
            self.assertIsNone(second.process)
        finally:
            br.stop_all()


if __name__ == "__main__":
    unittest.main()
