"""Exercise the supervisor with temporary Linux processes and a real hardware lease."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest


COMMON = Path(__file__).resolve().parents[1] / "src/infra/fss_common"


@unittest.skipUnless(sys.platform.startswith("linux"), "Hardware leases and process groups use Linux")
class FlightRunnerTest(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.lock = self.root / "hardware.lock"
        self.runners = []
        self.addCleanup(self.stop_runners)

    def stop_runners(self):
        for runner in self.runners:
            if runner.poll() is None:
                runner.send_signal(signal.SIGTERM)
                runner.wait(timeout=12)

    def script(self, name, body):
        path = self.root / name
        path.write_text(body)
        return path

    def worker(self, name):
        return self.script(name + ".py", (
            "import os, signal, time\nfrom pathlib import Path\n"
            f"Path({str(self.root / (name + '.pid'))!r}).write_text(str(os.getpid()))\n"
            "signal.signal(signal.SIGINT, lambda *_: exit(0))\n"
            "signal.signal(signal.SIGTERM, lambda *_: exit(0))\n"
            "while True: time.sleep(0.05)\n"
        ))

    def start(self, flight, imu=None):
        import fcntl

        command = [sys.executable, "-m", "fss_common.flight_runner"]
        if imu:
            wrapper = self.script("imu.sh", f'exec "{sys.executable}" "{imu}"\n')
            command.extend(["--imu-script", str(wrapper), "--imu-log", str(self.root / "imu.log")])
        command.extend(["--", *flight])
        with self.lock.open("wb") as lease:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            environment = dict(os.environ, PYTHONPATH=str(COMMON),
                               FSS_HARDWARE_LEASE_FD=str(lease.fileno()))
            runner = subprocess.Popen(command, env=environment, pass_fds=(lease.fileno(),),
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                      start_new_session=True)
        self.runners.append(runner)
        return runner

    def wait_for_file(self, path):
        deadline = time.monotonic() + 4
        while not path.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(path.exists(), path)

    def assert_released(self):
        import fcntl

        with self.lock.open("rb") as lease:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for path in self.root.glob("*.pid"):
            stat = Path("/proc") / path.read_text() / "stat"
            try:
                state = stat.read_text().rsplit(")", 1)[1].split()[0]
            except FileNotFoundError:
                continue
            # Orphaned descendants may await init reaping after closing their descriptors.
            self.assertEqual(state, "Z", f"Still running: {path.name}")

    def test_flight_failure_stops_imu_and_releases_lease(self):
        imu = self.worker("imu")
        flight = self.script("flight.py", (
            "import time\nfrom pathlib import Path\n"
            f"while not Path({str(self.root / 'imu.pid')!r}).exists(): time.sleep(0.02)\n"
            "raise SystemExit(23)\n"
        ))
        runner = self.start([sys.executable, str(flight)], imu)
        self.assertEqual(runner.wait(timeout=12), 23)
        self.assert_released()

    def test_imu_failure_stops_flight(self):
        flight = self.worker("flight")
        imu = self.script("imu.py", (
            "import time\nfrom pathlib import Path\n"
            f"while not Path({str(self.root / 'flight.pid')!r}).exists(): time.sleep(0.02)\n"
            "raise SystemExit(17)\n"
        ))
        runner = self.start([sys.executable, str(flight)], imu)
        self.assertEqual(runner.wait(timeout=12), 17)
        self.assert_released()

    def test_normal_flight_exit_stops_imu(self):
        imu = self.worker("imu")
        flight = self.script("flight.py", (
            "import time\nfrom pathlib import Path\n"
            f"while not Path({str(self.root / 'imu.pid')!r}).exists(): time.sleep(0.02)\n"
        ))
        runner = self.start([sys.executable, str(flight)], imu)
        self.assertEqual(runner.wait(timeout=12), 0)
        self.assert_released()

    def test_signal_stops_children_and_holds_lease_until_cleanup(self):
        import fcntl

        flight, imu = self.worker("flight"), self.worker("imu")
        runner = self.start([sys.executable, str(flight)], imu)
        self.wait_for_file(self.root / "flight.pid")
        self.wait_for_file(self.root / "imu.pid")
        with self.lock.open("rb") as lease:
            with self.assertRaises(BlockingIOError):
                fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        runner.send_signal(signal.SIGTERM)
        self.assertEqual(runner.wait(timeout=12), 128 + signal.SIGTERM)
        self.assert_released()

    def test_missing_flight_executable_cleans_started_imu(self):
        imu = self.worker("imu")
        runner = self.start([str(self.root / "missing-executable")], imu)
        self.assertEqual(runner.wait(timeout=12), 1)
        self.assertTrue((self.root / "imu.log").exists())
        self.assert_released()

    def test_cleanup_leaves_an_unrelated_process_running(self):
        external = subprocess.Popen([sys.executable, str(self.worker("external"))],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                    start_new_session=True)
        try:
            self.wait_for_file(self.root / "external.pid")
            flight = self.script("flight.py", "raise SystemExit(23)\n")
            runner = self.start([sys.executable, str(flight)], self.worker("imu"))
            self.assertEqual(runner.wait(timeout=12), 23)
            self.assertIsNone(external.poll())
        finally:
            external.terminate()
            external.wait(timeout=4)
        self.assert_released()

    def test_cleanup_stops_descendant_after_flight_leader_exits(self):
        descendant = self.worker("descendant")
        flight = self.script("flight.py", (
            "import subprocess, sys, time\nfrom pathlib import Path\n"
            f"subprocess.Popen([sys.executable, {str(descendant)!r}])\n"
            f"while not Path({str(self.root / 'descendant.pid')!r}).exists(): time.sleep(0.02)\n"
        ))
        runner = self.start([sys.executable, str(flight)])
        self.assertEqual(runner.wait(timeout=12), 0)
        self.assertTrue((self.root / "descendant.pid").exists())
        self.assert_released()


if __name__ == "__main__":
    unittest.main()
