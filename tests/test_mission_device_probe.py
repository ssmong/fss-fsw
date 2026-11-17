"""Passive hardware-owner discovery against synthetic procfs and device metadata."""

from pathlib import Path
import os
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/infra/fss_common"))
from fss_common.mission_device_probe import device_resource_owner


class DeviceResourceOwner(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.proc = Path(temp.name) / "proc"
        self.dev = Path(temp.name) / "dev"
        self.net = self.proc / "net"
        self.net.mkdir(parents=True)
        (self.net / "tcp").write_text("header\n")
        (self.net / "udp").write_text("header\n")
        (self.dev / "serial/by-id").mkdir(parents=True)

    def line(self, protocol, local, remote, state="01", inode=810):
        with (self.net / protocol).open("a") as stream:
            stream.write(f"0: {local} {remote} {state} 0:0 0:0 0 1000 0 {inode}\n")

    def inspect(self, **options):
        return device_resource_owner(self.proc, dev=self.dev, **options)

    def test_native_imu_reuse_requires_one_vendor_driver_on_the_same_ros_network(self):
        port = self.dev / "ttyUSB9"
        port.touch()
        link = self.dev / "serial/by-id/usb-Xsens_fixture"
        process = self.proc / "712"
        fd = process / "fd/1"
        fd.parent.mkdir(parents=True)
        try:
            link.symlink_to(port)
            fd.symlink_to(port)
        except OSError as exc:
            self.skipTest(f"Symbolic links unavailable: {exc}")
        (process / "cmdline").write_bytes(b"/opt/vendor/xsens_mti_node\0")
        (process / "environ").write_bytes(b"ROS_DOMAIN_ID=0\0ROS_LOCALHOST_ONLY=1\0")
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "1"}, clear=True):
            self.assertIn("Xsens", self.inspect())  # Legacy launch still owns its own raw driver.
            self.assertIsNone(self.inspect(share_imu=True))
            for key, value in (("ROS_DOMAIN_ID", "7"), ("ROS_LOCALHOST_ONLY", "0"),
                               ("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")):
                with patch.dict(os.environ, {key: value}):
                    self.assertIn("Xsens", self.inspect(share_imu=True))
            (process / "cmdline").write_bytes(b"/usr/bin/unrelated-reader\0")
            self.assertIn("Xsens", self.inspect(share_imu=True))
            (process / "cmdline").write_bytes(b"/opt/vendor/xsens_mti_node\0")
            self.line("udp", "00000000:268E", "00000000:0000", "07")
            self.assertIn("Vicon", self.inspect(share_imu=True))
            (self.net / "udp").write_text("header\n")
            second = self.proc / "713"
            (second / "fd").mkdir(parents=True)
            (second / "fd/1").symlink_to(port)
            (second / "cmdline").write_bytes((process / "cmdline").read_bytes())
            (second / "environ").write_bytes((process / "environ").read_bytes())
            self.assertIn("중복", self.inspect(share_imu=True))

    def test_bound_udp_reported_even_with_no_visible_pid(self):
        self.line("udp", "00000000:268E", "00000000:0000", "07")
        self.assertEqual(self.inspect(), "Vicon UDP :9870")

    def test_open_plc_socket_matches_exact_peer_and_owner(self):
        self.line("tcp", "0101A8C0:A771", "0201A8C0:01F6")
        fd = self.proc / "53/fd/20"
        fd.parent.mkdir(parents=True)
        fd.touch()
        with patch("fss_common.mission_device_probe.os.readlink", return_value="socket:[810]"):
            self.assertEqual(self.inspect(), "PLC TCP 192.168.1.2:502 (PID 53)")

    def test_unrelated_udp_and_modbus_and_time_wait_not_owners(self):
        self.line("udp", "00000000:2328", "0101A8C0:268E", "07")
        self.line("tcp", "0101A8C0:A771", "9901A8C0:01F6")
        self.line("tcp", "0101A8C0:A771", "0201A8C0:01F6", "06", 0)
        self.assertIsNone(self.inspect())

    def test_restricted_physical_driver_blocks_launch_but_unrelated_process_does_not(self):
        process = self.proc / "124"
        process.mkdir()
        original_iterdir = Path.iterdir

        def restricted(path):
            if path == process / "fd":
                raise PermissionError
            return original_iterdir(path)

        with patch.object(Path, "iterdir", restricted):
            (process / "cmdline").write_bytes(b"/usr/bin/python3\0/opt/ros/lib/arm_driver\0")
            self.assertIn("arm_driver (PID 124)", self.inspect())
            (process / "cmdline").write_bytes(b"")
            (process / "comm").write_text("valve_driver_no\n")
            self.assertIn("valve_driver_node (PID 124)", self.inspect())
            (process / "comm").write_text("systemd\n")
            self.assertIsNone(self.inspect())

    def test_renumbered_ftdi_and_xsens_ports(self):
        for by_id, name in (("usb-Xsens_Motion_Tracker_Dev._Board_SERIAL", "Xsens"),
                            ("usb-FTDI_USB__-__Serial_Converter_FTAKRQ9X-if00-port0", "팔 FTDI")):
            with self.subTest(by_id=by_id):
                port = self.dev / "ttyUSB8"
                port.touch()
                link = self.dev / "serial/by-id" / by_id
                fd = self.proc / "712/fd/1"
                fd.parent.mkdir(parents=True, exist_ok=True)
                try:
                    link.symlink_to(port)
                    fd.symlink_to(port)
                except OSError as exc:
                    self.skipTest(f"Symbolic links unavailable: {exc}")
                self.assertEqual(self.inspect(), f"{name} {port} (PID 712)")
                fd.unlink()
                link.unlink()

    def test_unreadable_network_metadata_or_scan_timeout_blocks_start(self):
        (self.net / "tcp").unlink()
        self.assertIn("확인 실패", self.inspect())
        (self.net / "tcp").write_text("header\n")
        (self.proc / "12/fd").mkdir(parents=True)
        with patch("fss_common.mission_device_probe.time.monotonic", side_effect=[100.0, 104.0]):
            self.assertIn("시간 초과", self.inspect())
