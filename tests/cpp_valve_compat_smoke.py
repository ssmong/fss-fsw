"""Native self-tossing valve compatibility using SimHal only, on an isolated ROS domain."""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
from fss_interfaces.msg import SystemMode, ValveCommands, ValveStatus
from fss_interfaces.srv import MainValveControl
import rclpy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
import yaml


class NativeValveCompatibility(unittest.TestCase):
    def exercise(self, *, dry=False, nozzle=-1):
        rclpy.init()
        observer = rclpy.create_node("native_valve_compatibility")
        process = None
        with tempfile.TemporaryDirectory(prefix="fss-valve-compat-") as temp, tempfile.TemporaryFile(mode="w+") as log:
            try:
                parameters = Path(temp) / "params.yaml"
                parameters.write_text(yaml.safe_dump({"valve_driver_node": {"ros__parameters": {
                    "use_sim": True, "use_sim_time": True, "use_thrusters": not dry,
                    "command_input": "duty", "dry_bench_solenoid": dry, "release_nozzle": nozzle,
                    "thrust_map_file": "", "plc": {
                        "ip": "127.0.0.1", "port": 502, "timeout_sec": .1,
                        "valve_register_start": 100, "valve_count": 8,
                        "main_valve_register": 99, "value_range": [0, 4000],
                        "deadband_minimum": 0, "control_rate_hz": 20.0,
                    },
                }}}), encoding="utf-8")
                executable = os.environ.get("FSS_VALVE_TEST_BINARY", str(
                    Path(get_package_prefix("fss_actuators")) / "lib/fss_actuators/valve_driver_node"))
                process = subprocess.Popen([executable, "--ros-args", "--params-file", str(parameters)],
                    stdout=log, stderr=subprocess.STDOUT)
                applied, statuses, events = [], [], []
                subscriptions = [
                    observer.create_subscription(ValveCommands, "/actuators/valve_outputs", applied.append, 10),
                    observer.create_subscription(ValveStatus, "/actuators/dry_bench/valve_status" if dry
                        else "/actuators/valve_status", statuses.append, 10),
                    observer.create_subscription(String, "/actuators/nozzle_solenoid_events" if nozzle >= 0
                        else "/actuators/dry_bench/solenoid_events", lambda msg: events.append(json.loads(msg.data)), 10),
                ]
                mode = observer.create_publisher(SystemMode, "/system/mode", 10)
                duty = observer.create_publisher(ValveCommands, "/actuators/valve_commands", 10)
                release = observer.create_publisher(String, "/hils/release_events", 10)
                clock = observer.create_publisher(Clock, "/clock", 10)
                main = observer.create_client(MainValveControl, "/actuators/main_valve")

                def spin(seconds=.05):
                    deadline = time.monotonic() + seconds
                    while time.monotonic() < deadline:
                        rclpy.spin_once(observer, timeout_sec=.01)
                        if process.poll() is not None:
                            log.seek(0)
                            self.fail(log.read())

                def wait(predicate):
                    deadline = time.monotonic() + 5
                    while not predicate() and time.monotonic() < deadline:
                        spin(.02)
                    log.seek(0)
                    self.assertTrue(predicate(), log.read())

                wait(lambda: duty.get_subscription_count() and mode.get_subscription_count()
                    and clock.get_subscription_count() and main.service_is_ready()
                    and (nozzle < 0 and not dry or release.get_subscription_count()))
                self.assertEqual(observer.count_publishers("/actuators/valve_commands"), 1,
                    "Duty input must never be republished by the driver")
                tick = Clock()
                tick.clock.sec = 100
                clock.publish(tick)
                spin()
                mode_message = SystemMode(mode=SystemMode.ACTIVE)
                mode.publish(mode_message)
                spin()
                duty.publish(ValveCommands(duty_cycles=[.6] + [0.0]*7, enabled=[True]*8))
                if nozzle < 0 and not dry:
                    wait(lambda: applied and applied[-1].duty_cycles[0] == .6)
                    self.assertEqual(statuses[-1].applied_values[0], 2400)
                    mode_message.mode = SystemMode.SAFE_HOLD
                    mode.publish(mode_message)
                    wait(lambda: list(applied[-1].duty_cycles) == [0.0]*8)
                    return
                spin()
                self.assertFalse(any(any(msg.duty_cycles) for msg in applied))
                if dry:
                    future = main.call_async(MainValveControl.Request(open=True))
                    wait(future.done)
                    self.assertFalse(future.result().success)
                payload = dict(event="release_open", state="REQUESTED", stamp_ns=100_000_000_000,
                               observed_elapsed_s=1.0, requested_elapsed_s=1.0)
                release.publish(String(data=json.dumps(payload)))
                # /clock is deliberately frozen: the physical pulse must still close.
                wait(lambda: any(event["state"] == "OFF_ACK" for event in events))
                self.assertEqual([event["state"] for event in events], ["ON_ACK", "OFF_ACK"])
                if nozzle >= 0:
                    self.assertTrue(any(msg.duty_cycles[nozzle] == .6 for msg in applied))
                    self.assertEqual(list(applied[-1].duty_cycles), [0.0]*8)
                release.publish(String(data=json.dumps(payload)))
                spin(.1)
                self.assertEqual(len(events), 2, "Duplicate release must not actuate again")
                payload["stamp_ns"] += 1  # Future timestamp in the paused ROS clock.
                release.publish(String(data=json.dumps(payload)))
                wait(lambda: len(events) == 3)
                self.assertEqual(events[-1]["state"], "REJECTED")
            finally:
                if process is not None and process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5)
                observer.destroy_node()
                rclpy.shutdown()

    def test_direct_duty_and_mode_gate(self):
        self.exercise()

    def test_reserved_nozzle_release_pulse(self):
        self.exercise(nozzle=2)

    def test_dry_bench_release_pulse(self):
        self.exercise(dry=True)


if __name__ == "__main__":
    unittest.main()
