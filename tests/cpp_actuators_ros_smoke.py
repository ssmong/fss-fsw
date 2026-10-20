"""Exercise installed native actuator adapters using simulation and a private ROS domain."""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
from fss_interfaces.msg import SystemMode, ThrustCommands, ValveCommands, ValveStatus
import rclpy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool
import yaml


class NativeActuatorAdapters(unittest.TestCase):
    def test_custom_calibration_gating_wheel_limits_and_ros_clock_pause(self):
        rclpy.init()
        observer = rclpy.create_node("native_actuator_adapter_smoke")
        processes = []
        logs = []
        try:
            with tempfile.TemporaryDirectory(prefix="fss-actuator-smoke-") as temporary:
                directory = Path(temporary)
                (directory / "calibration").mkdir()
                (directory / "hardware").mkdir()
                (directory / "calibration/custom.yaml").write_text(yaml.safe_dump({
                    "duty": [0.0, 1.0], "thrust_n": [[0.0, 2.0]] * 8,
                }), encoding="utf-8")
                wheel_config = yaml.safe_load((Path(__file__).resolve().parents[1]
                    / "config/hardware/reaction_wheel.yaml").read_text(encoding="utf-8"))
                wheel_config["reaction_wheel"]["electrical"]["max_wheel_torque_nm"] = 0.004
                (directory / "hardware/reaction_wheel.yaml").write_text(
                    yaml.safe_dump(wheel_config), encoding="utf-8")
                parameters = directory / "parameters.yaml"
                parameters.write_text(yaml.safe_dump({
                    "valve_driver_node": {"ros__parameters": {
                        "use_sim": True, "use_sim_time": True, "use_thrusters": True,
                        "thrust_map_file": "calibration/custom.yaml", "max_thrust_per_nozzle": 0.5,
                        "plc": {"ip": "127.0.0.1", "port": 502, "timeout_sec": 0.1,
                            "valve_register_start": 100, "valve_count": 8,
                            "main_valve_register": 99, "value_range": [0, 4000],
                            "deadband_minimum": 0, "control_rate_hz": 20.0},
                    }},
                    "reaction_wheel_driver_node": {"ros__parameters": {
                        "dry_run": True, "use_sim_time": True,
                    }},
                }), encoding="utf-8")
                environment = os.environ.copy()
                environment["FSS_CONFIG_DIR"] = str(directory)
                prefix = Path(get_package_prefix("fss_actuators")) / "lib/fss_actuators"
                for executable in ("valve_driver_node", "reaction_wheel_driver_node"):
                    log = tempfile.TemporaryFile(mode="w+")
                    logs.append(log)
                    processes.append(subprocess.Popen(
                        [str(prefix / executable), "--ros-args", "--params-file", str(parameters)],
                        env=environment, stdout=log, stderr=subprocess.STDOUT,
                    ))

                statuses, applied, wheels = [], [], []
                subscriptions = [
                    observer.create_subscription(ValveStatus, "/actuators/valve_status", statuses.append, 10),
                    observer.create_subscription(ValveCommands, "/actuators/valve_commands", applied.append, 10),
                    observer.create_subscription(String, "/reaction_wheel/status",
                        lambda message: wheels.append(json.loads(message.data)), 10),
                ]
                clock = observer.create_publisher(Clock, "/clock", 10)
                mode = observer.create_publisher(SystemMode, "/system/mode", 10)
                thrust = observer.create_publisher(ThrustCommands, "/actuators/thrust_commands", 10)
                torque = observer.create_publisher(Float64, "/reaction_wheel/torque_cmd", 10)
                enable = observer.create_client(SetBool, "/reaction_wheel/enable")
                clock_nanoseconds = 0

                def publish_clock(nanoseconds):
                    nonlocal clock_nanoseconds
                    clock_nanoseconds = nanoseconds
                    message = Clock()
                    message.clock.sec, message.clock.nanosec = divmod(nanoseconds, 1_000_000_000)
                    clock.publish(message)

                def spin_for(seconds):
                    end = time.monotonic() + seconds
                    while time.monotonic() < end:
                        rclpy.spin_once(observer, timeout_sec=0.01)
                        for process, log in zip(processes, logs):
                            if process.poll() is not None:
                                log.seek(0)
                                self.fail(f"Native adapter exited: {log.read()}")

                def wait_for(predicate, timeout=5.0, run_clock=False):
                    end = time.monotonic() + timeout
                    while not predicate() and time.monotonic() < end:
                        if run_clock:
                            publish_clock(clock_nanoseconds + 10_000_000)
                        spin_for(0.01 if run_clock else 0.02)
                    if not predicate():
                        outputs = [os.pread(log.fileno(), 65536, 0).decode("utf-8", errors="replace")
                            for log in logs]
                        self.fail(f"Native adapter did not reach the expected state; "
                            f"recent wheel status: {wheels[-3:]}\nAdapter logs: {outputs}")

                def tick(nanoseconds):
                    publish_clock(nanoseconds)
                    spin_for(0.05)

                wait_for(lambda: clock.get_subscription_count() >= 2
                    and mode.get_subscription_count() and thrust.get_subscription_count()
                    and torque.get_subscription_count() and enable.service_is_ready())
                spin_for(0.15)
                self.assertEqual((len(statuses), len(wheels)), (0, 0))
                tick(10_000_000_000)
                wait_for(lambda: statuses and wheels)
                counts = (len(statuses), len(wheels))
                spin_for(0.15)
                self.assertEqual((len(statuses), len(wheels)), counts)
                tick(10_100_000_000)
                self.assertGreater(len(statuses), counts[0])
                self.assertGreater(len(wheels), counts[1])

                mode_message = SystemMode()
                mode_message.mode = SystemMode.ACTIVE
                mode.publish(mode_message)
                spin_for(0.03)
                command = ThrustCommands()
                command.thrust_n = [1.0] + [0.0] * 7
                command.enabled = [True] * 8
                thrust.publish(command)
                wait_for(lambda: applied and applied[-1].duty_cycles[0] == 0.5
                    and statuses and statuses[-1].applied_values[0] == 2000.0)
                self.assertEqual(statuses[-1].applied_values[0], 2000.0)
                self.assertEqual(statuses[-1].estimated_thrust[0], 1.0)
                mode_message.mode = SystemMode.SAFE_HOLD
                mode.publish(mode_message)
                wait_for(lambda: list(applied[-1].duty_cycles) == [0.0] * 8)

                # Keep polling during wheel interactions; its 100ms safety age uses monotonic time.
                tick(10_200_000_000)
                request = SetBool.Request()
                request.data = True
                future = enable.call_async(request)
                wait_for(future.done, run_clock=True)
                self.assertTrue(future.result().success, future.result().message)
                torque_message = Float64()
                torque_message.data = 0.006  # valid in the repository config, invalid in this custom config
                torque.publish(torque_message)
                wait_for(lambda: wheels and not wheels[-1]["enabled"]
                    and "valid range" in wheels[-1]["last_error"], run_clock=True)
                self.assertEqual(wheels[-1]["telemetry"]["state"], 0)

                # Exercise successful command and disable services on the installed C++ driver.
                request.data = True
                future = enable.call_async(request)
                wait_for(future.done, run_clock=True)
                self.assertTrue(future.result().success, future.result().message)
                torque_message.data = 0.002
                torque.publish(torque_message)
                wait_for(lambda: wheels[-1]["enabled"]
                    and wheels[-1]["telemetry"]["generated_torque_nm"] == 0.002, run_clock=True)
                self.assertTrue(wheels[-1]["dry_run"] and wheels[-1]["connected"])
                request.data = False
                future = enable.call_async(request)
                wait_for(future.done, run_clock=True)
                self.assertTrue(future.result().success, future.result().message)
                wait_for(lambda: not wheels[-1]["enabled"]
                    and wheels[-1]["telemetry"]["state"] == 0, run_clock=True)
        finally:
            for process in processes:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5)
            for log in logs:
                log.close()
            observer.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
