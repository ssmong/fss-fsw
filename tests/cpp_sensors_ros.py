"""Exercise the installed C++ bridges with ROS messages and localhost UDP only."""

import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

import rclpy
from fss_interfaces.msg import SpacecraftState
from sensor_msgs.msg import Imu


def check_close(actual, expected):
    assert math.isclose(actual, expected, rel_tol=1e-8, abs_tol=1e-8), (actual, expected)


def main():
    imu_executable, vicon_executable = sys.argv[1:]
    os.environ.setdefault("ROS_DOMAIN_ID", str(160 + os.getpid() % 60))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
        reserve.bind(("127.0.0.1", 0))
        port = reserve.getsockname()[1]
    topic_root = f"/cpp_sensor_test_{os.getpid()}"
    processes = []
    with tempfile.TemporaryDirectory(prefix="fss-sensors-ros-") as directory:
        logs = []
        node = None
        try:
            configs = [
                (imu_executable, {
                    "input_topic": f"{topic_root}/input", "output_topic": f"{topic_root}/imu",
                    "frame_id": "body", "mounting_quaternion_xyzw": [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)],
                }),
                (vicon_executable, {
                    "bind_ip": "127.0.0.1", "source_ip": "127.0.0.1", "port": port,
                    "object_name": "robotA", "poll_rate_hz": 200.0, "vicon_frame_id": "table",
                    "frame_offset_x": 3.0, "frame_offset_y": 4.0, "frame_rotation_deg": 90.0,
                }),
            ]

            def bridge_args(executable, parameters):
                args = [executable, "--ros-args"]
                for key, value in parameters.items():
                    args.extend(["-p", f"{key}:={json.dumps(value)}"])
                return args

            for executable, parameters in (
                (imu_executable, {**configs[0][1], "mounting_quaternion_xyzw": [0.0, 0.0, 1.0]}),
                (vicon_executable, {**configs[1][1], "port": True}),
            ):
                invalid = subprocess.run(bridge_args(executable, parameters),
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                         text=True, timeout=10, check=False)
                assert invalid.returncode != 0, "Bridge accepted invalid parameter type or shape"
            for index, (executable, parameters) in enumerate(configs):
                log_path = Path(directory) / f"bridge_{index}.log"
                logs.append(log_path)
                with log_path.open("w", encoding="utf-8") as log:
                    processes.append(subprocess.Popen(bridge_args(executable, parameters),
                                                       stdout=log, stderr=subprocess.STDOUT))

            rclpy.init()
            node = rclpy.create_node(f"cpp_sensor_test_{os.getpid()}")
            imus, states = [], []
            node.create_subscription(Imu, f"{topic_root}/imu", imus.append, 10)
            node.create_subscription(SpacecraftState, "/sensors/vicon_state", states.append, 10)
            publisher = node.create_publisher(Imu, f"{topic_root}/input", 10)

            def until(predicate, timeout=10.0):
                deadline = time.monotonic() + timeout
                while not predicate():
                    assert all(process.poll() is None for process in processes), "Bridge exited unexpectedly"
                    assert time.monotonic() < deadline, "ROS bridge test timed out"
                    rclpy.spin_once(node, timeout_sec=0.05)

            def settle(duration=0.2):
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)

            until(lambda: node.count_subscribers(f"{topic_root}/input") > 0
                  and node.count_publishers(f"{topic_root}/imu") > 0
                  and node.count_publishers("/sensors/vicon_state") > 0)
            settle()
            measurement = Imu()
            measurement.header.stamp.sec = 123
            measurement.header.stamp.nanosec = 7
            measurement.header.frame_id = "imu_link"
            measurement.angular_velocity.x = 1.0
            measurement.linear_acceleration.y = 1.0
            measurement.linear_acceleration.z = 9.81
            measurement.orientation.z = math.sqrt(0.5)
            measurement.orientation.w = math.sqrt(0.5)
            measurement.angular_velocity_covariance = [1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0]
            measurement.orientation_covariance = measurement.angular_velocity_covariance
            publisher.publish(measurement)
            until(lambda: len(imus) == 1)
            converted = imus[-1]
            assert converted.header.stamp == measurement.header.stamp
            assert converted.header.frame_id == "body"
            check_close(converted.angular_velocity.x, 0.0)
            check_close(converted.angular_velocity.y, 1.0)
            check_close(converted.linear_acceleration.x, -1.0)
            check_close(converted.linear_acceleration.z, 9.81)
            check_close(converted.orientation.w, 1.0)
            check_close(converted.orientation.z, 0.0)
            check_close(converted.angular_velocity_covariance[0], 2.0)
            check_close(converted.angular_velocity_covariance[4], 1.0)

            measurement.angular_velocity.x = math.nan
            publisher.publish(measurement)
            settle()
            assert len(imus) == 1, "Nonfinite IMU input was published"
            measurement.angular_velocity.x = 1.0
            measurement.header.stamp.nanosec = 8
            measurement.orientation.x = math.nan
            measurement.orientation_covariance = [-1.0] + [math.nan] * 8
            publisher.publish(measurement)
            until(lambda: len(imus) == 2)
            assert imus[-1].orientation_covariance.tolist() == [-1.0] + [0.0] * 8
            assert imus[-1].orientation.x == imus[-1].orientation.w == 0.0
            assert imus[-1].header.stamp == measurement.header.stamp

            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                def send_pose(frame, source_time, x, yaw):
                    data = {"frame": frame, "t": source_time, "objects": {
                        "robotA": {"valid": True, "x": x, "y": 2000.0, "yaw": yaw}}}
                    sender.sendto(json.dumps(data).encode(), ("127.0.0.1", port))

                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as wrong_sender:
                    wrong_sender.bind(("127.0.0.2", 0))
                    wrong_sender.sendto(b'{"frame":0,"t":99,"objects":{"robotA":'
                                        b'{"valid":true,"x":1000,"y":2000,"yaw":0}}}',
                                        ("127.0.0.1", port))
                    settle()
                    assert not states, "Vicon packet from a filtered source was published"
                send_pose(1, 100.0, 1000.0, 0.0)
                until(lambda: len(states) == 1)
                first = states[-1]
                assert first.header.frame_id == "table" and first.header.stamp.sec > 1000
                check_close(first.x, 1.0)
                check_close(first.y, 5.0)
                check_close(first.theta, math.pi / 2)
                assert first.vx == first.vy == first.omega == 0.0
                send_pose(2, 100.1, 1100.0, math.degrees(0.1))
                until(lambda: len(states) == 2)
                check_close(states[-1].vx, 0.0)
                check_close(states[-1].vy, 1.0)
                check_close(states[-1].omega, 1.0)
                sender.sendto(b'{"frame":3,"t":100.2,"objects":{"robotA":{"valid":false}}}',
                              ("127.0.0.1", port))
                sender.sendto(b'{"frame":', ("127.0.0.1", port))
                settle()
                assert len(states) == 2, "Invalid or partial Vicon packet was published"
            print("C++ sensor ROS adapter checks passed")
        except Exception:
            for log in logs:
                print(log.read_text(encoding="utf-8"), file=sys.stderr)
            raise
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for process in processes:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5)


if __name__ == "__main__":
    main()
