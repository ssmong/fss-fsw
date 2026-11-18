"""Exercise the installed C++ executable through its real ROS topics and simulated clock."""

import os
import math
import subprocess
import sys
import tempfile
import time

os.environ.setdefault("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))

import rclpy
from fss_interfaces.msg import SpacecraftState
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


def check_ros_node(executable, use_imu=True):
    parameters = dict(gyro_noise_density=1.2217e-4, accel_noise_density=5.886e-4,
                      gyro_bias_random_walk=1e-5, accel_bias_random_walk=1e-4,
                      table_accel_bias_random_walk=1e-5, vicon_position_noise=5e-4,
                      vicon_heading_noise=1.745e-3, gate_threshold=11.345, nis_window=50,
                      publish_rate_hz=50.0, vicon_timeout_ms=100.0, max_prediction_cycles=50,
                      use_sim_time=True, use_imu=use_imu, vicon_only_velocity_lpf_hz=5.0)
    command = [str(executable), "--ros-args"]
    for name, value in parameters.items():
        command.extend(["-p", f"{name}:={str(value).lower()}"])
    if use_imu:
        for invalid in ("vicon_recovery_samples:=-1", "vicon_delay_ms:=100.0", "vicon_only_velocity_lpf_hz:=0.0"):
            result = subprocess.run(command + ["-p", invalid], capture_output=True, text=True, timeout=5)
            assert result.returncode != 0, f"Invalid navigation parameter accepted: {invalid}"
    rclpy.init()
    node = rclpy.create_node("cpp_navigation_integration_test")
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    imu_pub = node.create_publisher(Imu, "/sensors/imu", 10)
    vicon_pub = node.create_publisher(SpacecraftState, "/sensors/vicon_state", 10)
    states = []
    subscription = node.create_subscription(SpacecraftState, "/navigation/state", states.append, 10)

    def spin(seconds=0.08):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            assert process.poll() is None, "C++ EKF process exited early"

    def clock(stamp):
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(stamp, 1_000_000_000)
        clock_pub.publish(msg)
        spin()

    def imu(stamp, ax=0.0, available=True):
        msg = Imu()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        msg.linear_acceleration.x = ax
        if not available:
            msg.linear_acceleration_covariance[0] = -1.0
        imu_pub.publish(msg)
        spin()

    def vicon(stamp, px=0.0, **fields):
        msg = SpacecraftState()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        msg.header.frame_id = "table"
        msg.x = px
        for name, value in fields.items():
            setattr(msg, name, value)
        vicon_pub.publish(msg)
        spin()

    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 10
            while not ((not use_imu or imu_pub.get_subscription_count()) and vicon_pub.get_subscription_count()
                       and clock_pub.get_subscription_count() and node.count_publishers("/navigation/state")):
                assert time.monotonic() < deadline, "ROS discovery timed out"
                spin()
            spin(0.2)
            if not use_imu:
                assert imu_pub.get_subscription_count() == 0, "Vicon-only mode subscribed to IMU"
                clock(1_000_000_000)
                vicon(1_000_000_000, px=1.0, theta=7.0, vx=1.0, vy=-1.0, omega=0.5)
                clock(1_020_000_000)
                assert states and states[-1].header.stamp.sec == 1
                assert states[-1].header.stamp.nanosec == 0
                assert states[-1].header.frame_id == "table"
                assert abs(states[-1].theta - math.remainder(7.0, 2*math.pi)) < 1e-12
                assert states[-1].vx == 1.0 and states[-1].std_x == 5e-4
                assert states[-1].std_theta == 1.745e-3
                vicon(1_020_000_000, px=1.1, vx=2.0, vy=-2.0, omega=1.0)
                clock(1_040_000_000)
                alpha = -math.expm1(-2*math.pi*5*.02)
                assert abs(states[-1].vx - (1+alpha)) < 1e-12
                assert abs(states[-1].vy + (1+alpha)) < 1e-12
                assert abs(states[-1].omega - (.5+.5*alpha)) < 1e-12
                vicon(1_020_000_000, px=100.0, vx=100.0)  # duplicate must not filter twice
                clock(1_060_000_000)
                assert states[-1].x == 1.1 and abs(states[-1].vx - (1+alpha)) < 1e-12
                vicon(1_060_000_000, vx=float("nan"))
                states.clear()
                clock(1_080_000_000)
                assert not states, "Nonfinite sample left an old state publishing"
                vicon(1_080_000_000, vx=3.0)
                clock(1_100_000_000)
                assert states[-1].vx == 3.0, "Recovery did not reset velocity filter"
                states.clear()
                clock(1_180_000_000)  # strict timeout boundary, age == 100 ms
                assert not states, "Expired Vicon-only state kept publishing"
                vicon(1_180_000_000, vx=-2.0)
                clock(1_200_000_000)
                assert states[-1].vx == -2.0
                assert states[-1].header.stamp.nanosec == 180_000_000
                print("C++ Vicon-only timestamps, velocity LPF, invalid samples and expiry passed")
                return
            clock(1_000_000_000)
            imu(1_000_000_000)
            clock(1_010_000_000)
            imu(1_010_000_000)
            clock(1_020_000_000)
            assert not states, "IMU alone published an uninitialized filter"

            vicon(1_020_000_000)
            imu(1_020_000_000, ax=1.0)
            clock(1_040_000_000)
            assert not states, "First IMU anchor incorrectly published after initialization"
            imu(1_040_000_000, ax=1.0)
            clock(1_060_000_000)
            assert states, "Initialized and predicted filter did not publish"
            assert abs(states[-1].vx - 0.02) < 1e-12
            assert states[-1].header.frame_id == "table"
            assert states[-1].header.stamp.nanosec == 40_000_000
            assert states[-1].std_x > 0.0 and states[-1].std_theta > 0.0

            imu(1_040_000_000, ax=100.0)  # duplicate
            imu(1_035_000_000, ax=100.0)  # reordered
            imu(1_060_000_000, ax=100.0, available=False)
            clock(1_080_000_000)
            assert abs(states[-1].vx - 0.02) < 1e-12
            assert states[-1].header.stamp.nanosec == 40_000_000

            states.clear()
            clock(1_200_000_000)
            assert not states, "IMU outage did not suspend publication"
            vicon(1_020_000_000, px=10.0)  # queued old sample cannot restart
            imu(1_200_000_000)
            clock(1_220_000_000)
            imu(1_220_000_000)
            clock(1_240_000_000)
            assert not states, "Stale Vicon incorrectly resynchronized navigation"

            vicon(1_240_000_000, px=1.0)
            imu(1_240_000_000)
            clock(1_260_000_000)
            assert not states, "First IMU anchor incorrectly published after resynchronization"
            imu(1_260_000_000)
            clock(1_280_000_000)
            assert states and abs(states[-1].x - 1.0) < 1e-12
            assert states[-1].header.stamp.nanosec == 260_000_000
            print("C++ ROS topics, sensor timestamps, readiness, invalid gating and outage recovery passed")
        except Exception:
            log.seek(0)
            print(log.read(), file=sys.stderr)
            raise
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    check_ros_node(sys.argv[1])
    check_ros_node(sys.argv[1], use_imu=False)
