"""Read-only hardware acquisition for the later Jetson visit; not exercised without a board."""

import argparse, csv, json, sys, time
from pathlib import Path
from collections import deque
import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

p = argparse.ArgumentParser()
p.add_argument("--port", required=True)
p.add_argument("--mode", choices=["static", "motion"], required=True)
p.add_argument("--seconds", type=float, default=30)
p.add_argument("--out", type=Path, required=True)
a = p.parse_args()
T = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(T / "vendor"))
from YbImuSerialLib import YbImuSerial

a.out.mkdir(parents=True, exist_ok=False)


class Capture(Node):
    def __init__(self):
        super().__init__("task2_physical_imu")
        self.started = time.monotonic()
        self.seq = 0
        self.buffer = deque(maxlen=5)
        self.imu = YbImuSerial(a.port)
        self.imu._dev.exclusive = True
        self.imu._dev.timeout = 0
        self.raw = self.create_publisher(Imu, "/imu/data_raw", 100)
        self.filtered = self.create_publisher(Imu, "/imu/data", 100)
        self.f = (a.out / "samples.csv").open("w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(
            ["sequence", "host_monotonic_ns", "ros_time_ns", "time_s"]
            + [
                f"{kind}_{axis}"
                for kind in ["raw", "filtered"]
                for axis in ["ax", "ay", "az", "gx", "gy", "gz"]
            ]
        )
        self.binary = (a.out / "serial_bytes.bin").open("wb")
        original = self.imu._parse_data

        def parse(kind, data):
            original(kind, data)
            if kind == self.imu.FUNC_REPORT_IMU_RAW:
                self.emit()

        self.imu._parse_data = parse
        self.timer = self.create_timer(0.002, self.read)

    def read(self):
        chunk = self.imu._dev.read(min(4096, self.imu._dev.in_waiting))
        self.binary.write(chunk)
        for b in chunk:
            self.imu._receive_data(b)

    def emit(self):
        stamp = time.monotonic_ns()
        ros = self.get_clock().now()
        x = np.r_[
            np.array(self.imu.get_accelerometer_data()) * 9.80665,
            self.imu.get_gyroscope_data(),
        ]
        self.buffer.append(x)
        y = np.mean(self.buffer, axis=0)
        for pub, vals in [(self.raw, x), (self.filtered, y)]:
            m = Imu()
            m.header.stamp = ros.to_msg()
            m.header.frame_id = "imu_sensor"
            m.orientation_covariance[0] = -1.0
            (
                m.linear_acceleration.x,
                m.linear_acceleration.y,
                m.linear_acceleration.z,
            ) = map(float, vals[:3])
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(
                float, vals[3:]
            )
            pub.publish(m)
        self.w.writerow(
            [self.seq, stamp, ros.nanoseconds, time.monotonic() - self.started, *x, *y]
        )
        self.seq += 1

    def close(self):
        self.imu._dev.close()
        self.f.close()
        self.binary.close()
        (a.out / "session.json").write_text(
            json.dumps(
                {
                    "source": "physical IMU",
                    "mode": a.mode,
                    "samples": self.seq,
                    "elapsed_s": time.monotonic() - self.started,
                    "serial_device_setting_writes": 0,
                    "timestamp": "host receipt; no device clock",
                    "filter": "causal5, partial window during first4 samples",
                    "hardware_validation": "must be validated on target device before claiming Jetson results",
                },
                indent=2,
            )
        )


rclpy.init()
n = Capture()
try:
    while time.monotonic() - n.started < a.seconds:
        rclpy.spin_once(n, timeout_sec=0.05)
finally:
    n.close()
    n.destroy_node()
    rclpy.shutdown()
