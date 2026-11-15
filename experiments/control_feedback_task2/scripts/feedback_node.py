"""Real-data replay -> causal filter/rate relay -> state-feedback sink (separate ROS processes)."""

import argparse, csv, json, time
from pathlib import Path
from collections import deque
import numpy as np, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu

p = argparse.ArgumentParser()
p.add_argument("role", choices=["source", "relay", "sink"])
p.add_argument("--input", type=Path, required=True)
p.add_argument("--out", type=Path, required=True)
p.add_argument("--source-hz", type=int, required=True)
p.add_argument("--feedback-hz", type=int, required=True)
p.add_argument("--seconds", type=float, default=30)
p.add_argument("--serial-port", help="Read live RAW packets instead of replay; never writes sensor settings")
a = p.parse_args()
a.out.mkdir(parents=True, exist_ok=True)


class Worker(Node):
    def __init__(self):
        super().__init__("task2_" + a.role)
        self.started = time.monotonic()
        self.done = False
        self.seq = 0
        self.last = None
        self.buffer = deque(maxlen=5)
        self.count = 0
        self.seen = set()
        self.native = []
        self.last_new = None
        self.file = (a.out / (a.role + ".csv")).open("w", newline="")
        self.w = csv.writer(self.file)
        if a.role == "source":
            self.pub = self.create_publisher(
                Float64MultiArray, "/task2/source_packet", 1000
            )
            self.imu = self.create_publisher(Imu, "/imu/data_raw", 1000)
            self.rows = None if a.serial_port else np.genfromtxt(a.input, delimiter=",", names=True)
            self.begin = None
            self.w.writerow(
                [
                    "source_sequence",
                    "captured_monotonic_ns",
                    "published_monotonic_ns",
                    "input_index",
                ]
            )
            if a.serial_port:
                import sys
                sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vendor"))
                from YbImuSerialLib import YbImuSerial
                self.device = YbImuSerial(a.serial_port)
                self.device._dev.exclusive = True
                self.device._dev.timeout = 0
                self.binary = (a.out / "serial_bytes.bin").open("wb")
                original = self.device._parse_data
                def parsed(kind, data):
                    original(kind, data)
                    if kind == self.device.FUNC_REPORT_IMU_RAW:
                        self.hardware_sample()
                self.device._parse_data = parsed
                self.timer = self.create_timer(0.002, self.read_serial)
            else:
                self.timer = self.create_timer(1 / a.source_hz, self.source)
        elif a.role == "relay":
            self.pub = self.create_publisher(
                Float64MultiArray, "/task2/feedback_packet", 1000
            )
            self.imu = self.create_publisher(Imu, "/imu/data", 1000)
            self.sub = self.create_subscription(
                Float64MultiArray, "/task2/source_packet", self.new, 1000
            )
            self.timer = self.create_timer(1 / a.feedback_hz, self.relay)
            self.w.writerow(
                [
                    "publication_sequence",
                    "source_sequence",
                    "source_capture_ns",
                    "source_publish_ns",
                    "relay_publish_ns",
                    "filter_samples",
                ]
            )
        else:
            self.sub = self.create_subscription(
                Float64MultiArray, "/task2/feedback_packet", self.sink, 1000
            )
            self.imusub = self.create_subscription(
                Imu, "/imu/data", self.check_imu, 1000
            )
            self.imu_count = 0
            self.w.writerow(
                [
                    "publication_sequence",
                    "source_sequence",
                    "source_capture_ns",
                    "source_publish_ns",
                    "relay_publish_ns",
                    "sink_receive_ns",
                    "sink_done_ns",
                ]
                + [f"filtered_{x}" for x in ["ax", "ay", "az", "gx", "gy", "gz"]]
            )

    def emit_imu(self, pub, vals, ros_ns):
        m = Imu()
        m.header.stamp.sec = ros_ns // 10**9
        m.header.stamp.nanosec = ros_ns % 10**9
        m.header.frame_id = "imu_sensor"
        m.orientation_covariance[0] = -1.0
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(
            float, vals[:3]
        )
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(
            float, vals[3:]
        )
        pub.publish(m)

    def read_serial(self):
        if time.monotonic() - self.started >= a.seconds:
            self.done = True
            return
        chunk = self.device._dev.read(min(4096, self.device._dev.in_waiting))
        self.binary.write(chunk)
        for b in chunk:
            self.device._receive_data(b)

    def hardware_sample(self):
        capture = time.monotonic_ns()
        vals = [*(np.array(self.device.get_accelerometer_data()) * 9.80665),
                *self.device.get_gyroscope_data()]
        ros = self.get_clock().now().nanoseconds
        published = time.monotonic_ns()
        self.pub.publish(Float64MultiArray(data=[float(self.seq), float(capture),
                         float(published), float(ros), *map(float, vals)]))
        self.emit_imu(self.imu, vals, ros)
        self.w.writerow([self.seq, capture, published, -1])
        self.seq += 1

    def source(self):
        if self.pub.get_subscription_count() < 1:
            return
        if self.begin is None:
            self.begin = time.monotonic()
        # Clock follows source timer; low-rate trial decimates the 200Hz recording.
        if self.seq >= round(a.seconds * a.source_hz):
            self.done = True
            return
        index = round(self.seq * 200 / a.source_hz)
        if index >= len(self.rows):
            self.done = True
            return
        row = self.rows[index]
        vals = [float(row[k]) for k in ["ax", "ay", "az", "gx", "gy", "gz"]]
        capture = time.monotonic_ns()
        ros = self.get_clock().now().nanoseconds
        pub = time.monotonic_ns()
        self.pub.publish(
            Float64MultiArray(
                data=[float(self.seq), float(capture), float(pub), float(ros), *vals]
            )
        )
        self.emit_imu(self.imu, vals, ros)
        self.w.writerow([self.seq, capture, pub, index])
        self.seq += 1

    def new(self, m):
        self.last = list(m.data)
        self.buffer.append(np.array(m.data[4:], dtype=float))
        self.last_new = time.monotonic()
        self.native.append(int(m.data[0]))

    def relay(self):
        if self.last is None or self.pub.get_subscription_count() < 1:
            return
        if time.monotonic() - self.last_new > 0.1:
            return  # stop republishing when source stops
        vals = np.mean(self.buffer, axis=0)
        now = time.monotonic_ns()
        s, capture, published, ros = self.last[:4]
        self.pub.publish(
            Float64MultiArray(
                data=[
                    float(self.seq),
                    s,
                    capture,
                    published,
                    float(now),
                    *map(float, vals),
                ]
            )
        )
        self.emit_imu(self.imu, vals, int(ros))
        self.w.writerow(
            [self.seq, int(s), int(capture), int(published), now, len(self.buffer)]
        )
        self.seq += 1

    def sink(self, m):
        rx = time.monotonic_ns()
        seq, source, cap, sp, rp = m.data[:5]
        vals = np.array(m.data[5:])
        assert np.isfinite(vals).all()
        self.last = vals
        done = time.monotonic_ns()
        self.w.writerow(
            [
                int(seq),
                int(source),
                int(cap),
                int(sp),
                int(rp),
                rx,
                done,
                *map(float, vals),
            ]
        )
        self.seen.add(int(seq))
        self.count += 1

    def check_imu(self, m):
        self.imu_count += 1

    def close(self):
        if hasattr(self, "device"):
            self.device._dev.close()
            self.binary.close()
        self.file.close()
        (a.out / (a.role + "_summary.json")).write_text(
            json.dumps(
                {
                    "role": a.role,
                    "published": self.seq,
                    "received": self.count,
                    "received_imu": getattr(self, "imu_count", None),
                    "unique_source_messages": len(set(self.native)),
                    "source_sequence_gaps": sum(
                        max(0, b - c - 1) for c, b in zip(self.native, self.native[1:])
                    ),
                    "source_hz": a.source_hz,
                    "feedback_hz": a.feedback_hz,
                    "source": ("live physical IMU, host parse timestamps; not sensor-clock latency" if a.serial_port else "public TUM-VI real IMU replay, host receipt timestamps; no sensor hardware"),
                    "host_elapsed_s": time.monotonic() - self.started,
                },
                indent=2,
            )
        )


rclpy.init()
n = Worker()
try:
    while not n.done and time.monotonic() - n.started < a.seconds + 12:
        rclpy.spin_once(n, timeout_sec=0.05)
except KeyboardInterrupt:
    pass
finally:
    n.close()
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
