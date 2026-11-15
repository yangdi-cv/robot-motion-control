"""ROS joint position/velocity references for isolated Panda simulation."""

import argparse, csv, json, time
from pathlib import Path
import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory import reference

p = argparse.ArgumentParser()
p.add_argument("--config", type=Path, required=True)
p.add_argument("--out", type=Path, required=True)
a = p.parse_args()
c = json.loads(a.config.read_text())


class Control(Node):
    def __init__(self):
        super().__init__("panda_gain_feedforward_test")
        self.start = None
        self.done = False
        self.count = 0
        self.last_command_sim = None
        self.pub = self.create_publisher(JointState, c["command_topic"], 100)
        self.sub = self.create_subscription(
            JointState, c["state_topic"], self.update, 100
        )
        self.f = (a.out / "tracking.csv").open("w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(
            [
                "sequence",
                "phase",
                "phase_time_s",
                "sim_time_s",
                "host_receive_ns",
                "host_publish_ns",
            ]
            + [
                f"{key}_{i}"
                for key in ["ref", "actual", "velocity_ref", "velocity_actual"]
                for i in range(1, 8)
            ]
        )

    def update(self, m):
        if self.done:
            return
        rx = time.monotonic_ns()
        sim = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.start is None:
            if not self.pub.get_subscription_count():
                return
            self.start = sim
            (a.out / "schedule.json").write_text(json.dumps({"start_sim_time": sim}))
        t = sim - self.start
        phase = int(max(0, t) / c["phase_duration_s"])
        if phase >= len(c["stages"]):
            self.done = True
            self.f.flush()
            (a.out / "controller_done.json").write_text(
                json.dumps({"samples": self.count, "stages": len(c["stages"])})
            )
            return
        condition = c["stages"][phase]
        hz = condition.get("feedback_hz")
        if (
            hz
            and self.last_command_sim is not None
            and sim - self.last_command_sim < 1 / hz - 1e-6
        ):
            return
        self.last_command_sim = sim
        local = t - phase * c["phase_duration_s"]
        start = np.array(c["initial_positions_rad"])
        end = np.array(c["target_positions_rad"])
        end = start + (end - start) * condition.get("amplitude_scale", 1.0)
        if condition["direction"] == "reverse":
            start, end = end, start
        q, qd, _ = reference(
            local - c["hold_before_s"], c["trajectory_duration_s"], start, end
        )
        command = JointState()
        command.header = m.header
        command.name = c["joint_names"][:7]
        command.position = q[:7].tolist()
        command.velocity = (
            qd[:7] if condition["velocity_feedforward"] else np.zeros(7)
        ).tolist()
        self.pub.publish(command)
        indices = [m.name.index(n) for n in command.name]
        actual = [m.position[i] for i in indices]
        vel = [m.velocity[i] for i in indices]
        self.w.writerow(
            [
                self.count,
                phase,
                local,
                sim,
                rx,
                time.monotonic_ns(),
                *q[:7],
                *actual,
                *qd[:7],
                *vel,
            ]
        )
        self.count += 1
        if self.count % 800 == 0:
            self.f.flush()
            print("samples", self.count, "phase", phase, flush=True)


rclpy.init()
node = Control()
begin = time.monotonic()
try:
    while not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - begin > 650:
            raise TimeoutError("controller timeout")
finally:
    node.f.close()
    node.destroy_node()
    rclpy.shutdown()
