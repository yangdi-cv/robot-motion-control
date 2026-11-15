"""Real localhost UDP between processes; synthetic chunks, no robot actuation."""

from pathlib import Path
import socket, time, json, csv, multiprocessing as mp
import numpy as np

R = Path(__file__).resolve().parents[1]


def sender(port, start, duration, case):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pending = []
    for seq in range(round(duration / 0.05)):
        deadline = start + seq * 0.05
        while time.monotonic() < deadline:
            time.sleep(0.0005)
        now = time.monotonic()
        if case == "outage" and 1.0 <= now - start < 2.0:
            continue
        values = []
        for i in range(30):
            u = np.clip((now - start + i * 0.01) / duration, 0, 1)
            values.append(float(0.1 * (10 * u**3 - 15 * u**4 + 6 * u**5)))
        pkt = {"seq": seq, "generation": now, "step": 0.01, "action": values}
        if case == "delay":
            time.sleep(0.04)
        pkt["send"] = time.monotonic()
        sock.sendto(json.dumps(pkt).encode(), ("127.0.0.1", port))
    sock.close()


def run(case, duration=4.0):
    folder = R / "network" / case
    folder.mkdir(parents=True, exist_ok=True)
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.setblocking(False)
    start = time.monotonic() + 0.3
    child = mp.get_context("spawn").Process(
        target=sender, args=(receiver.getsockname()[1], start, duration, case)
    )
    child.start()
    latest = None
    latencies = []
    age_values = []
    source_delay = []
    received = 0
    timeouts = 0
    held = False
    action = 0.0
    rows = []
    for i in range(round(duration / 0.01)):
        deadline = start + i * 0.01
        while time.monotonic() < deadline:
            time.sleep(0.0003)
        now = time.monotonic()
        while True:
            try:
                data, _ = receiver.recvfrom(65535)
            except BlockingIOError:
                break
            packet = json.loads(data)
            arrival = time.monotonic()
            received += 1
            latencies.append(arrival - packet["send"])
            source_delay.append(arrival - packet["generation"])
            if latest is None or packet["seq"] > latest["seq"]:
                latest = packet
        now = time.monotonic()
        age = None if latest is None else now - latest["generation"]
        timeout = age is None or age > 0.5
        if timeout:
            if not held:
                timeouts += 1
            held = True
        else:
            index = int(age / latest["step"])
            action = latest["action"][min(max(index, 0), len(latest["action"]) - 1)]
            held = False
            age_values.append(age)
        rows.append(
            [
                now - start,
                (now - deadline) * 1000,
                -1 if age is None else age * 1000,
                int(timeout),
                action,
            ]
        )
    child.join(timeout=3)
    assert child.exitcode == 0
    receiver.close()
    with (folder / "receiver.csv").open("w") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "wall_time_s",
                "scheduling_lateness_ms",
                "source_age_ms",
                "hold",
                "synthetic_command_rad",
            ]
        )
        w.writerows(rows)
    a = np.array(rows)
    result = {
        "case": case,
        "transport": "UDP 127.0.0.1, separate processes, same host monotonic clock",
        "source": "analytic quintic future action chunks; no VLA and no robot",
        "duration_wall_s": duration,
        "sender_hz": 20,
        "receiver_target_hz": 100,
        "packet_count": received,
        "receive_poll_latency_p95_ms": float(np.quantile(latencies, 0.95) * 1000),
        "generation_to_receive_p95_ms": float(np.quantile(source_delay, 0.95) * 1000),
        "receiver_lateness_p95_ms": float(np.quantile(a[:, 1], 0.95)),
        "hold_samples": int(a[:, 3].sum()),
        "hold_events_including_startup": timeouts,
        "injected_application_delay_ms": 40 if case == "delay" else 0,
        "injected_sender_outage_s": [1, 2] if case == "outage" else None,
    }
    (folder / "summary.json").write_text(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    results = {case: run(case) for case in ["normal", "delay", "outage"]}
    (R / "network_results.json").write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
