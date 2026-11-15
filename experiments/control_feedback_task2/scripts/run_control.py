"""Start the simulator and external ROS controller; no hardware access."""

import argparse, json, os, signal, subprocess, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
T = ROOT / "experiments/control_feedback_task2"
p = argparse.ArgumentParser()
p.add_argument("--out", type=Path, required=True)
p.add_argument("--config", type=Path, default=T / "config/control_suite.json")
a = p.parse_args()
a.out.mkdir(parents=True, exist_ok=False)
(a.out / "config.json").write_bytes(a.config.read_bytes())
processes = []
logs = []


def spawn(wrapper, script, name):
    f = (a.out / name).open("w")
    logs.append(f)
    proc = subprocess.Popen(
        [
            str(T / "scripts" / wrapper),
            str(T / "scripts" / script),
            "--config",
            str(a.config.resolve()),
            "--out",
            str(a.out.resolve()),
        ],
        cwd=ROOT,
        stdout=f,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    processes.append(proc)
    return proc


try:
    sim = spawn("isaac-python", "isaac_scene.py", "isaac.log")
    deadline = time.monotonic() + 180
    while not (a.out / "sim_ready").exists():
        if sim.poll() is not None:
            raise RuntimeError("Isaac failed")
        if time.monotonic() > deadline:
            raise TimeoutError("Isaac startup")
        time.sleep(0.2)
    control = spawn("ros-python", "controller.py", "controller.log")
    if control.wait(timeout=650):
        raise RuntimeError("controller failed")
    if sim.wait(timeout=25):
        raise RuntimeError("simulator failed")
    (a.out / "RUN_COMPLETE").touch()
    print("CONTROL_COMPLETE", flush=True)
finally:
    for proc in processes:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
    for f in logs:
        f.close()
    (a.out / "exit_codes.json").write_text(json.dumps([p.poll() for p in processes]))
