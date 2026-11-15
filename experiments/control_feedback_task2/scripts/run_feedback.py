"""Six real-data ROS replay trials; no serial or hardware control."""

from pathlib import Path
import subprocess, time, os, signal, json, argparse

T = Path(__file__).resolve().parents[1]
ROOT = T.parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--out", type=Path, default=T / "runs/feedback001")
parser.add_argument("--input", type=Path, default=T / "data/tum_vi/motion_30s.csv")
parser.add_argument("--serial-port")
args = parser.parse_args()
out = args.out.resolve()
out.mkdir(exist_ok=False)
for source in ([25] if args.serial_port else [200, 25]):
    for feedback in [50, 100, 200]:
        run = out / f"source{source}_feedback{feedback}"
        run.mkdir()
        procs = []
        logs = []
        try:
            for role in ["sink", "relay", "source"]:
                f = (run / f"{role}.log").open("w")
                logs.append(f)
                cmd = [
                    "bash", str(T / "scripts/ros-python"),
                    str(T / "scripts/feedback_node.py"),
                    role,
                    "--input",
                    str(args.input.resolve()),
                    "--out",
                    str(run),
                    "--source-hz",
                    str(source),
                    "--feedback-hz",
                    str(feedback),
                ]
                if args.serial_port:
                    cmd += ["--serial-port", args.serial_port]
                proc = subprocess.Popen(
                    cmd,
                    cwd=ROOT,
                    stdout=f,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                procs.append(proc)
                time.sleep(0.4)
            if procs[-1].wait(timeout=45):
                raise RuntimeError("source failed")
            time.sleep(1)
        finally:
            for proc in procs:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
            for f in logs:
                f.close()
            (run / "process_codes.json").write_text(
                json.dumps([p.poll() for p in procs])
            )
        print("COMPLETE", source, feedback, flush=True)
(out / "RUN_COMPLETE").touch()
