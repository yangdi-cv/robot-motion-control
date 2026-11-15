from pathlib import Path
import time, subprocess

T = Path(__file__).resolve().parents[1]
while not (T / "runs/feedback001/RUN_COMPLETE").exists():
    time.sleep(2)
subprocess.run(
    [
        "/usr/bin/python3",
        str(T / "scripts/run_control.py"),
        "--config",
        str(T / "config/control_frequency.json"),
        "--out",
        str(T / "runs/control_frequency"),
    ],
    check=True,
)
subprocess.run(
    ["/usr/bin/python3", str(T / "scripts/analyze_control_frequency.py")], check=True
)
