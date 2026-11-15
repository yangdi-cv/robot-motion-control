from pathlib import Path
import time, subprocess

T = Path(__file__).resolve().parents[1]
while not (T / "runs/control_validation/RUN_COMPLETE").exists():
    if (T / "runs/control_validation/exit_codes.json").exists():
        raise RuntimeError("validation failed")
    time.sleep(2)
subprocess.run(
    [
        "/usr/bin/python3",
        str(T / "scripts/analyze_control.py"),
        str(T / "runs/control_validation"),
    ],
    check=True,
)
subprocess.run(["/usr/bin/python3", str(T / "scripts/run_feedback.py")], check=True)
subprocess.run(["/usr/bin/python3", str(T / "scripts/analyze_feedback.py")], check=True)
