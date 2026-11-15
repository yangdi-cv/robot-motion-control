from pathlib import Path
import subprocess, time

T = Path(__file__).resolve().parents[1]
deadline = time.monotonic() + 600
while not (T / "runs/control_frequency_norender/RUN_COMPLETE").exists():
    if time.monotonic() > deadline:
        raise TimeoutError("norender run not finished")
    time.sleep(2)
subprocess.run(
    [
        "/usr/bin/python3",
        str(T / "scripts/analyze_control_frequency.py"),
        "--run",
        str(T / "runs/control_frequency_norender"),
    ],
    check=True,
)
for script in [
    "check_software.py",
    "verify.py",
    "write_report.py",
    "build_delivery.py",
]:
    subprocess.run(["/usr/bin/python3", str(T / "scripts" / script)], check=True)
