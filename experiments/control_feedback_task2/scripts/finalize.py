from pathlib import Path
import subprocess, time

T = Path(__file__).resolve().parents[1]
deadline = time.monotonic() + 600
while not (T / "runs/control_frequency/analysis.json").exists():
    if time.monotonic() > deadline:
        raise TimeoutError("final frequency analysis not finished")
    time.sleep(2)
for script in [
    "check_software.py",
    "verify.py",
    "write_report.py",
    "build_delivery.py",
]:
    subprocess.run(["/usr/bin/python3", str(T / "scripts" / script)], check=True)
(T / "RUN_COMPLETE").touch()
