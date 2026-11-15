"""Sequential Isaac experiments; a nonzero process exit alone is not a completion check."""

from pathlib import Path
import subprocess, json, sys

R = Path(__file__).resolve().parents[1]
base = [
    "--inner",
    "pid",
    "--contact-gate",
    "--freeze-on-contact",
    "--ik-seed",
    "differential",
    "--force-feedback",
    "ema",
]
cases = {
    "gravity_probe": ["--gravity", "--gravity-compensation"],
    "no_gravity_probe": [],
    "gravity_uncompensated": ["--gravity"],
    "gravity_gripper": ["--gravity", "--gravity-compensation", "--geometry", "gripper"],
    "gripper_ramp": [
        "--gravity",
        "--gravity-compensation",
        "--geometry",
        "gripper",
        "--force-ramp",
        "1.",
    ],
    "gripper_brake": [
        "--gravity",
        "--gravity-compensation",
        "--geometry",
        "gripper",
        "--force-ramp",
        "1.",
        "--approach-profile",
        "brake",
    ],
    "gripper_soft": [
        "--gravity",
        "--gravity-compensation",
        "--geometry",
        "gripper",
        "--force-ramp",
        "1.",
        "--approach-profile",
        "brake",
        "--stiffness",
        "10000",
        "--contact-damping",
        "100",
    ],
}
for name, args in cases.items():
    if len(sys.argv) > 1 and name not in sys.argv[1:]:
        continue
    out = R / "runs" / name
    if (out / "COMPLETE").exists():
        continue
    print("START", name, flush=True)
    with (R / "runs" / f"{name}.log").open("w") as log:
        proc = subprocess.run(
            [
                str(R / "scripts/isaac-python"),
                str(R / "scripts/initial_run.py"),
                "--out",
                str(out),
                *base,
                *args,
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
        )
    print(
        "FINISH",
        name,
        "complete=",
        (out / "COMPLETE").exists(),
        "exit=",
        proc.returncode,
        flush=True,
    )
