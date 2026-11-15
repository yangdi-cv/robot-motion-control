"""Follow-up changes chosen after rigid-contact sensitivity was observed."""

from pathlib import Path
import subprocess, sys

R = Path(__file__).resolve().parents[1]
base = [
    "--inner",
    "pid",
    "--gravity",
    "--gravity-compensation",
    "--geometry",
    "gripper",
    "--contact-gate",
    "--freeze-on-contact",
    "--ik-seed",
    "differential",
    "--force-feedback",
    "ema",
    "--force-ramp",
    "1",
    "--approach-profile",
    "brake",
]
cases = {
    "refine_damping100": ["--normal-damping", "100"],
    "refine_damping300": ["--normal-damping", "300"],
    "refine_damping100_slow": [
        "--normal-damping",
        "100",
        "--force-velocity-limit",
        ".001",
        "--duration",
        "32",
    ],
    "refine_velocity": ["--force-velocity-limit", ".003"],
    "refine_braking": ["--brake-distance", ".015", "--duration", "32"],
    "refine_combined": [
        "--force-velocity-limit",
        ".003",
        "--brake-distance",
        ".015",
        "--duration",
        "32",
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
                str(R / "scripts/run.py"),
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
