"""Confirm a slower rigid-contact candidate without changing M=1/B=40."""

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
    "--brake-distance",
    ".015",
    "--minimum-approach-speed",
    ".001",
    "--force-velocity-limit",
    ".003",
    "--solver-iterations",
    "4",
    "--duration",
    "32",
]
cases = {
    "candidate_velocity3": ["--force-velocity-limit", ".003"],
    "candidate_velocity30": ["--force-velocity-limit", ".03"],
    "final_soft_half_step": [
        "--stiffness",
        "50000",
        "--contact-damping",
        "150",
        "--physics-dt",
        ".0005",
    ],
    "final_soft": ["--stiffness", "50000", "--contact-damping", "150"],
    "final_soft_higher5mm": [
        "--stiffness",
        "50000",
        "--contact-damping",
        "150",
        "--plane-height-shift",
        ".005",
    ],
    "final_soft_lower5mm": [
        "--stiffness",
        "50000",
        "--contact-damping",
        "150",
        "--plane-height-shift",
        "-.005",
    ],
    "final_selected": [],
    "final_default": ["--force-velocity-limit", ".001"],
    "final_higher": ["--plane-height-shift", ".001"],
    "final_lower": ["--plane-height-shift", "-.001"],
    "final_higher5mm": ["--plane-height-shift", ".005"],
    "final_lower5mm": ["--plane-height-shift", "-.005"],
    "final_friction_low": ["--friction", ".1"],
    "final_friction_high": ["--friction", "1."],
    "final_repeat": [],
    "final_half_step": ["--physics-dt", ".0005"],
    "video_admittance": ["--record"],
    "video_position": ["--record", "--mode", "position"],
}
for name, args in cases.items():
    if len(sys.argv) > 1 and name not in sys.argv[1:]:
        continue
    out = R / "runs" / name
    if (out / "COMPLETE").exists():
        continue
    active = base.copy()
    if name == "video_position":
        active.remove("--freeze-on-contact")
    print("START", name, flush=True)
    with (R / "runs" / f"{name}.log").open("w") as log:
        proc = subprocess.run(
            [
                str(R / "scripts/isaac-python"),
                str(R / "scripts/run.py"),
                "--out",
                str(out),
                *active,
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
