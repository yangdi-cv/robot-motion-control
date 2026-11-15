"""One-factor perturbations around the same rigid-contact controller."""

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
    "validation_repeat": [],
    "validation_higher": ["--plane-height-shift", ".001"],
    "validation_lower": ["--plane-height-shift", "-.001"],
    "validation_higher5mm": ["--plane-height-shift", ".005"],
    "validation_lower5mm": ["--plane-height-shift", "-.005"],
    "validation_friction_low": ["--friction", ".1"],
    "validation_friction_high": ["--friction", "1."],
    "validation_soft_2k": ["--stiffness", "2000", "--contact-damping", "50"],
    "validation_soft_50k": ["--stiffness", "50000", "--contact-damping", "150"],
    "validation_solver4": ["--solver-iterations", "4"],
    "validation_mass_matrix": ["--inner", "compensated"],
    "validation_5ms": ["--control-dt", ".005"],
    "validation_20ms": ["--control-dt", ".02"],
}
for name, args in cases.items():
    if len(sys.argv) > 1 and name not in sys.argv[1:]:
        continue
    out = R / "runs" / name
    if (out / "COMPLETE").exists():
        continue
    print("START", name, flush=True)
    with (R / "runs" / f"{name}.log").open("w") as log:
        p = subprocess.run(
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
        p.returncode,
        flush=True,
    )
