"""Check duration, raw sampling and invariants before reporting completed runs."""

from pathlib import Path
import json, hashlib, numpy as np

R = Path(__file__).resolve().parents[1]
checks = {}
for d in sorted((R / "runs").iterdir()):
    if not d.is_dir() or not (d / "COMPLETE").exists():
        continue
    raw = np.genfromtxt(d / "physics_force.csv", delimiter=",", names=True)
    x = np.genfromtxt(d / "trace.csv", delimiter=",", names=True)
    m = json.loads((d / "metadata.json").read_text())
    a = m["optimization"]
    clock = json.loads((d / "time_audit.json").read_text())
    assert len(raw) == round(a["duration"] / m["physics_dt"]), (d, "raw rows")
    assert len(x) == round(a["duration"] / a["control_dt"]), (d, "control rows")
    assert np.allclose(np.diff(raw["t"]), m["physics_dt"], atol=1e-10), (d, "sampling")
    assert abs(clock["measured_duration"] - a["duration"]) < 1e-5, (d, "clock")
    assert all(np.isfinite(raw[n]).all() for n in raw.dtype.names), (d, "nonfinite raw")
    assert all(np.isfinite(x[n]).all() for n in x.dtype.names), (d, "nonfinite control")
    ref = np.stack([x[f"ref{i}"] for i in range(7)], axis=1)
    moving = x["t"] >= 5.01
    assert np.max(np.abs(np.diff(ref[moving], axis=0))) <= 0.00301, (d, "IK step")
    usda = (d / "scene.usda").read_text()
    assert "physics:staticFriction" in usda
    assert m["gravity"] == ("enabled" if a["gravity"] else "disabled")
    if (d / "source.py").exists():
        assert (
            hashlib.sha256((d / "source.py").read_bytes()).hexdigest()
            == m["source_sha256"]
        ), (d, "source snapshot")
    if "torque0" in x.dtype.names:
        effort = np.stack([x[f"torque{i}"] for i in range(9)], axis=1)
        assert np.all(
            np.abs(effort) <= np.array([87] * 4 + [12] * 3 + [5, 5]) + 1e-5
        ), (d, "effort limit")
    if "sensor_bodies" in m:
        assert len(m["sensor_bodies"]) == 3, (d, "force sensor bodies")
    checks[d.name] = {
        "physics_rows": len(raw),
        "control_rows": len(x),
        "clock_error_s": clock["measured_duration"] - a["duration"],
        "finite": True,
        "reference_step_guard": True,
    }
(R / "verification.json").write_text(json.dumps(checks, indent=2))
print("Verified", len(checks), "runs")

pairs = {}
for left, right in [
    ("final_selected", "final_repeat"),
    ("video_position", "video_admittance"),
]:
    if (R / "runs" / left / "COMPLETE").exists() and (
        R / "runs" / right / "COMPLETE"
    ).exists():
        x = np.genfromtxt(R / "runs" / left / "trace.csv", delimiter=",", names=True)
        y = np.genfromtxt(R / "runs" / right / "trace.csv", delimiter=",", names=True)
        err = max(
            float(np.max(np.abs(x[f"q{i}"][x["t"] < 5] - y[f"q{i}"][y["t"] < 5])))
            for i in range(7)
        )
        referr = max(
            float(np.max(np.abs(x[f"ref{i}"][x["t"] < 5] - y[f"ref{i}"][y["t"] < 5])))
            for i in range(7)
        )
        assert referr < 1e-10, (left, right, "initial target mismatch")
        a = np.genfromtxt(
            R / "runs" / left / "physics_force.csv", delimiter=",", names=True
        )["fz_N"]
        b = np.genfromtxt(
            R / "runs" / right / "physics_force.csv", delimiter=",", names=True
        )["fz_N"]
        pairs[left + "__" + right] = {
            "first5_actual_max_difference_rad": err,
            "first5_reference_max_difference_rad": referr,
            "full_force_max_difference_N": float(np.max(np.abs(a - b))),
        }
(R / "pair_checks.json").write_text(json.dumps(pairs, indent=2))
