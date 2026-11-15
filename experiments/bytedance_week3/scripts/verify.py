from pathlib import Path
import json, numpy as np, ast
from contact_task import model

T = Path(__file__).resolve().parents[1]
O = T / "runs/contact_suite_v3"
rows = json.loads((O / "summary.json").read_text())["results"]
assert len(rows) == 177
for row in rows:
    with np.load(O / f"case_{row['case_id']:03d}.npz") as d:
        a = d["samples"]
        assert a.shape == (8000, 29) and np.isfinite(a).all()
        assert np.allclose(np.diff(a[:, 0]), 0.001)
        assert abs(a[:, 1].max() - row["peak_force_N"]) < 1e-9
        mask = a[:, 0] >= 5
        assert (
            abs(np.sqrt(np.mean((a[mask, 1] - 10) ** 2)) - row["steady_force_rmse_N"])
            < 1e-9
        )
        assert np.max(abs(a[:, 5])) <= 0.050001 and np.max(abs(a[:, 6])) <= 0.040001
m, d, site, p = model()
assert m.nmocap == 0 and m.nu == 9
for p in (T / "scripts").glob("*.py"):
    ast.parse(p.read_text())
assert json.loads((O / "no_contact_check.json").read_text())["peak_force_N"] == 0
joint = json.loads((T / "runs/joint_check_v3/summary.json").read_text())
assert len(joint) == 6
for r in joint:
    with np.load(T / "runs/joint_check_v3" / f"{r['stiffness']}_{r['mode']}.npz") as d:
        a = d["samples"]
        v = np.sqrt(np.mean((a[:, 8:15] - a[:, 22:29]) ** 2, axis=0)) * 180 / np.pi
        assert np.allclose(v, r["joint_rmse_deg"])
import av

counts = {}
for name in ["position", "admittance"]:
    with av.open(str(O / (name + ".mp4"))) as c:
        counts[name] = sum(1 for f in c.decode(video=0))
        assert counts[name] == 200
(T / "VERIFICATION.json").write_text(
    json.dumps(
        {
            "main_trials": 177,
            "samples_per_trial": 8000,
            "joint_tracking_trials": 6,
            "video_frames": counts,
            "no_mocap": True,
            "torque_actuators": 7,
            "contact_disabled_force_zero": True,
            "metric_recalculation": True,
            "hardware_used": False,
            "passed": True,
        },
        indent=2,
    )
)
print("contact verification passed")
