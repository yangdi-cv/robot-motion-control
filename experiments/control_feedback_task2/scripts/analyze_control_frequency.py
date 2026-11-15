from pathlib import Path
import json, csv, argparse
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

T = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--run", type=Path, default=T / "runs/control_frequency")
args = parser.parse_args()
R = args.run.resolve()
c = json.loads((R / "config.json").read_text())
d = np.genfromtxt(R / "tracking.csv", delimiter=",", names=True)
rows = []
for phase, s in enumerate(c["stages"]):
    v = d[d["phase"] == phase]
    t = v["phase_time_s"]
    mask = (t >= 2) & (t <= 7)
    q = np.stack([v[f"ref_{j}"] for j in range(1, 4)], axis=1)
    actual = np.stack([v[f"actual_{j}"] for j in range(1, 4)], axis=1)
    dt = np.diff(v["host_publish_ns"]) * 1e-9
    rows.append(
        dict(
            phase=phase,
            **s,
            rmse_deg=float(
                np.sqrt(np.mean((q[mask] - actual[mask]) ** 2)) * 180 / np.pi
            ),
            max_abs_deg=float(np.max(np.abs(q[mask] - actual[mask])) * 180 / np.pi),
            samples=len(v),
            observed_host_hz=float(1 / np.mean(dt)),
            host_interval_std_ms=float(dt.std(ddof=1) * 1000),
        )
    )
with (R / "metrics.csv").open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=rows[0].keys())
    w.writeheader()
    w.writerows(rows)
groups = []
for hz in [50, 100, 200]:
    for ff in [False, True]:
        rr = [
            r
            for r in rows
            if r["feedback_hz"] == hz and r["velocity_feedforward"] == ff
        ]
        groups.append(
            dict(
                feedback_hz=hz,
                tuned_and_velocity_ff=ff,
                mean_rmse_deg=float(np.mean([r["rmse_deg"] for r in rr])),
                worst_abs_deg=float(max(r["max_abs_deg"] for r in rr)),
                mean_host_hz=float(np.mean([r["observed_host_hz"] for r in rr])),
                phases=len(rr),
            )
        )
(R / "analysis.json").write_text(
    json.dumps(
        {
            "groups": groups,
            "physics_dt_s": c["physics_dt_s"],
            "stages": len(c["stages"]),
            "feedback_basis": "simulation timestamp gating of actual ROS callbacks; host rates separately measured",
            "imu_used_for_joint_control": False,
        },
        indent=2,
    )
)
fig, ax = plt.subplots(figsize=(8, 4))
for ff in [False, True]:
    rr = [r for r in groups if r["tuned_and_velocity_ff"] == ff]
    ax.plot(
        [r["feedback_hz"] for r in rr],
        [r["mean_rmse_deg"] for r in rr],
        "o-",
        label="Tuned + velocity FF" if ff else "Default",
    )
ax.set_xlabel("Outer feedback / command rate [Hz, simulation time]")
ax.set_ylabel("J1-J3 motion RMSE [deg]")
ax.grid(alpha=0.2)
ax.legend()
fig.tight_layout()
fig.savefig(
    T
    / (
        "figures/panda_frequency_norender.png"
        if R.name.endswith("norender")
        else "figures/panda_frequency.png"
    ),
    dpi=160,
)
plt.close(fig)
print(json.dumps(groups, indent=2))
