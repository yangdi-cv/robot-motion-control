"""Summarize motion tracking, static residual, overshoot and reference-delay fit."""

from pathlib import Path
import argparse, json, csv
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

p = argparse.ArgumentParser()
p.add_argument("run", type=Path)
p.add_argument("--select", action="store_true")
a = p.parse_args()
c = json.loads((a.run / "config.json").read_text())
d = np.genfromtxt(a.run / "tracking.csv", delimiter=",", names=True)
rows = []
for phase, cond in enumerate(c["stages"]):
    part = d[d["phase"] == phase]
    t = part["phase_time_s"]
    moving = (t >= 2) & (t <= 7)
    hold = t > 7.2
    for j in range(1, 4):
        ref = part[f"ref_{j}"]
        actual = part[f"actual_{j}"]
        error = ref - actual
        velocity = part[f"velocity_actual_{j}"]
        vref = part[f"velocity_ref_{j}"]
        direction = np.sign(ref[-1] - ref[0])
        lags = np.arange(-0.1, 0.401, 0.001)
        loss = [
            np.mean((np.interp(t[moving] - lag, t, ref) - actual[moving]) ** 2)
            for lag in lags
        ]
        lag = lags[np.argmin(loss)]
        rows.append(
            dict(
                phase=phase,
                joint=j,
                **cond,
                samples=int(moving.sum()),
                motion_rmse_rad=float(np.sqrt(np.mean(error[moving] ** 2))),
                motion_max_abs_rad=float(np.max(np.abs(error[moving]))),
                lag_fit_ms=float(lag * 1000),
                velocity_rmse_rad_s=float(
                    np.sqrt(np.mean((vref[moving] - velocity[moving]) ** 2))
                ),
                hold_mean_abs_rad=float(np.mean(np.abs(error[hold]))),
                hold_std_rad=float(np.std(actual[hold], ddof=1)),
                overshoot_rad=float(
                    max(0, np.max(direction * (actual[t >= 2] - ref[-1])))
                ),
            )
        )
with (a.run / "joint_metrics.csv").open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=rows[0].keys())
    w.writeheader()
    w.writerows(rows)
groups = []
for scale, ff in sorted(set((x["kp_scale"], x["velocity_feedforward"]) for x in rows)):
    r = [x for x in rows if x["kp_scale"] == scale and x["velocity_feedforward"] == ff]
    groups.append(
        dict(
            kp_scale=scale,
            kd_scale=scale**0.5,
            velocity_feedforward=ff,
            motion_rmse_deg_mean=float(
                np.mean([x["motion_rmse_rad"] for x in r]) * 180 / np.pi
            ),
            motion_max_abs_deg_worst=float(
                max(x["motion_max_abs_rad"] for x in r) * 180 / np.pi
            ),
            lag_fit_ms_mean=float(np.mean([x["lag_fit_ms"] for x in r])),
            hold_mean_abs_deg_mean=float(
                np.mean([x["hold_mean_abs_rad"] for x in r]) * 180 / np.pi
            ),
            hold_std_deg_worst=float(max(x["hold_std_rad"] for x in r) * 180 / np.pi),
            overshoot_deg_worst=float(max(x["overshoot_rad"] for x in r) * 180 / np.pi),
        )
    )
summary = {
    "groups": groups,
    "stages": len(c["stages"]),
    "total_feedback_samples": len(d),
    "lag_method": "least squared delayed reference fit, -100..400ms, 1ms grid; not transport latency",
}
if a.select:
    eligible = [
        g
        for g in groups
        if g["overshoot_deg_worst"] <= 0.5 and g["hold_std_deg_worst"] <= 0.1
    ]
    best = min(eligible or groups, key=lambda g: g["motion_rmse_deg_mean"])
    summary["selected"] = best
    summary["selection_rule"] = (
        "mean J1-J3 motion RMSE; overshoot<=0.5deg and hold std<=0.1deg, otherwise lowest RMSE with constraint failure disclosed"
    )
    val = json.loads(json.dumps(c))
    val["stages"] = []
    for repeat in range(2):
        for amp in [0.6, 1.2]:
            for direction in ["forward", "reverse"]:
                for setting in [
                    dict(kp_scale=1.0, kd_scale=1.0, velocity_feedforward=False),
                    {
                        k: best[k]
                        for k in ["kp_scale", "kd_scale", "velocity_feedforward"]
                    },
                ]:
                    val["stages"].append(
                        dict(
                            **setting,
                            repeat=repeat,
                            direction=direction,
                            amplitude_scale=amp,
                        )
                    )
    (a.run.parents[1] / "config/control_validation.json").write_text(
        json.dumps(val, indent=2)
    )
(a.run / "analysis.json").write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
labels = [
    f"Kp x{g['kp_scale']:g}" + (" + velocity FF" if g["velocity_feedforward"] else "")
    for g in groups
]
fig, axs = plt.subplots(1, 2, figsize=(12, 4))
axs[0].bar(labels, [g["motion_rmse_deg_mean"] for g in groups])
axs[0].set_ylabel("Mean J1-J3 motion RMSE [deg]")
axs[1].bar(labels, [g["lag_fit_ms_mean"] for g in groups])
axs[1].set_ylabel("Reference-delay fit [ms]")
for ax in axs:
    ax.tick_params(axis="x", rotation=35)
    ax.grid(axis="y", alpha=0.2)
fig.tight_layout()
fig.savefig(a.run / "tracking_comparison.png", dpi=160)
plt.close(fig)
