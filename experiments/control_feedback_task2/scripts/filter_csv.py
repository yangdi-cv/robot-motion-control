"""Analyze either the later six-axis hardware CSV or the normalized public-data CSV."""

import argparse, csv, json
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

p = argparse.ArgumentParser()
p.add_argument("--input", type=Path, required=True)
p.add_argument("--out", type=Path, required=True)
a = p.parse_args()
a.out.mkdir(parents=True, exist_ok=False)
d = np.genfromtxt(a.input, delimiter=",", names=True)
axes = ["ax", "ay", "az", "gx", "gy", "gz"]
names = d.dtype.names
if all("raw_" + x in names for x in axes):
    keys = ["raw_" + x for x in axes]
elif all(x in names for x in axes):
    keys = axes
else:
    raise ValueError("Expected six axes ax,ay,az,gx,gy,gz, optionally prefixed raw_")
t = d["time_s"]
x = np.stack([d[k] for k in keys], axis=1)
assert len(t) > 5 and np.all(np.diff(t) > 0) and np.isfinite(x).all()
y = np.array([x[max(0, i - 4) : i + 1].mean(axis=0) for i in range(len(x))])
statistics = []
for j, ch in enumerate(axes):
    raw = x[4:, j]
    filtered = y[4:, j]
    statistics.append(
        dict(
            channel=ch,
            raw_mean=float(raw.mean()),
            filtered_mean=float(filtered.mean()),
            raw_std=float(raw.std(ddof=1)),
            filtered_std=float(filtered.std(ddof=1)),
        )
    )
with (a.out / "statistics.csv").open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=statistics[0].keys())
    w.writeheader()
    w.writerows(statistics)
with (a.out / "filtered.csv").open("w", newline="") as f:
    w = csv.writer(f)
    w.writerow(
        ["time_s"] + ["raw_" + k for k in axes] + ["filtered_" + k for k in axes]
    )
    w.writerows(np.c_[t, x, y])
fig, axs = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
for j, ax in enumerate(axs.flat):
    ax.plot(t - t[0], x[:, j], lw=0.5, alpha=0.5, label="Raw")
    ax.plot(t - t[0], y[:, j], lw=0.8, label="Causal MA5")
    ax.set_ylabel(axes[j])
    ax.grid(alpha=0.2)
axs[0, 0].legend()
fig.tight_layout()
fig.savefig(a.out / "six_axes.png", dpi=150)
plt.close(fig)
(a.out / "summary.json").write_text(
    json.dumps(
        {
            "input": str(a.input),
            "samples": len(t),
            "duration_s": float(t[-1] - t[0]),
            "observed_rate_hz": float((len(t) - 1) / (t[-1] - t[0])),
            "window_samples": 5,
            "group_delay_estimate_ms": float(2000 * np.median(np.diff(t))),
            "warmup_samples_excluded_from_statistics": 4,
            "stationarity_or_sensor_accuracy_not_assumed": True,
        },
        indent=2,
    )
)
