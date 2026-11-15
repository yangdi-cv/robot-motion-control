"""Six-axis causal filtering and timing analysis of attributed real IMU records."""

from pathlib import Path
import json, csv
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import signal

T = Path(__file__).resolve().parents[1]
D = T / "data/tum_vi"
F = T / "figures"
F.mkdir(exist_ok=True)
AXES = ["ax", "ay", "az", "gx", "gy", "gz"]


def ma5(x):
    out = np.empty_like(x, dtype=float)
    s = np.zeros(x.shape[1])
    for i, v in enumerate(x):
        s += v
        if i >= 5:
            s -= x[i - 5]
        out[i] = s / min(i + 1, 5)
    return out


def analyze(t, x, name):
    assert np.isfinite(x).all() and np.all(np.diff(t) > 0)
    fs = 1 / np.median(np.diff(t))
    y = ma5(x)
    valid = slice(4, None)
    table = []
    for j, ch in enumerate(AXES):
        raw = x[valid, j]
        filt = y[valid, j]
        table.append(
            dict(
                channel=ch,
                unit="m/s2" if j < 3 else "rad/s",
                raw_mean=float(raw.mean()),
                filtered_mean=float(filt.mean()),
                raw_std=float(raw.std(ddof=1)),
                filtered_std=float(filt.std(ddof=1)),
            )
        )
    with (T / "runs" / f"{name}_filter_statistics.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=table[0].keys())
        w.writeheader()
        w.writerows(table)
    with (D / f"{name}_30s.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", *AXES])
        w.writerows(np.c_[t - t[0], x])
    fig, axs = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for j, ax in enumerate(axs.flat):
        ax.plot(t - t[0], x[:, j], lw=0.5, alpha=0.5, label="Raw")
        ax.plot(t - t[0], y[:, j], lw=0.8, label="Causal MA(5)")
        ax.set_ylabel(AXES[j] + (" [m/s2]" if j < 3 else " [rad/s]"))
        ax.grid(alpha=0.2)
    axs[0, 0].legend()
    axs[-1, 0].set_xlabel("Time [s]")
    axs[-1, 1].set_xlabel("Time [s]")
    fig.suptitle(name + " real IMU / 5-point moving average")
    fig.tight_layout()
    fig.savefig(F / f"{name}_six_axes.png", dpi=150)
    plt.close(fig)
    return dict(
        samples=len(t),
        duration_s=float(t[-1] - t[0]),
        median_rate_hz=float(fs),
        mean_rate_hz=float((len(t) - 1) / (t[-1] - t[0])),
        interval_min_ms=float(np.diff(t).min() * 1000),
        interval_max_ms=float(np.diff(t).max() * 1000),
        nominal_ma5_group_delay_ms=float(2000 / fs),
        statistics=table,
    )


static = np.load(D / "static_600s.npy")
t = static[:, 0]
t = t - t[0]
x = static[:, [4, 5, 6, 1, 2, 3]]
mask = (t >= 60) & (t < 90)
res = {"static": analyze(t[mask], x[mask], "static")}
with (D / "room1_imu.csv").open() as f:
    rows = [row for row in csv.reader(f) if row and not row[0].startswith("#")]
stamps = np.array([int(row[0]) for row in rows], dtype=np.int64)
# Subtract epoch timestamps as integers before converting to float seconds.
rt = (stamps - stamps[0]).astype(float) * 1e-9
values = np.array([[float(value) for value in row[1:]] for row in rows])
rx = values[:, [3, 4, 5, 0, 1, 2]]
# Fixed middle 30 seconds; no signal-dependent selection.
start = max(0, (rt[-1] - 30) / 2)
mask = (rt >= start) & (rt < start + 30)
res["motion"] = analyze(rt[mask], rx[mask], "motion")
res["motion"]["selection_start_s"] = float(start)
# Extend static analysis to PSD and overlapping Allan deviation; no hardware bias calibration.
fig, axs = plt.subplots(2, 1, figsize=(10, 7))
fs = float(1 / np.median(np.diff(t)))
for j, ch in enumerate(AXES):
    f, power = signal.welch(x[:, j], fs=fs, nperseg=4096)
    axs[j // 3].loglog(f[1:], np.sqrt(power[1:]), label=ch)
for ax in axs:
    ax.grid(alpha=0.2)
    ax.legend()
    ax.set_xlabel("Frequency [Hz]")
axs[0].set_ylabel("Acceleration ASD")
axs[1].set_ylabel("Angular-rate ASD")
fig.tight_layout()
fig.savefig(F / "static_psd.png", dpi=150)
plt.close(fig)
cs = np.vstack([np.zeros(6), np.cumsum(x, axis=0)])
ms = np.unique(np.logspace(0, np.log10(len(x) // 20), 36).astype(int))
adev = []
for m in ms:
    a = (cs[2 * m :] - 2 * cs[m:-m] + cs[: -2 * m]) / m
    adev.append(np.sqrt(np.mean(a * a, axis=0) / 2))
fig, axs = plt.subplots(1, 2, figsize=(11, 4))
adev = np.array(adev)
for j, ch in enumerate(AXES):
    axs[j // 3].loglog(ms / fs, adev[:, j], label=ch)
for ax in axs:
    ax.set_xlabel("Averaging time [s]")
    ax.grid(alpha=0.2)
    ax.legend()
axs[0].set_ylabel("Acceleration Allan deviation [m/s2]")
axs[1].set_ylabel("Gyro Allan deviation [rad/s]")
fig.tight_layout()
fig.savefig(F / "static_allan.png", dpi=150)
plt.close(fig)
res["static_spectrum"] = {
    "samples": len(x),
    "duration_s": float(t[-1]),
    "temperature_min_C": float(static[:, 7].min()),
    "temperature_max_C": float(static[:, 7].max()),
    "allan_max_tau_s": float(ms[-1] / fs),
    "noise_density_or_bias_calibration_claimed": False,
}
(T / "runs/imu_analysis.json").write_text(json.dumps(res, indent=2))
print(
    json.dumps(
        {k: {a: b for a, b in v.items() if a != "statistics"} for k, v in res.items()},
        indent=2,
    )
)
