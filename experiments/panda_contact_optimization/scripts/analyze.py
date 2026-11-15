"""Compare raw physics contact and fixed 10 ms bins, independent of control filter."""

from pathlib import Path
import json
import numpy as np

R = Path(__file__).resolve().parents[1]


def metrics(d):
    x = np.genfromtxt(d / "trace.csv", delimiter=",", names=True)
    raw = np.genfromtxt(d / "physics_force.csv", delimiter=",", names=True)
    physics_dt = json.loads((d / "metadata.json").read_text())["physics_dt"]
    stride = round(0.01 / physics_dt)
    f = raw["fz_N"]
    n = len(f) // stride
    b = f[: n * stride].reshape(-1, stride).mean(axis=1)
    t = np.arange(n) * 0.01 + 0.01
    tail_raw = f[-round(2 / physics_dt) :]
    tail = b[-200:]
    mask = np.abs(b - 5) <= 0.5
    longest = cur = 0
    first = None
    hit = np.flatnonzero(b > 0.5)
    h = t[hit[0]] if len(hit) else None
    for i, v in enumerate(mask):
        cur = cur + 1 if v else 0
        longest = max(longest, cur)
        if cur >= 51 and first is None:
            first = t[i - 50]
    q = np.stack([x[f"q{i}"] for i in range(3)], axis=1)
    qr = np.stack([x[f"ref{i}"] for i in range(3)], axis=1)
    return {
        "duration_s": len(f) * physics_dt,
        "raw_peak_N": float(f.max()),
        "max_single_step_impulse_Ns": float(f.max() * physics_dt),
        "cycle_peak_N": float(b.max()),
        "last2_mean_N": float(tail.mean()),
        "last2_rmse_N": float(np.sqrt(np.mean((tail - 5) ** 2))),
        "last2_in_band_pct": float(np.mean(np.abs(tail - 5) <= 0.5) * 100),
        "longest_in_band_s": max(0, longest - 1) * 0.01,
        "first_contact_s": h,
        "settle_after_contact_s": (
            None if first is None or h is None else float(first - h)
        ),
        "first5_joint_rmse_deg": np.rad2deg(
            np.sqrt(np.mean((q[x["t"] < 5] - qr[x["t"] < 5]) ** 2, axis=0))
        ).tolist(),
        "last2_raw_peak_N": float(tail_raw.max()),
        "last2_raw_in_band_pct": float(np.mean(np.abs(tail_raw - 5) <= 0.5) * 100),
        "contact_loss_fraction_after_1s": (
            None if h is None else float(np.mean(b[t > h + 1] < 0.1))
        ),
        "time_audit": json.loads((d / "time_audit.json").read_text()),
        "optimization": json.loads((d / "metadata.json").read_text())["optimization"],
    }


if __name__ == "__main__":
    out = {
        d.name: metrics(d)
        for d in sorted((R / "runs").iterdir())
        if d.is_dir() and (d / "COMPLETE").exists()
    }
    targets = json.loads((R / "PROTOCOL.json").read_text())[
        "engineering_targets_not_guarantees"
    ]
    for v in out.values():
        v["target_checks"] = {
            "raw_peak": v["raw_peak_N"] <= targets["raw_peak_N_max"],
            "tail_band": v["last2_in_band_pct"] >= targets["last2_in_band_pct_min"],
            "tail_rmse": v["last2_rmse_N"] <= targets["last2_rmse_N_max"],
            "tracking": max(v["first5_joint_rmse_deg"])
            <= targets["first5_joint_rmse_deg_each_max"],
        }
        v["all_targets_met"] = all(v["target_checks"].values())
    (R / "results.json").write_text(json.dumps(out, indent=2))
    for k, v in out.items():
        print(
            k,
            "peak",
            round(v["raw_peak_N"], 2),
            "mean",
            round(v["last2_mean_N"], 3),
            "rmse",
            round(v["last2_rmse_N"], 3),
            "band",
            v["last2_in_band_pct"],
        )
