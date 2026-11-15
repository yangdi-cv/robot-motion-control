"""Read-only metrics and paired summaries for the frozen physical runs."""

import csv
import json
from pathlib import Path
import numpy as np

R = Path(__file__).resolve().parents[1]


def evaluate(path):
    result = json.loads((path / "result.json").read_text())
    cfg = result["config"]
    a = np.load(path / "audit.npz")
    t = a["time"]
    mask = t >= 0.5
    error = a["com"][:, 0] - a["target_com"][:, 0]
    velocity = a["vcom"][:, 0] - a["target_velocity"][:, 0]
    active = mask & (t <= 0.5 + cfg["steps"] * cfg["step_time"])
    ds = active & a["contact_mask"].all(axis=1) & (cfg["ds"] > 0)
    force = a["measured_wrench"][:, :, 1]
    denominator = force.sum(axis=1)
    load_error = np.abs(
        force[:, 0] / np.maximum(denominator, 1) - a["desired_loads"][:, 0]
    )
    recovery = None
    acceptable = (
        (np.abs(error) < 0.01)
        & (np.abs(velocity) < 0.05)
        & (np.abs(a["qpos"][:, 2]) < 0.03)
    )
    window = int(np.ceil(0.3 * cfg["hz"]))
    for i in np.flatnonzero(t >= 2.8):
        if i + window <= len(t) and acceptable[i : i + window].all():
            recovery = round(max(0, float(t[i] - 2.8)), 6)
            break
    differences = np.diff(a["torque"], axis=0)
    after = (t[1:] >= 0.4) & (t[:-1] >= 0.4)
    slew = np.abs(differences[after]) * cfg["hz"]

    # Exclude the settling interval and report measurement at the control sample,
    # before that sample's new torque has affected the contact solver.
    def rmse(values):
        return float(np.sqrt(np.nanmean(values**2)))

    row = dict(
        success=result["success"],
        failure=result["failure"],
        duration=result["duration_s"],
        progress=result["progress_m"],
        landings=sum(result["verified_landings"]),
        planned_steps=cfg["steps"],
        com_rmse_mm=1000 * rmse(error[mask]),
        com_peak_after_push_mm=1000 * float(np.max(np.abs(error[t >= 2.7])))
        if np.any(t >= 2.7)
        else None,
        cop_measured_rmse_mm=1000 * rmse((a["measured_cop"] - a["cop_command"])[mask]),
        cop_predicted_rmse_mm=1000
        * rmse((a["predicted_cop"] - a["cop_command"])[mask]),
        ds_load_mae=float(np.mean(load_error[ds])) if ds.any() else None,
        torque_step_max_Nm=float(np.max(np.abs(differences[after])))
        if after.any()
        else None,
        torque_rate_max_Nm_s=float(np.max(slew)) if slew.size else None,
        recovery_s=recovery,
        sensor_age_p95_ms=float(np.quantile(a["sensor_age"][mask], 0.95) * 1000),
        terminal_foot_mm=1000 * result["terminal_foot_error_m"],
    )
    return row


def main():
    manifest = json.loads((R / "manifest.json").read_text())
    root = R / manifest["root"]
    result = {
        k: {"config": cfg, **evaluate(root / k)}
        for k, cfg in manifest["configs"].items()
    }
    summary = dict(
        n=len(result),
        passed=sum(x["success"] for x in result.values()),
        groups={},
        cases=result,
    )
    for name, keys in manifest["groups"].items():
        summary["groups"][name] = {
            "n": len(keys),
            "passed": sum(result[k]["success"] for k in keys),
        }
    (R / "summary.json").write_text(json.dumps(summary, indent=2))
    with (R / "metrics.csv").open("w") as f:
        fields = ["id", *next(iter(result.values())).keys()]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for key, row in result.items():
            writer.writerow(dict(id=key, **row))
    print(json.dumps({k: v for k, v in summary.items() if k != "cases"}, indent=2))


if __name__ == "__main__":
    main()
