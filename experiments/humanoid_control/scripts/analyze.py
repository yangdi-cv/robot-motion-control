"""Summarize frozen 3-D physical trials without rerunning controllers."""

import csv
import json
import numpy as np
from model import R


def main():
    m = json.loads((R / "manifest.json").read_text())
    root = R / m["root"]
    cases = {}
    for key, cfg in m["configs"].items():
        path = root / key
        r = json.loads((path / "result.json").read_text())
        a = np.load(path / "trace.npz")
        c = np.load(path / "controller.npz")
        time = a["time"]
        controltime = c["time"]
        r["id"] = key
        r["whole_body_p50_ms"] = float(np.quantile(c["solve_seconds"], 0.5) * 1000)
        r["whole_body_p99_ms"] = float(np.quantile(c["solve_seconds"], 0.99) * 1000)
        r["whole_body_max_ms"] = float(c["solve_seconds"].max() * 1000)
        r["deadline_misses"] = int(np.sum(c["solve_seconds"] > 1 / cfg["hz"]))
        r["max_normalized_constraint_violation"] = float(
            c["normalized_constraint_violation"].max()
        )
        force = a["actual_wrench"][:, :, :3]
        moment = a["actual_wrench"][:, :, 3:] + np.cross(a["feet"], force)
        total = force[:, :, 2].sum(axis=1)
        cop = np.c_[
            -moment[:, :, 1].sum(axis=1), moment[:, :, 0].sum(axis=1)
        ] / np.maximum(total[:, None], 1)
        indices = np.searchsorted(time, controltime)
        mask = (controltime >= 0.5) & (total[indices] > 20)
        r["cop_rmse_mm"] = float(
            np.sqrt(np.mean((cop[indices][mask] - c["cop_command"][mask]) ** 2)) * 1000
        )
        error = np.linalg.norm(
            c["com"][:, :2] - a["reference_com"][indices, :2], axis=1
        )
        velocity = np.linalg.norm(c["vcom"][:, :2], axis=1)
        r["standing_recovery_s"] = None
        if cfg["scenario"] == "stand" and (cfg["push_x"] or cfg["push_y"]):
            good = (error < 0.01) & (velocity < 0.05)
            window = int(np.ceil(0.3 * cfg["hz"]))
            end = cfg["push_time"] + cfg["push_duration"]
            for i in np.flatnonzero(controltime >= end):
                if i + window <= len(good) and good[i : i + window].all():
                    r["standing_recovery_s"] = round(float(controltime[i] - end), 6)
                    break
        cases[key] = r
    summary = dict(
        n=len(cases),
        passed=sum(c["success"] for c in cases.values()),
        groups={},
        cases=cases,
    )
    for name, keys in m["groups"].items():
        summary["groups"][name] = dict(
            n=len(set(keys)), passed=sum(cases[k]["success"] for k in set(keys))
        )
    (R / "summary.json").write_text(json.dumps(summary, indent=2))
    columns = [
        "id",
        "scenario",
        "backend",
        "hz",
        "push_x",
        "push_y",
        "support_side",
        "step_distance",
        "walk_steps",
        "success",
        "failure",
        "duration",
        "forward_progress",
        "landings",
        "single_support_seconds",
        "com_rmse",
        "max_roll_pitch",
        "qp_p95_ms",
        "whole_body_p99_ms",
        "whole_body_max_ms",
        "deadline_misses",
        "cop_rmse_mm",
        "standing_recovery_s",
    ]
    with (R / "metrics.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for key, r in cases.items():
            data = {**r, **r["config"]}
            writer.writerow({name: data.get(name) for name in columns})
    print(json.dumps({k: v for k, v in summary.items() if k != "cases"}, indent=2))


if __name__ == "__main__":
    main()
