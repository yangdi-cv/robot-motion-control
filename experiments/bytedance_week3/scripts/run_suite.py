from pathlib import Path
import json, csv, itertools, numpy as np
from contact_task import run

T = Path(__file__).resolve().parents[1]
O = T / "runs/contact_suite_v3"
O.mkdir(exist_ok=True)
configs = []
for k in [2000, 8000, 20000]:
    for mode, K in [("position", 0), ("admittance", 0), ("admittance_spring", 200)]:
        configs.append(dict(group="core", mode=mode, K=K, stiffness=k))
for k, rate, delay, seed, mode in itertools.product(
    [2000, 8000, 20000],
    [50, 100, 200],
    [0, 20, 60],
    range(3),
    ["position", "admittance"],
):
    configs.append(
        dict(
            group="stress",
            mode=mode,
            stiffness=k,
            rate=rate,
            delay_ms=delay,
            noise_std=0.5,
            seed=seed,
        )
    )
for duration, mode in itertools.product([1.0, 2.0, 4.0], ["position", "admittance"]):
    configs.append(dict(group="approach", mode=mode, approach_s=duration))
(T / "config/suite.json").write_text(
    json.dumps(
        {
            "conditions": configs,
            "plant_dt_s": 0.001,
            "admittance_M": 2,
            "admittance_B": 100,
            "desired_force_N": 10,
            "torque_limit_Nm": [87] * 4 + [12] * 3,
            "offset_limit_m": [-0.025, 0.05],
            "offset_velocity_limit_m_s": 0.04,
            "contact_geometries": "probe/surface only; other robot geoms non-colliding",
        },
        indent=2,
    )
)
results = []
for index, config in enumerate(configs):
    path = O / f"case_{index:03d}.json"
    cfg = dict(config)
    group = cfg.pop("group")
    if path.exists():
        s = json.loads(path.read_text())
    else:
        a, s = run(**cfg)
        s.update(group=group, case_id=index)
        np.savez_compressed(O / f"case_{index:03d}.npz", samples=a)
        path.write_text(json.dumps(s, indent=2))
    results.append(s)
    if index % 20 == 0:
        print("done", index, s["steady_force_rmse_N"], flush=True)
with (O / "metrics.csv").open("w", newline="") as f:
    keys = list(results[0])
    w = csv.DictWriter(f, fieldnames=keys)
    w.writeheader()
    w.writerows(results)
(O / "summary.json").write_text(
    json.dumps({"cases": len(results), "results": results}, indent=2)
)
# Actual rendered trajectories and a causal physical-contact check.
for mode in ["position", "admittance"]:
    a, s = run(mode, video=O / (mode + ".mp4"))
    (O / (mode + "_video.json")).write_text(json.dumps(s, indent=2))
a, free = run("position", disable_contacts=True)
(O / "no_contact_check.json").write_text(json.dumps(free, indent=2))
assert free["peak_force_N"] == 0
print("SUITE COMPLETE", len(results), flush=True)
