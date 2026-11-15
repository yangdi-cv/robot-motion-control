"""Packet-limited feedback and constrained double-support stepping experiments."""

from pathlib import Path
from dataclasses import asdict
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib, json, shutil, sys, time
from network import NetworkConfig, run as network_run
from gait import GaitConfig, simulate as gait_run

R = Path(__file__).resolve().parents[1]


def digest(c):
    return hashlib.sha256(json.dumps(asdict(c), sort_keys=True).encode()).hexdigest()[
        :16
    ]


def worker(job):
    kind, cfg, out, source = job
    out = Path(out)
    if (out / "result.json").exists():
        return json.loads((out / "result.json").read_text())
    result = network_run(cfg, out)[0] if kind == "network" else gait_run(cfg, out)
    result["source_sha256"] = source
    (out / "result.json").write_text(json.dumps(result, indent=2))
    return result


def network_conditions():
    groups = {"rates": [], "colored": [], "bursts": []}
    for method in ["capture", "lqr_predict", "capture_predict"]:
        for sensor in [25, 100, 200]:
            for control in [100, 200]:
                for delay in [0.02, 0.08]:
                    for jitter in [0.0, 0.02]:
                        for loss in [0.0, 0.1]:
                            for impulse in [6.0, 12.0]:
                                for seed in [0, 1, 2]:
                                    groups["rates"].append(
                                        NetworkConfig(
                                            controller=method,
                                            hz=control,
                                            sensor_hz=sensor,
                                            delay=delay,
                                            jitter=jitter,
                                            loss=loss,
                                            impulse_x=impulse,
                                            seed=seed,
                                        )
                                    )
        for sensor in [25, 100]:
            for delay in [0.02, 0.08]:
                for impulse in [6.0, 12.0]:
                    for mode in ["white", "colored", "drift"]:
                        for seed in [20, 21, 22]:
                            groups["colored"].append(
                                NetworkConfig(
                                    controller=method,
                                    sensor_hz=sensor,
                                    delay=delay,
                                    impulse_x=impulse,
                                    seed=seed,
                                    color_tau=0.0 if mode == "white" else 0.08,
                                    drift_position=0.005 if mode == "drift" else 0.0,
                                    drift_velocity=0.02 if mode == "drift" else 0.0,
                                )
                            )
        for threshold in [0.15, 0.30, 1.0]:
            for duration in [0.2, 0.4]:
                for impulse in [0.0, 12.0]:
                    for seed in [30, 31, 32]:
                        groups["bursts"].append(
                            NetworkConfig(
                                controller=method,
                                sensor_hz=25,
                                delay=0.04,
                                impulse_x=impulse,
                                seed=seed,
                                max_age=threshold,
                                burst_duration=duration,
                            )
                        )
    return groups


def gait_conditions():
    groups = {"support": [], "swing_limits": [], "late_validation": []}
    for method in ["fixed", "spatial", "timing"]:
        for ds in [0.0, 0.12, 0.20]:
            for impulse in [0.0, 6.0, 12.0, 15.0]:
                for push_time in [2.45, 2.70, 2.90]:
                    for seed in [0, 1, 2]:
                        groups["support"].append(
                            GaitConfig(
                                method=method,
                                ds=ds,
                                impulse_x=impulse,
                                push_time=push_time,
                                seed=seed,
                            )
                        )
        for ds in [0.0, 0.12, 0.20]:
            for speed in [1.2, 1.8, 2.4]:
                for seed in [20, 21, 22]:
                    groups["swing_limits"].append(
                        GaitConfig(method=method, ds=ds, swing_speed=speed, seed=seed)
                    )
        for impulse in [9.0, 15.0]:
            for seed in [30, 31, 32]:
                groups["late_validation"].append(
                    GaitConfig(
                        method=method, impulse_x=impulse, push_time=2.92, seed=seed
                    )
                )
    return groups


if __name__ == "__main__":
    kind = sys.argv[1] if len(sys.argv) > 1 else "network"
    files = (
        ["engine.py", "network.py"]
        if kind == "network"
        else ["gait.py", "trajectory.py"]
    )
    source = hashlib.sha256(
        b"".join((R / "scripts" / n).read_bytes() for n in files)
    ).hexdigest()
    root = R / "runs" / (kind + "_" + source[:10])
    root.mkdir(exist_ok=True)
    for n in files:
        shutil.copy2(R / "scripts" / n, root / n)
    groups = network_conditions() if kind == "network" else gait_conditions()
    cases = {digest(c): c for values in groups.values() for c in values}
    manifest = {
        "source_sha256": source,
        "source_files": files,
        "root": str(root.relative_to(R)),
        "groups": {k: [digest(c) for c in cs] for k, cs in groups.items()},
        "configurations": {k: asdict(v) for k, v in cases.items()},
    }
    (R / f"{kind}_manifest.json").write_text(json.dumps(manifest, indent=2))
    results = {}
    start = time.monotonic()
    with ProcessPoolExecutor(max_workers=6) as pool:
        jobs = {
            pool.submit(worker, (kind, cfg, str(root / key), source)): key
            for key, cfg in cases.items()
        }
        for i, f in enumerate(as_completed(jobs), 1):
            results[jobs[f]] = f.result()
            if i % 30 == 0:
                print(
                    kind,
                    i,
                    "/",
                    len(jobs),
                    "elapsed",
                    round(time.monotonic() - start, 1),
                    flush=True,
                )
    (R / f"{kind}_results.json").write_text(json.dumps(results, indent=2))
    print("DONE", kind, len(results), flush=True)
