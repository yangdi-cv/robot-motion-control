"""Frozen controller experiments with separate grid and validation seeds."""

from pathlib import Path
from dataclasses import asdict
from concurrent.futures import ProcessPoolExecutor, as_completed
import json, hashlib, shutil, os, time
from core import Config, simulate, key, CONTROLLERS

R = Path(__file__).resolve().parents[1]


def worker(item):
    cfg, directory, source_sha = item
    directory = Path(directory)
    if (directory / "result.json").exists():
        data = json.loads((directory / "result.json").read_text())
        if data.get("source_sha256") == source_sha:
            return data
        raise RuntimeError(f"Source mismatch {directory}")
    data, _ = simulate(cfg, directory)
    data["source_sha256"] = source_sha
    (directory / "result.json").write_text(json.dumps(data, indent=2))
    return data


def conditions():
    groups = {
        "grid": [],
        "boundary": [],
        "domain": [],
        "numerical": [],
        "showcases": [],
        "phase_validation": [],
        "gait_validation": [],
    }
    for method in CONTROLLERS:
        for hz in [50, 100, 200]:
            for delay in [0.0, 0.02, 0.04, 0.08, 0.12]:
                for impulse in [0.0, 6.0, 12.0]:
                    for seed in [0, 1, 2]:
                        groups["grid"].append(
                            Config(
                                controller=method,
                                hz=hz,
                                delay=delay,
                                impulse_x=impulse,
                                seed=seed,
                            )
                        )
    # Additional disturbance levels and held-out noise seeds; never used to tune gains.
    for method in CONTROLLERS:
        for delay in [0.0, 0.04, 0.08, 0.12]:
            for impulse in [0.0, 3.0, 6.0, 9.0, 12.0, 15.0]:
                for seed in [20, 21, 22]:
                    groups["boundary"].append(
                        Config(
                            controller=method, delay=delay, impulse_x=impulse, seed=seed
                        )
                    )
    for method in CONTROLLERS:
        for height in [0.72, 0.88]:
            for px, py in [(6.0, 0.0), (0.0, 4.0), (-6.0, 0.0)]:
                for seed in [30, 31, 32]:
                    groups["domain"].append(
                        Config(
                            controller=method,
                            height=height,
                            impulse_x=px,
                            impulse_y=py,
                            seed=seed,
                        )
                    )
        for mu in [0.2, 0.4]:
            for seed in [30, 31, 32]:
                groups["domain"].append(
                    Config(controller=method, friction=mu, impulse_x=12.0, seed=seed)
                )
        for noise in [0.003, 0.005]:
            for seed in [30, 31, 32]:
                groups["domain"].append(
                    Config(
                        controller=method,
                        noise_position=noise,
                        noise_velocity=noise * 10,
                        impulse_x=6.0,
                        delay=0.04,
                        seed=seed,
                    )
                )
        for bias in [-0.01, 0.01]:
            groups["domain"].append(
                Config(controller=method, bias_position=bias, impulse_x=6.0, seed=33)
            )
        for dt in [0.001, 0.0005]:
            groups["numerical"].append(
                Config(
                    controller=method,
                    dt=dt,
                    impulse_x=12.0,
                    noise_position=0.0,
                    noise_velocity=0.0,
                )
            )
    for method in ["open_loop", *CONTROLLERS]:
        for impulse in [0.0, 6.0, 12.0]:
            groups["showcases"].append(
                Config(
                    controller=method,
                    impulse_x=impulse,
                    noise_position=0.0,
                    noise_velocity=0.0,
                )
            )
    groups["showcases"] += [
        Config(controller=m, delay=0.08, impulse_x=12.0, seed=1)
        for m in ["capture", "capture_predict"]
    ]
    for method in CONTROLLERS:
        for delay in [0.0, 0.08]:
            for impulse in [6.0, 12.0]:
                for push_time in [2.45, 2.70, 2.90]:
                    for seed in [40, 41, 42]:
                        groups["phase_validation"].append(
                            Config(
                                controller=method,
                                delay=delay,
                                impulse_x=impulse,
                                push_time=push_time,
                                seed=seed,
                            )
                        )
        for duration in [0.5, 0.7]:
            for delay in [0.0, 0.08]:
                for seed in [40, 41, 42]:
                    groups["gait_validation"].append(
                        Config(
                            controller=method,
                            delay=delay,
                            impulse_x=12.0,
                            step_time=duration,
                            seed=seed,
                        )
                    )
    return groups


if __name__ == "__main__":
    source_sha = hashlib.sha256((R / "scripts/core.py").read_bytes()).hexdigest()
    base = R / "runs" / ("lipm_" + source_sha[:10])
    base.mkdir(parents=True, exist_ok=True)
    shutil.copy2(R / "scripts/core.py", base / "source.py")
    groups = conditions()
    unique = {key(c): c for values in groups.values() for c in values}
    manifest = {
        "source_sha256": source_sha,
        "root": str(base.relative_to(R)),
        "groups": {g: [key(c) for c in cs] for g, cs in groups.items()},
        "unique_count": len(unique),
        "configurations": {k: asdict(c) for k, c in unique.items()},
    }
    (R / "manifest.json").write_text(json.dumps(manifest, indent=2))
    start = time.monotonic()
    results = {}
    with ProcessPoolExecutor(max_workers=6) as pool:
        futures = {
            pool.submit(worker, (cfg, str(base / k), source_sha)): k
            for k, cfg in unique.items()
        }
        for i, f in enumerate(as_completed(futures), 1):
            results[futures[f]] = f.result()
            if i % 50 == 0:
                print(
                    i,
                    "/",
                    len(unique),
                    "elapsed",
                    round(time.monotonic() - start, 1),
                    flush=True,
                )
    (R / "results.json").write_text(json.dumps(results, indent=2))
    (R / "suite_summary.json").write_text(
        json.dumps(
            {
                "unique_runs": len(results),
                "wall_seconds": time.monotonic() - start,
                "group_counts": {g: len(cs) for g, cs in groups.items()},
                "successes": sum(x["success"] for x in results.values()),
            },
            indent=2,
        )
    )
    print("DONE", len(results), flush=True)
