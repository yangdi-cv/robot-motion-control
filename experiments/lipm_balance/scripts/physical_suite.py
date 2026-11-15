"""Finite-foot contact validation; separate counts from reduced-order stepping."""

from pathlib import Path
from dataclasses import asdict
from concurrent.futures import ProcessPoolExecutor, as_completed
import json, hashlib, shutil, time
from physical import PhysicalConfig, simulate_physical

R = Path(__file__).resolve().parents[1]


def worker(job):
    cfg, out, source = job
    out = Path(out)
    if (out / "result.json").exists():
        return json.loads((out / "result.json").read_text())
    result = simulate_physical(cfg, out)
    result["source_sha256"] = source
    (out / "result.json").write_text(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    source = hashlib.sha256(
        (R / "scripts/physical.py").read_bytes() + (R / "models/ankle.xml").read_bytes()
    ).hexdigest()
    root = R / "runs" / ("physical_" + source[:10])
    root.mkdir(exist_ok=True)
    shutil.copy2(R / "scripts/physical.py", root / "source.py")
    cfgs = []
    for method in ["pd", "lqr"]:
        for hz in [50, 100, 200]:
            for delay in [0.0, 0.04, 0.08, 0.12]:
                for force in [0.0, 40.0, 80.0, 120.0]:
                    for seed in [0, 1, 2]:
                        cfgs.append(
                            PhysicalConfig(
                                controller=method,
                                hz=hz,
                                delay=delay,
                                force=force,
                                seed=seed,
                                noise=0.002,
                            )
                        )
        for friction in [0.1, 0.4]:
            for force in [0.0, 40.0, 80.0]:
                for seed in [10, 11, 12]:
                    cfgs.append(
                        PhysicalConfig(
                            controller=method,
                            friction=friction,
                            force=force,
                            seed=seed,
                            noise=0.002,
                        )
                    )
        for dt in [0.001, 0.0005]:
            for force in [0.0, 80.0]:
                cfgs.append(PhysicalConfig(controller=method, dt=dt, force=force))
    cfgs.append(PhysicalConfig(controller="none", force=0.0))
    results = {}
    start = time.monotonic()
    manifest = {}
    with ProcessPoolExecutor(max_workers=4) as pool:
        jobs = {}
        for cfg in cfgs:
            key = hashlib.sha256(
                json.dumps(asdict(cfg), sort_keys=True).encode()
            ).hexdigest()[:16]
            manifest[key] = asdict(cfg)
            jobs[pool.submit(worker, (cfg, str(root / key), source))] = key
        for i, f in enumerate(as_completed(jobs), 1):
            results[jobs[f]] = f.result()
            if i % 40 == 0:
                print(
                    i,
                    "/",
                    len(jobs),
                    "elapsed",
                    round(time.monotonic() - start, 1),
                    flush=True,
                )
    (R / "physical_results.json").write_text(json.dumps(results, indent=2))
    (R / "physical_manifest.json").write_text(
        json.dumps(
            {
                "root": str(root.relative_to(R)),
                "source_sha256": source,
                "configurations": manifest,
            },
            indent=2,
        )
    )
    print("DONE", len(results), flush=True)
