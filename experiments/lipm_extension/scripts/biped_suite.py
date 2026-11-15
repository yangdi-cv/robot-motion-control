"""Distinct physical conditions; no random-seed duplicates for an ideal observer."""

from pathlib import Path
from dataclasses import asdict
from concurrent.futures import ProcessPoolExecutor, as_completed
import json, hashlib, shutil, time
from biped import BipedConfig, simulate

R = Path(__file__).resolve().parents[1]


def key(c):
    return hashlib.sha256(json.dumps(asdict(c), sort_keys=True).encode()).hexdigest()[
        :16
    ]


def worker(job):
    cfg, path, source = job
    path = Path(path)
    if (path / "result.json").exists():
        return json.loads((path / "result.json").read_text())
    result = simulate(cfg, path)
    result["source_sha256"] = source
    (path / "result.json").write_text(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    files = ["scripts/biped.py", "scripts/trajectory.py", "models/biped.xml"]
    source = hashlib.sha256(b"".join((R / n).read_bytes() for n in files)).hexdigest()
    root = R / "runs" / ("biped_" + source[:10])
    root.mkdir(exist_ok=True)
    for n in files:
        shutil.copy2(R / n, root / Path(n).name)
    groups = {
        "walking": [],
        "friction": [],
        "torque": [],
        "support": [],
        "numerical": [],
        "standing": [],
    }
    for step in [0.08, 0.12, 0.16]:
        for hz in [50, 100, 200]:
            for force in [0.0, 20.0, 40.0]:
                groups["walking"].append(
                    BipedConfig(step_length=step, hz=hz, push=force)
                )
    for mu in [0.2, 0.4]:
        for force in [0.0, 20.0, 40.0]:
            groups["friction"].append(BipedConfig(friction=mu, push=force))
    for scale in [0.5, 0.75]:
        for force in [0.0, 20.0, 40.0]:
            groups["torque"].append(BipedConfig(torque_scale=scale, push=force))
    for ds in [0.0, 0.06, 0.12, 0.20]:
        for force in [0.0, 40.0]:
            groups["support"].append(BipedConfig(ds=ds, push=force))
    for force in [0.0, 40.0]:
        groups["numerical"].append(BipedConfig(dt=0.0005, push=force))
    for scale in [0.5, 1.0]:
        for force in [0.0, 40.0]:
            groups["standing"].append(
                BipedConfig(walk=False, torque_scale=scale, push=force)
            )
    cases = {key(c): c for cs in groups.values() for c in cs}
    (R / "biped_manifest.json").write_text(
        json.dumps(
            {
                "source_sha256": source,
                "source_files": files,
                "root": str(root.relative_to(R)),
                "groups": {g: [key(c) for c in cs] for g, cs in groups.items()},
                "configurations": {k: asdict(c) for k, c in cases.items()},
            },
            indent=2,
        )
    )
    results = {}
    start = time.monotonic()
    with ProcessPoolExecutor(max_workers=3) as pool:
        jobs = {
            pool.submit(worker, (c, str(root / k), source)): k for k, c in cases.items()
        }
        for i, f in enumerate(as_completed(jobs), 1):
            results[jobs[f]] = f.result()
            if i % 5 == 0:
                print(
                    i,
                    "/",
                    len(jobs),
                    "elapsed",
                    round(time.monotonic() - start, 1),
                    flush=True,
                )
    (R / "biped_results.json").write_text(json.dumps(results, indent=2))
    print("DONE", len(results), flush=True)
