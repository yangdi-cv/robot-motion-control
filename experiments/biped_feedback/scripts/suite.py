"""Frozen condition matrix; outputs are keyed by full config and source hash."""

from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import time
from biped import BipedConfig, simulate

R = Path(__file__).resolve().parents[1]


def key(config):
    return hashlib.sha256(
        json.dumps(asdict(config), sort_keys=True).encode()
    ).hexdigest()[:16]


def protocol():
    groups = {
        name: []
        for name in [
            "mapping",
            "feedback",
            "support",
            "slew",
            "integration",
            "long_walk",
        ]
    }
    for hz in [50, 100, 200]:
        for push in [0.0, 40.0, -40.0]:
            for mode in ["legacy", "coupled"]:
                groups["mapping"].append(BipedConfig(hz=hz, push=push, mapping=mode))
    for hz in [50, 100, 200]:
        for delay in [0.02, 0.04, 0.08]:
            for mode in ["hold", "predict"]:
                for seed in [11, 29, 47]:
                    groups["feedback"].append(
                        BipedConfig(
                            hz=hz,
                            push=40.0,
                            feedback=mode,
                            delay=delay,
                            position_noise=0.001,
                            velocity_noise=0.01,
                            seed=seed,
                        )
                    )
    for ds in [0.0, 0.06, 0.12, 0.20]:
        for push in [0.0, 40.0]:
            groups["support"].append(BipedConfig(ds=ds, push=push))
    for rate in [600.0, 1500.0]:
        for hz in [50, 100, 200]:
            for push in [0.0, 40.0]:
                groups["slew"].append(BipedConfig(torque_rate=rate, hz=hz, push=push))
    for push in [0.0, 40.0]:
        groups["integration"].append(BipedConfig(dt=0.0005, push=push))
        groups["long_walk"].append(BipedConfig(steps=20, duration=14.0, push=push))
    return groups


def worker(job):
    config, path, source = job
    path = Path(path)
    if (path / "result.json").exists():
        result = json.loads((path / "result.json").read_text())
        if result.get("source_sha256") != source:
            raise RuntimeError("Source mismatch")
        return result
    result = simulate(config, path)
    result["source_sha256"] = source
    (path / "result.json").write_text(json.dumps(result, indent=2))
    return result


def main():
    files = [
        "scripts/biped.py",
        "scripts/feedback.py",
        "scripts/trajectory.py",
        "scripts/suite.py",
        "models/biped.xml",
    ]
    hashes = {
        name: hashlib.sha256((R / name).read_bytes()).hexdigest() for name in files
    }
    source = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()
    root = R / "runs" / source[:12]
    root.mkdir(parents=True, exist_ok=True)
    for name in files:
        dest = root / "source" / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(R / name, dest)
    groups = protocol()
    cases = {key(c): c for cs in groups.values() for c in cs}
    manifest = dict(
        source_sha256=source,
        files=hashes,
        root=str(root.relative_to(R)),
        groups={name: [key(c) for c in cs] for name, cs in groups.items()},
        configs={k: asdict(c) for k, c in cases.items()},
        note="Explicit deterministic physical conditions; seeds only repeated for noisy feedback. Failed conditions retained.",
    )
    (R / "manifest.json").write_text(json.dumps(manifest, indent=2))
    results = {}
    start = time.monotonic()
    with ProcessPoolExecutor(max_workers=3) as pool:
        jobs = {
            pool.submit(worker, (c, str(root / k), source)): k for k, c in cases.items()
        }
        for f in as_completed(jobs):
            result = f.result()
            results[jobs[f]] = result
            print(
                len(results),
                "/",
                len(jobs),
                result["success"],
                result["failure"],
                "elapsed",
                round(time.monotonic() - start, 1),
                flush=True,
            )
            tmp = R / "results.tmp"
            tmp.write_text(json.dumps(results, indent=2))
            tmp.replace(R / "results.json")
    print("DONE", len(results), flush=True)


if __name__ == "__main__":
    main()
