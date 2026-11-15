"""Frozen 3-D condition matrix, including backend ablations and failure boundaries."""

from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
import hashlib, json, shutil, time
from pathlib import Path
from model import R, build
from run import Config, simulate
from storage import atomic_json


def key(c):
    config = {
        name: float(value) if isinstance(value, (int, float)) else value
        for name, value in asdict(c).items()
    }
    # A disabled perturbation has no meaningful onset time; do not count it twice.
    if not config["push_x"] and not config["push_y"]:
        config["push_time"] = 0.0
    return hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()[:16]


def protocol():
    groups = {
        name: []
        for name in [
            "standing",
            "transfer",
            "single_support",
            "one_step",
            "walking",
            "friction",
            "heading",
            "backend",
        ]
    }
    for hz in [50, 100, 200]:
        for x, y in [(0, 0), (40, 0), (-40, 0), (0, 40), (0, -40), (0, 60)]:
            groups["standing"].append(Config(hz=hz, push_x=x, push_y=y))
    for shift in [-0.06, 0.06]:
        for force in [0, 20, -20]:
            groups["transfer"].append(
                Config(scenario="shift", shift=shift, push_y=force)
            )
    for support in [0, 1]:
        for hz in [100, 200]:
            for force in [0, 10, -10]:
                groups["single_support"].append(
                    Config(
                        scenario="single",
                        duration=8,
                        hz=hz,
                        support_side=support,
                        push_y=force,
                        push_time=3.0,
                    )
                )
    for support in [0, 1]:
        for distance in [0.06, 0.10]:
            groups["one_step"].append(
                Config(
                    scenario="step",
                    duration=8,
                    support_side=support,
                    step_distance=distance,
                )
            )
    for hz in [100, 200]:
        for distance in [0.04, 0.06]:
            groups["walking"].append(
                Config(scenario="walk", duration=29, hz=hz, step_distance=distance)
            )
    groups["walking"].append(
        Config(scenario="walk", duration=29, push_y=10, push_time=10.0)
    )
    groups["walking"].append(Config(scenario="walk", duration=43, walk_steps=6))
    groups["friction"] = [
        Config(friction=0.3, push_y=40),
        Config(scenario="step", duration=8, friction=0.3),
    ]
    for yaw in [-0.3, 0.3]:
        groups["heading"].append(Config(initial_yaw=yaw, push_y=20))
    for backend in ["osqp", "clarabel"]:
        for scenario, duration in [("single", 8), ("walk", 29)]:
            groups["backend"].append(
                Config(scenario=scenario, duration=duration, backend=backend)
            )
    return groups


def worker(job):
    cfg, out, source = job
    out = Path(out)
    if (out / "result.json").exists():
        r = json.loads((out / "result.json").read_text())
        if r.get("source_sha256") != source:
            raise RuntimeError(f"Unfinished or mismatched case {out}")
        return r
    r = simulate(cfg, out)
    r["source_sha256"] = source
    atomic_json(out / "result.json", r)
    return r


def main():
    build()
    files = [
        "scripts/model.py",
        "scripts/controller.py",
        "scripts/qp.py",
        "scripts/run.py",
        "scripts/metrics.py",
        "scripts/storage.py",
        "scripts/suite.py",
        "models/g1_torque.xml",
        "models/scene.xml",
        "research/model_source.json",
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
        groups={k: [key(c) for c in cs] for k, cs in groups.items()},
        configs={k: asdict(c) for k, c in cases.items()},
        scope="Ideal state feedback, 6-DoF floating base, 29 torque motors. Quasi-static walking, not dynamic locomotion.",
    )
    atomic_json(R / "manifest.json", manifest)
    results = {}
    start = time.monotonic()
    with ProcessPoolExecutor(max_workers=3) as pool:
        pending = {
            pool.submit(worker, (c, str(root / k), source)): k for k, c in cases.items()
        }
        for f in as_completed(pending):
            results[pending[f]] = f.result()
            atomic_json(R / "results.json", results)
            r = results[pending[f]]
            print(
                len(results),
                "/",
                len(cases),
                r["config"]["scenario"],
                r["config"]["backend"],
                r["success"],
                r["failure"],
                "elapsed",
                round(time.monotonic() - start, 1),
                flush=True,
            )
    print("DONE", len(results), flush=True)


if __name__ == "__main__":
    main()
