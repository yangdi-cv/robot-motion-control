"""Eight prespecified stronger perturbations; no controller tuning on these cases."""

from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
import json, hashlib
from model import R
from run import Config
from suite import key, worker
from storage import atomic_json


def main():
    baseline = json.loads((R / "manifest.json").read_text())
    source = baseline["source_sha256"]
    configs = [Config(push_y=force) for force in [150, 250]]
    configs += [
        Config(scenario="single", duration=8, push_y=force, push_time=3.0)
        for force in [-50, -30, 30, 50]
    ]
    configs += [
        Config(scenario="walk", duration=29, push_y=force, push_time=10.0)
        for force in [-40, 40]
    ]
    root = R / baseline["root"] / "stress"
    root.mkdir(exist_ok=True)
    manifest = dict(
        source_sha256=source,
        protocol_sha256=hashlib.sha256(
            __import__("pathlib").Path(__file__).read_bytes()
        ).hexdigest(),
        root=str(root.relative_to(R)),
        configs={key(c): asdict(c) for c in configs},
        scope="Stronger held-out perturbations; controller unchanged; all failures retained",
    )
    atomic_json(R / "stress_manifest.json", manifest)
    results = {}
    with ProcessPoolExecutor(max_workers=3) as pool:
        jobs = {
            pool.submit(worker, (c, str(root / key(c)), source)): key(c)
            for c in configs
        }
        for future in as_completed(jobs):
            r = future.result()
            results[jobs[future]] = r
            atomic_json(R / "stress_results.json", results)
            print(
                len(results),
                r["config"]["scenario"],
                r["config"]["push_y"],
                r["success"],
                r["failure"],
                flush=True,
            )


if __name__ == "__main__":
    main()
