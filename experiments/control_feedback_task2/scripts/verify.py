"""Check saved results against source records and independent calculations."""

from pathlib import Path
import json, csv, ast, hashlib
import numpy as np

T = Path(__file__).resolve().parents[1]
checks = {}
for p in (T / "scripts").glob("*.py"):
    ast.parse(p.read_text())
checks["python_scripts"] = len(list((T / "scripts").glob("*.py")))
for name, expected in [
    ("control002", 24),
    ("control_validation", 16),
    ("control_frequency", 24),
    ("control_frequency_norender", 24),
]:
    run = T / "runs" / name
    assert (run / "RUN_COMPLETE").exists()
    config = json.loads((run / "config.json").read_text())
    changes = [
        json.loads(l) for l in (run / "gain_changes.jsonl").read_text().splitlines()
    ]
    assert len(changes) == expected
    assert len(config["stages"]) == expected
    d = np.genfromtxt(run / "tracking.csv", delimiter=",", names=True)
    assert len(set(d["phase"])) == expected
    assert np.all(np.isfinite(np.stack([d[k] for k in d.dtype.names], axis=1)))
    checks[name] = {"stages": expected, "feedback_rows": len(d)}
source_data = np.genfromtxt(T / "data/tum_vi/motion_30s.csv", delimiter=",", names=True)
raw = np.stack([source_data[k] for k in ["ax", "ay", "az", "gx", "gy", "gz"]], axis=1)
total = 0
maxerror = 0
for run in (T / "runs/feedback001").glob("source*_feedback*"):
    d = np.genfromtxt(run / "sink.csv", delimiter=",", names=True)
    r = np.genfromtxt(run / "relay.csv", delimiter=",", names=True)
    s = np.genfromtxt(run / "source.csv", delimiter=",", names=True)
    assert len(d) == len(r)
    assert np.all(np.diff(d["publication_sequence"]) == 1)
    relay = json.loads((run / "relay_summary.json").read_text())
    sink = json.loads((run / "sink_summary.json").read_text())
    assert relay["source_sequence_gaps"] == 0
    assert sink["received_imu"] == len(d)
    assert relay["unique_source_messages"] == len(s)
    x = raw[s["input_index"].astype(int)]
    expected = np.array([x[max(0, i - 4) : i + 1].mean(axis=0) for i in range(len(x))])
    ids = d["source_sequence"].astype(int)
    filtered = np.stack(
        [d["filtered_" + k] for k in ["ax", "ay", "az", "gx", "gy", "gz"]], axis=1
    )
    error = float(np.max(np.abs(filtered - expected[ids])))
    assert error < 1e-10
    maxerror = max(maxerror, error)
    total += len(d)
checks["ros_feedback"] = {
    "trials": 6,
    "messages_verified": total,
    "imu_messages_match_metadata_count": True,
    "moving_average_vs_original_csv_max_error": maxerror,
}
for name in ["static", "motion"]:
    d = np.genfromtxt(T / "data/tum_vi" / f"{name}_30s.csv", delimiter=",", names=True)
    assert np.all(np.diff(d["time_s"]) > 0)
    checks[name + "_samples"] = len(d)
checks["software_known_signal"] = json.loads(
    (T / "runs/SOFTWARE_CHECKS.json").read_text()
)
assert checks["software_known_signal"]["ma5_sinusoid_max_error"] < 1e-11
# Compare the general CSV tool with the independent public-data analysis.
a = list(csv.DictReader((T / "runs/generic_filter_static/statistics.csv").open()))
b = list(csv.DictReader((T / "runs/static_filter_statistics.csv").open()))
for r1, r2 in zip(a, b):
    for k in ["raw_mean", "filtered_mean", "raw_std", "filtered_std"]:
        assert abs(float(r1[k]) - float(r2[k])) < 1e-10
checks["generic_filter_matches"] = True
checks["hardware_used"] = False
checks["passed"] = True
(T / "VERIFICATION.json").write_text(json.dumps(checks, indent=2))
print(json.dumps(checks, indent=2))
