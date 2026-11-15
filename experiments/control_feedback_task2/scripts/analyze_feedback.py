"""Separate transport sequence loss, intentional decimation and repeated samples."""

from pathlib import Path
import json, csv, argparse
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

T = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--out", type=Path, default=T / "runs/feedback001")
args = parser.parse_args()
out = args.out.resolve()
rows = []
for run in sorted(out.glob("source*_feedback*")):
    d = np.genfromtxt(run / "sink.csv", delimiter=",", names=True)
    relay = np.genfromtxt(run / "relay.csv", delimiter=",", names=True)
    source = np.genfromtxt(run / "source.csv", delimiter=",", names=True)
    assert len(d) > 500
    # Use the central steady portion; exclude startup and the source-stop grace interval.
    t = (d["sink_receive_ns"] - d["sink_receive_ns"][0]) * 1e-9
    mask = (t >= 1) & (t < 29)
    v = d[mask]
    intervals = np.diff(v["sink_receive_ns"]) * 1e-6
    transport = (v["sink_receive_ns"] - v["relay_publish_ns"]) * 1e-6
    age = (v["sink_receive_ns"] - v["source_capture_ns"]) * 1e-6
    src_hz = int(run.name.split("_")[0][6:])
    fb_hz = int(run.name.split("feedback")[1])
    gaps = np.diff(d["publication_sequence"]) - 1
    row = dict(
        source_hz=src_hz,
        feedback_target_hz=fb_hz,
        source_samples=len(source),
        source_rate_hz=float(
            (len(source) - 1)
            / (
                (
                    source["captured_monotonic_ns"][-1]
                    - source["captured_monotonic_ns"][0]
                )
                * 1e-9
            )
        ),
        relay_published=len(relay),
        sink_received=len(d),
        transport_lost_between_first_last=int(np.maximum(gaps, 0).sum()),
        lost_total_vs_publisher=int(len(relay) - len(d)),
        received_imu=json.loads((run / "sink_summary.json").read_text())[
            "received_imu"
        ],
        mean_feedback_hz=float(1000 / intervals.mean()),
        interval_std_ms=float(intervals.std(ddof=1)),
        interval_p95_ms=float(np.percentile(intervals, 95)),
        source_age_p50_ms=float(np.median(age)),
        source_age_p95_ms=float(np.percentile(age, 95)),
        transport_p50_ms=float(np.median(transport)),
        transport_p95_ms=float(np.percentile(transport, 95)),
        duplicate_source_fraction=float(np.mean(np.diff(v["source_sequence"]) == 0)),
        intentional_skipped_source_samples=int(
            np.maximum(np.diff(v["source_sequence"]) - 1, 0).sum()
        ),
        filter_theoretical_delay_ms=2000 / src_hz,
    )
    live = "live physical" in json.loads((run / "source_summary.json").read_text()).get("source", "")
    row["source_kind"] = "physical IMU" if live else "public data replay"
    if live:
        row["filter_theoretical_delay_ms"] = 2000 / row["source_rate_hz"]
        row["source_samples_not_observed_at_sink"] = row.pop("intentional_skipped_source_samples")
    rows.append(row)
with (out / "metrics.csv").open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=rows[0].keys())
    w.writeheader()
    w.writerows(rows)
(out / "analysis.json").write_text(json.dumps(rows, indent=2))
fig, axs = plt.subplots(1, 3, figsize=(13, 4))
for sh in [200, 25]:
    rr = sorted(
        [r for r in rows if r["source_hz"] == sh], key=lambda x: x["feedback_target_hz"]
    )
    x = [r["feedback_target_hz"] for r in rr]
    for ax, key in zip(
        axs, ["source_age_p95_ms", "duplicate_source_fraction", "interval_std_ms"]
    ):
        ax.plot(x, [r[key] for r in rr], "o-", label=f"Source {sh}Hz")
        ax.set_xlabel("Feedback target [Hz]")
        ax.set_ylabel(key)
        ax.grid(alpha=0.2)
axs[0].legend()
fig.tight_layout()
fig.savefig(out / "feedback_rates.png", dpi=160)
plt.close(fig)
print(json.dumps(rows, indent=2))
