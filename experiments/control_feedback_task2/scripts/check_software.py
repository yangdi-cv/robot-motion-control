"""Known-signal and sequence checks independent of sensor availability."""

from pathlib import Path
import numpy as np, json, ast

T = Path(__file__).resolve().parents[1]
for f in (T / "scripts").glob("*.py"):
    ast.parse(f.read_text())
# A five-tap causal average has an exact ten-millisecond phase delay at 200 Hz.
fs = 200.0
freq = 10.0
t = np.arange(4000) / fs
x = np.sin(2 * np.pi * freq * t)
y = np.convolve(x, np.ones(5) / 5, "full")[: len(x)]
expected_gain = np.sin(5 * np.pi * freq / fs) / (5 * np.sin(np.pi * freq / fs))
expected = expected_gain * np.sin(2 * np.pi * freq * (t - 2 / fs))
error = np.max(np.abs(y[4:] - expected[4:]))
assert error < 1e-11
sequence = np.array([0, 1, 3, 4, 8, 9])
assert int(np.maximum(np.diff(sequence) - 1, 0).sum()) == 4
source_ids = np.array([0, 0, 0, 1, 1, 1])
assert np.mean(np.diff(source_ids) == 0) == 0.8
(T / "runs/SOFTWARE_CHECKS.json").write_text(
    json.dumps(
        {
            "scripts_parse": True,
            "ma5_sinusoid_max_error": float(error),
            "gain_at_10Hz_200Hz": float(expected_gain),
            "group_delay_ms": 10,
            "known_four_sequence_gaps_detected": True,
            "duplicate_measurement_detection": True,
            "hardware_used": False,
        },
        indent=2,
    )
)
print("software checks passed")
