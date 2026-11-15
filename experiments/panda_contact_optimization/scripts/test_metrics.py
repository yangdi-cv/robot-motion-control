"""Regression: raw impulses must survive evaluation, including at a changed physics step."""

import csv, json, tempfile, unittest
from pathlib import Path
import numpy as np
from analyze import metrics


class ForceEvaluation(unittest.TestCase):
    def fixture(self, root, dt):
        root.mkdir()
        (root / "metadata.json").write_text(
            json.dumps({"physics_dt": dt, "optimization": {}})
        )
        (root / "time_audit.json").write_text(json.dumps({"measured_duration": 20.0}))
        n = round(20 / dt)
        f = np.full(n, 5.0)
        f[round(10 / dt)] = 5.0 + 0.095 / dt
        with (root / "physics_force.csv").open("w") as fd:
            w = csv.writer(fd)
            w.writerow(["t", "fz_N"])
            w.writerows(zip((np.arange(n) + 1) * dt, f))
        with (root / "trace.csv").open("w") as fd:
            w = csv.writer(fd)
            w.writerow(
                ["t"] + [f"q{i}" for i in range(3)] + [f"ref{i}" for i in range(3)]
            )
            w.writerows([i * 0.01, 0.001, 0.001, 0.001, 0, 0, 0] for i in range(2000))
        return metrics(root)

    def test_impulse_and_fixed_bins(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = self.fixture(Path(tmp) / "one_ms", 0.001)
            b = self.fixture(Path(tmp) / "half_ms", 0.0005)
            self.assertAlmostEqual(a["raw_peak_N"], 100.0)
            self.assertAlmostEqual(b["raw_peak_N"], 195.0)
            self.assertAlmostEqual(a["cycle_peak_N"], 14.5)
            self.assertAlmostEqual(b["cycle_peak_N"], 14.5)
            self.assertEqual(a["duration_s"], 20.0)
            self.assertEqual(b["duration_s"], 20.0)
            self.assertEqual(a["last2_in_band_pct"], 100.0)
            self.assertAlmostEqual(a["first5_joint_rmse_deg"][0], np.rad2deg(0.001))


if __name__ == "__main__":
    unittest.main()
