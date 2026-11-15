"""Read-only rendering re-runs; excluded from independent trial counts."""

from pathlib import Path
import json
from physical import PhysicalConfig, simulate_physical

R = Path(__file__).resolve().parents[1]
if __name__ == "__main__":
    for m in ["pd", "lqr"]:
        out = R / "recordings" / ("physical_" + m)
        if (out / "frames.npy").exists():
            continue
        cfg = PhysicalConfig(controller=m, force=40.0, noise=0.002, seed=0, record=True)
        result = simulate_physical(cfg, out)
        print(m, result["success"], result["recovery_s"], flush=True)
