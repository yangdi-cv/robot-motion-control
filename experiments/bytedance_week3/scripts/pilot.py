from contact_task import run
from pathlib import Path
import json, numpy as np

T = Path(__file__).resolve().parents[1]
for mode in ["position", "admittance"]:
    a, s = run(mode)
    np.savetxt(T / "runs" / f"pilot_{mode}.csv", a, delimiter=",")
    print(s, flush=True)
