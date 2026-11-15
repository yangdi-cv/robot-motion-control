from contact_task import run
from pathlib import Path
import numpy as np, json

T = Path(__file__).resolve().parents[1]
O = T / "runs/joint_check_v3"
O.mkdir(exist_ok=False)
rows = []
for k in [2000, 8000, 20000]:
    for mode in ["position", "admittance"]:
        a, s = run(mode, stiffness=k)
        assert a.shape[1] == 29
        err = a[:, 8:15] - a[:, 22:29]
        s["joint_rmse_deg"] = np.sqrt(np.mean(err**2, axis=0)).tolist()
        s["joint_rmse_deg"] = [float(v * 180 / np.pi) for v in s["joint_rmse_deg"]]
        np.savez_compressed(O / f"{k}_{mode}.npz", samples=a)
        rows.append(s)
(O / "summary.json").write_text(json.dumps(rows, indent=2))
print(rows, flush=True)
