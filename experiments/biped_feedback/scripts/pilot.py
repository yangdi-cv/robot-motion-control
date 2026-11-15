from pathlib import Path
import argparse, json
from biped import BipedConfig, simulate

R = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument("name")
p.add_argument("--config", default="{}")
a = p.parse_args()
r = simulate(BipedConfig(**json.loads(a.config)), R / "pilots" / a.name)
print(json.dumps(r, indent=2), flush=True)
