"""Download the fixed official policy, verifying its SHA256 before saving."""
from pathlib import Path
import hashlib,urllib.request
rev='276801e46c5d433564f24658bac64f254b7d2d4b'
expected='cf668f75b90d1abf73d2b87612a6e76bccc61ff7e083b63582d3f6aaa3c1759d'
url=f'https://raw.githubusercontent.com/unitreerobotics/unitree_rl_gym/{rev}/deploy/pre_train/g1/motion.pt'
p=Path(__file__).resolve().parents[1]/'upstream/deploy/pre_train/g1/motion.pt'
b=urllib.request.urlopen(url,timeout=60).read()
assert hashlib.sha256(b).hexdigest()==expected,'Policy digest mismatch'
p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b)
print('Verified fixed official policy:',p)
