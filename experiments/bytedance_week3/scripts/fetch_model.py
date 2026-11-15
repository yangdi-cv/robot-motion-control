from pathlib import Path
import requests, json, hashlib, xml.etree.ElementTree as E
from concurrent.futures import ThreadPoolExecutor

T = Path(__file__).resolve().parents[1]
D = T / "assets/panda"
D.mkdir(parents=True, exist_ok=True)
rev = "62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35"
base = f"https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/{rev}/franka_emika_panda/"


def fetch(name):
    p = D / name
    p.parent.mkdir(parents=True, exist_ok=True)
    r = requests.get(base + name, timeout=60)
    r.raise_for_status()
    p.write_bytes(r.content)
    return {"file": name, "sha256": hashlib.sha256(r.content).hexdigest()}


records = [fetch("panda.xml"), fetch("LICENSE")]
tree = E.parse(D / "panda.xml")
root = tree.getroot()
# Keep collision meshes/inertias; the upstream decorative meshes are unnecessary for this contact test.
for parent in root.iter():
    for child in list(parent):
        if child.tag == "geom" and child.get("class") == "visual":
            parent.remove(child)
used = {g.get("mesh") for g in root.iter("geom") if g.get("mesh")}
asset = root.find("asset")
for m in list(asset):
    if m.tag == "mesh" and m.get("name", Path(m.get("file", "")).stem) not in used:
        asset.remove(m)
files = ["assets/" + m.get("file") for m in asset.findall("mesh")]
with ThreadPoolExecutor(max_workers=6) as pool:
    records += list(pool.map(fetch, files))
root.find("option").set("integrator", "implicit")
root.find("option").set("timestep", ".001")
tree.write(D / "panda_collision.xml")
(D / "SOURCE.json").write_text(
    json.dumps(
        {
            "repo": "google-deepmind/mujoco_menagerie",
            "revision": rev,
            "files": records,
            "changes": "Removed visual-only meshes; dynamics and collision geometry retained; MuJoCo2.3 integrator compatibility.",
        },
        indent=2,
    )
)
print("model ready", rev, len(files), flush=True)
