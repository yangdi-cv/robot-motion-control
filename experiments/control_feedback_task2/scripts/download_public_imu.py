"""Fetch only the required TUM-VI IMU bytes with checked HTTP ranges."""

from pathlib import Path
import requests, re, json, hashlib, io, ast, time
import numpy as np

T = Path(__file__).resolve().parents[1]
D = T / "data/tum_vi"
D.mkdir(parents=True, exist_ok=True)
STATIC = "https://cdn3.vision.in.tum.de/tumvi/imu_static/dataset-calib-imu-static2.npy"
ROOM = (
    "https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room1_512_16.tar"
)
records = []


def get(url, start, end):
    for attempt in range(3):
        try:
            r = requests.get(
                url + f"?task2_range={start}_{end}",
                headers={"Range": f"bytes={start}-{end}"},
                timeout=90,
            )
            r.raise_for_status()
            cr = r.headers.get("Content-Range", "")
            if (
                r.status_code != 206
                or not cr.startswith(f"bytes {start}-")
                or len(r.content) != end - start + 1
            ):
                raise ValueError((r.status_code, cr, len(r.content), start, end))
            records.append(
                {
                    "url": url,
                    "range": cr,
                    "sha256": hashlib.sha256(r.content).hexdigest(),
                    "bytes": len(r.content),
                }
            )
            return r.content
        except Exception:
            if attempt == 2:
                raise
            time.sleep(2)


if not (D / "static_600s.npy").exists():
    header = get(STATIC, 0, 1023)
    hsize = 10 + int.from_bytes(header[8:10], "little")
    info = ast.literal_eval(header[10:hsize].decode().strip())
    assert (
        info["descr"] == "<f8" and not info["fortran_order"] and info["shape"][1] == 8
    )
    rows = 120001
    data = get(STATIC, hsize, hsize + rows * 8 * 8 - 1)
    arr = np.frombuffer(data, dtype="<f8").reshape(-1, 8).copy()
    np.save(D / "static_600s.npy", arr)
    print("static", arr.shape, arr[-1, 0] - arr[0, 0], flush=True)
if not (D / "room1_imu.csv").exists():
    total = 1707110400
    length = 16 * 1024 * 1024
    start = (total - length) // 512 * 512
    tail = get(ROOM, start, total - 1)
    found = None
    for pos in range(0, len(tail) - 512, 512):
        h = tail[pos : pos + 512]
        name = h[:100].split(b"\0")[0].decode(errors="replace")
        if name.endswith("/imu0/data.csv"):
            size = int(h[124:136].strip(b"\0 ") or b"0", 8)
            content = tail[pos + 512 : pos + 512 + size]
            if len(content) != size:
                content = get(ROOM, start + pos + 512, start + pos + 511 + size)
            found = content
            print("room member", name, "size", size, flush=True)
            break
    if found is None:
        import subprocess, sys

        subprocess.run(
            [sys.executable, str(T / "scripts/download_motion.py")], check=True
        )
        found = (D / "room1_imu.csv").read_bytes()
    (D / "room1_imu.csv").write_bytes(found)
(D / "download.json").write_text(
    json.dumps(
        {
            "dataset": "TUM VI, Schubert et al. 2018",
            "landing_page": "https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset",
            "license": "CC BY 4.0",
            "static_subset": "first120001rows, original NPY float64 columns time_s,gx,gy,gz,ax,ay,az,temp_C",
            "room": "full original room1 imu0/data.csv extracted from uncompressed TAR by range",
            "requests": records,
            "full_remote_file_checksum_verified": False,
        },
        indent=2,
    )
)
