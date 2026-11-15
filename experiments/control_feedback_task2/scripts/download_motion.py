"""Locate IMU member between camera directories in the official uncompressed TAR."""

from pathlib import Path
import requests, json, hashlib, time

T = Path(__file__).resolve().parents[1]
D = T / "data/tum_vi"
U = "https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room1_512_16.tar"
TOTAL = 1707110400
records = []


def get(start, size):
    start = start // 512 * 512
    end = min(TOTAL - 1, start + size - 1)
    r = requests.get(
        U + f"?task2_seek={start}_{end}",
        headers={"Range": f"bytes={start}-{end}"},
        timeout=90,
    )
    r.raise_for_status()
    assert r.status_code == 206 and r.headers["Content-Range"].startswith(
        f"bytes {start}-"
    )
    records.append(
        {
            "url": U,
            "range": r.headers["Content-Range"],
            "sha256": hashlib.sha256(r.content).hexdigest(),
        }
    )
    (D / "motion_download_ranges.json").write_text(json.dumps(records, indent=2))
    return start, r.content


def headers(data):
    result = []
    for i in range(0, len(data) - 512, 512):
        h = data[i : i + 512]
        if not h.startswith(b"dataset-room1_512_16/"):
            continue
        try:
            stored = int(h[148:156].strip(b"\0 ") or b"0", 8)
            actual = sum(h[:148]) + 8 * 32 + sum(h[156:])
            size = int(h[124:136].strip(b"\0 ") or b"0", 8)
        except ValueError:
            continue
        if stored != actual:
            continue
        result.append((i, h[:100].split(b"\0")[0].decode(), size))
    return result


def find(data, start, items):
    for offset, name, size in items:
        if name.endswith("/imu0/data.csv"):
            c = data[offset + 512 : offset + 512 + size]
            if len(c) != size:
                _, c = get(start + offset + 512, size)
            (D / "room1_imu.csv").write_bytes(c)
            print("FOUND", start + offset, name, size, flush=True)
            return True
    return False


low = 0
high = TOTAL - 16 * 1024 * 1024
for j in range(14):
    mid = ((low + high) // 2) // 512 * 512
    start, data = get(mid, 1024 * 1024)
    items = headers(data)
    assert items, (mid, "no valid header")
    print(j, mid, items[0][1], flush=True)
    if find(data, start, items):
        break
    if "/cam1/" in items[0][1]:
        low = mid
    else:
        high = mid
    if high - low < 4 * 1024 * 1024:
        start, data = get(max(0, low - 8 * 1024 * 1024), 24 * 1024 * 1024)
        if not find(data, start, headers(data)):
            raise RuntimeError("Boundary window has no imu CSV")
        break
else:
    raise RuntimeError("search exceeded")
