"""Render the completed report and create source-only / experiment packages."""

from pathlib import Path
import sys, json, hashlib, re, subprocess, zipfile, shutil

sys.path.insert(0, "/tmp/task2-tools")
import markdown

T = Path(__file__).resolve().parents[1]
ROOT = T.parents[1]
OUT = ROOT / "deliverables/20261020_字节Task2阶段"
OUT.mkdir(exist_ok=True)
provenance = {
    "date": "2026-10-20 Australia/Sydney",
    "simulation": "Isaac5.1.0 / ROS2 Humble / NVIDIA Franka Panda, domain71",
    "dataset": {
        "name": "TUM VI Benchmark",
        "authors": "Schubert, Demmel, von Stumberg, Usenko, Cremers (2018)",
        "page": "https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset",
        "license": "CC BY 4.0",
        "static_url": "https://cdn3.vision.in.tum.de/tumvi/imu_static/dataset-calib-imu-static2.npy",
        "motion_url": "https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room1_512_16.tar",
        "static": "first120001 float64 rows; NPY container recreated, original values retained",
        "motion": "original imu0/data.csv bytes, full member extracted by checked HTTP ranges",
        "full_remote_file_md5_verified": False,
        "local_hashes": {
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (T / "data/tum_vi").iterdir()
            if p.is_file()
        },
    },
    "hardware_results": {
        "jetson": False,
        "own_board_capture": False,
        "physical_motion_commands": False,
    },
    "source_sha256": {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (T / "scripts").iterdir()
        if p.is_file()
    },
    "new_scripts_postrun_changes": "Formatting, CLI output-path options, and clean signal handling; numerical analysis and experimental configurations preserved.",
}
(T / "PROVENANCE.json").write_text(json.dumps(provenance, ensure_ascii=False, indent=2))
(T / "data/tum_vi/ATTRIBUTION.md").write_text(
    """# 数据来源\n\nTUM VI Benchmark，Schubert等，2018，CC BY4.0。\nhttps://cvg.cit.tum.de/data/datasets/visual-inertial-dataset\n\n本目录包含官方静止NPY的前120001行数值子集、room1的原始imu0/data.csv，以及用于本实验的两个30秒CSV。静止NPY容器重新保存，数值没有改写；30秒CSV重新排列为time_s、ax、ay、az、gx、gy、gz，并将运动原始纳秒时间换为相对秒。未经原作者或本实验声称的自有设备采集。\n\n完整远端文件MD5未验证，本地内容哈希与已请求的HTTP范围见PROVENANCE.json、motion_download_ranges.json。\n"""
)
style = """@page{size:A4;margin:18mm 16mm 19mm;@bottom-right{content:counter(page);font-size:9pt;color:#68778a}}body{font-family:"Noto Sans CJK SC","Microsoft YaHei",sans-serif;color:#253347;font-size:10pt;line-height:1.7}h1{font-size:22pt;line-height:1.4}h2{font-size:15pt;margin-top:8mm;break-after:avoid}h3{font-size:12pt;break-after:avoid}table{width:100%;border-collapse:collapse;font-size:8.7pt;break-inside:avoid;margin:4mm 0}td,th{border:1px solid #cbd5df;padding:5px 7px;text-align:left}th{background:#eaf0f5}img{max-width:100%;max-height:105mm;object-fit:contain}p:has(img){text-align:center;break-inside:avoid;margin:5mm 0}pre{background:#f1f5f8;padding:10px;font-size:8pt;line-height:1.5;white-space:pre-wrap}code{font-family:"Noto Sans Mono CJK SC",monospace;font-size:8.6pt;overflow-wrap:anywhere}a{color:#295f88;text-decoration:none}p{orphans:3;widows:3}.toc{break-before:page}.refs{font-size:9pt;color:#496174;border-left:3px solid #ccdfec;padding-left:9px}"""
md = (T / "REPORT.md").read_text()
body = markdown.markdown(md, extensions=["tables", "fenced_code"])
body = body.replace("<h2>目录</h2>", '<h2 class="toc">目录</h2>')
body = re.sub(r"<p>(相关代码在.*?)</p>", r'<p class="refs">\1</p>', body, flags=re.S)
html = T / "REPORT.html"
html.write_text(
    '<!doctype html><html lang="zh"><meta charset="utf-8"><title>Panda控制与IMU反馈阶段进展</title><style>'
    + style
    + "</style><body>"
    + body
    + "</body></html>"
)
subprocess.run(
    [
        "/usr/bin/google-chrome",
        "--headless",
        "--no-sandbox",
        "--disable-gpu",
        "--no-pdf-header-footer",
        "--print-to-pdf=" + str(T / "REPORT.pdf"),
        html.as_uri(),
    ],
    stdout=subprocess.DEVNULL,
    stderr=subprocess.PIPE,
    check=True,
)
shutil.copy2(T / "REPORT.pdf", OUT / "字节Task2_阶段进展.pdf")
source = []
for d in ["scripts", "config", "vendor"]:
    source += [
        p for p in (T / d).rglob("*") if p.is_file() and "__pycache__" not in p.parts
    ]
internal = {
    "log_progress.py",
    "after_control.py",
    "after_feedback.py",
    "finalize.py",
    "finalize_norender.py",
    "build_delivery.py",
}
source = [p for p in source if p.name not in internal]
source += [T / "README.md", T / "requirements-analysis.txt"]
materials = source + [
    T / n for n in ["REPORT.md", "REPORT.pdf", "PROVENANCE.json", "VERIFICATION.json"]
]
materials += [p for p in (T / "data/tum_vi").iterdir() if p.is_file()]
materials += list((T / "figures").glob("*.png"))
for folder in [
    "control002",
    "control_validation",
    "control_frequency",
    "control_frequency_norender",
    "feedback001",
    "generic_filter_static",
]:
    materials += [
        p
        for p in (T / "runs" / folder).rglob("*")
        if p.is_file() and p.suffix in [".csv", ".json", ".jsonl", ".png", ".log"]
    ]
materials += [
    p for p in (T / "runs").iterdir() if p.is_file() and p.suffix in [".json", ".csv"]
]
manifest = []
for name, files in [
    ("字节Task2_源码.zip", source),
    ("字节Task2_实验材料.zip", materials),
]:
    path = OUT / name
    with zipfile.ZipFile(
        path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6
    ) as z:
        for p in sorted(set(files)):
            z.write(p, p.relative_to(ROOT))
    with zipfile.ZipFile(path) as z:
        assert z.testzip() is None
    manifest.append(
        {
            "file": name,
            "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    )
(T / "DELIVERY.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
print(json.dumps(manifest, ensure_ascii=False, indent=2))
