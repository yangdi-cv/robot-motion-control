"""Pair actual Isaac camera frames with corresponding measured force histories."""

from pathlib import Path
import subprocess, json
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import imageio_ffmpeg

R = Path(__file__).resolve().parents[1]
font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font = lambda n: ImageFont.truetype(font_path, n)
if __name__ == "__main__":
    names = ["video_position", "video_admittance"]
    labels = ["位置控制", "导纳控制（优化后）"]
    colors = ["#dc6547", "#247fa0"]
    traces = [
        np.genfromtxt(R / "runs" / n / "physics_force.csv", delimiter=",", names=True)
        for n in names
    ]
    bins = [x["fz_N"].reshape(-1, 10).mean(axis=1) for x in traces]
    frames = [sorted((R / "runs" / n / "frames").glob("*.png")) for n in names]
    count = min(map(len, frames))
    assert count > 0
    duration = count / 10
    xmax = max(len(b) for b in bins) * 0.01
    ymax = max(10.0, max(float(b.max()) for b in bins) * 1.1)
    W, H = 1400, 1000
    out = R / "comparison.mp4"
    cmd = [
        imageio_ffmpeg.get_ffmpeg_exe(),
        "-y",
        "-f",
        "rawvideo",
        "-vcodec",
        "rawvideo",
        "-s",
        f"{W}x{H}",
        "-pix_fmt",
        "rgb24",
        "-r",
        "10",
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-crf",
        "20",
        "-pix_fmt",
        "yuv420p",
        str(out),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    for k in range(count):
        now = k * 0.1 + 0.001
        im = Image.new("RGB", (W, H), "#f2f5f8")
        d = ImageDraw.Draw(im)
        d.text(
            (28, 12),
            "Panda 接触控制｜相同重力、几何、材质与内环",
            font=font(27),
            fill="#19384c",
        )
        d.text((W - 260, 18), f"仿真时间 {now:5.1f} s", font=font(22), fill="#344b60")
        for j, (n, label, col) in enumerate(zip(names, labels, colors)):
            x0 = 20 + j * 695
            d.text((x0 + 10, 57), label, font=font(25), fill=col)
            cam = Image.open(frames[j][k]).convert("RGB").resize((675, 506))
            im.paste(cam, (x0, 101))
            idx = min(int(now / 0.01) - 1, len(bins[j]) - 1)
            val = bins[j][max(0, idx)]
            raw = traces[j]["fz_N"][: min(len(traces[j]), k * 100 + 1)]
            d.text(
                (x0 + 10, 615),
                f"10ms 均力 {val:.2f} N  ·  目标 5 N",
                font=font(22),
                fill=col,
            )
            d.text(
                (x0 + 10, 650),
                f"截至当前原始峰值 {raw.max():.2f} N",
                font=font(19),
                fill="#43586b",
            )
        box = (85, 730, 1350, 925)
        left, top, right, bottom = box
        mapxy = lambda t, f: (
            left + t / xmax * (right - left),
            bottom - f / ymax * (bottom - top),
        )
        d.rectangle(box, fill="white", outline="#c7d2dd")
        y1 = mapxy(0, 5.5)[1]
        y2 = mapxy(0, 4.5)[1]
        d.rectangle((left, y1, right, y2), fill="#e0f0df")
        for y in np.linspace(0, ymax, 5):
            yy = mapxy(0, y)[1]
            d.line((left, yy, right, yy), fill="#dde5ec")
            d.text((10, yy - 10), f"{y:.0f} N", font=font(16), fill="#536574")
        for j, b in enumerate(bins):
            end = min(len(b), max(1, int(now / 0.01)))
            pts = [mapxy((i + 1) * 0.01, float(b[i])) for i in range(end)]
            if len(pts) > 1:
                d.line(pts, fill=colors[j], width=3)
        yy = mapxy(0, 5)[1]
        d.line((left, yy, right, yy), fill="#56865b", width=2)
        xx = mapxy(now, 0)[0]
        d.line((xx, top, xx, bottom), fill="#263d50", width=2)
        d.text(
            (85, 692),
            "力曲线：原始测量固定分箱均值；绿色区域为 5±0.5 N",
            font=font(21),
            fill="#344b60",
        )
        for tick in np.linspace(0, xmax, 5):
            tick_x = mapxy(tick, 0)[0]
            d.text((tick_x - 15, 930), f"{tick:.0f}s", font=font(16), fill="#536574")
        d.text(
            (85, 970),
            "固定基座仿真；画面每 0.1 秒采样，原始力每 0.001 秒记录。",
            font=font(20),
            fill="#536574",
        )
        if k == max(0, count - 20):
            (R / "figures").mkdir(exist_ok=True)
            im.save(R / "figures/comparison_keyframe.png")
        proc.stdin.write(im.tobytes())
    proc.stdin.close()
    err = proc.stderr.read()
    ret = proc.wait()
    if ret:
        raise RuntimeError(err.decode())
    (R / "video_metadata.json").write_text(
        json.dumps(
            {
                "source_runs": names,
                "frames": count,
                "fps": 10,
                "duration_s": duration,
                "force_curve": "unfiltered raw force averaged over fixed 10ms bins",
                "camera_frame_time_s": "0.001 + k*0.1",
                "no_interpolated_simulation_frames": True,
            },
            indent=2,
        )
    )
    print(out)
