"""Animations of saved model states and actual MuJoCo camera frames."""

from pathlib import Path
import json, subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import imageio_ffmpeg
from core import Config, Reference, key

R = Path(__file__).resolve().parents[1]
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def font(n):
    return ImageFont.truetype(FONT, n)


def encoder(path, w, h, fps):
    return subprocess.Popen(
        [
            imageio_ffmpeg.get_ffmpeg_exe(),
            "-y",
            "-f",
            "rawvideo",
            "-vcodec",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            f"{w}x{h}",
            "-r",
            str(fps),
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-crf",
            "20",
            "-pix_fmt",
            "yuv420p",
            str(path),
        ],
        stdin=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )


def close(p):
    p.stdin.close()
    assert p.wait() == 0


def lipm_video():
    manifest = json.loads((R / "manifest.json").read_text())
    results = json.loads((R / "results.json").read_text())
    methods = ["lqr", "capture", "capture_predict"]
    labels = ["LQR · 固定落脚", "LQR · 落脚调整", "LQR · 预测＋落脚调整"]
    colors = ["#a45b43", "#7063a9", "#218259"]
    cfgs = [Config(controller=m, impulse_x=12.0, delay=0.08, seed=1) for m in methods]
    raw = [np.load(R / manifest["root"] / key(c) / "physics.npz") for c in cfgs]
    controls = [
        np.load(R / manifest["root"] / key(c) / "trace.npz")["data"] for c in cfgs
    ]
    reference = Reference(cfgs[0])
    dt = cfgs[0].dt
    W, H = 1400, 1100
    output = R / "lipm_comparison.mp4"
    p = encoder(output, W, H, 20)
    times = np.arange(0, reference.n + 1, 50) * dt
    all_errors = [
        np.linalg.norm(
            v["state"][:, :, 0] - reference.state[: len(v["state"]), :, 0] - v["shift"],
            axis=1,
        )
        for v in raw
    ]
    for frame, t in enumerate(times):
        im = Image.new("RGB", (W, H), "#f2f5f8")
        d = ImageDraw.Draw(im)
        d.text(
            (26, 12),
            "倒立摆多步控制：12 N·s推扰 ＋ 80 ms反馈延迟",
            font=font(28),
            fill="#18374d",
        )
        d.text((1175, 18), f"t = {t:4.2f} s", font=font(23), fill="#40576b")
        for j, (cfg, v, label, col) in enumerate(zip(cfgs, raw, labels, colors)):
            left = 20 + j * 460
            index = min(round(t / dt), len(v["state"]) - 1)
            end_time = (len(v["state"]) - 1) * dt
            terminated = (not results[key(cfg)]["success"]) and t >= end_time - 1e-9
            state = v["state"][index]
            foot = v["foot"][index]
            shift = v["shift"][index]
            row = max(
                0,
                np.searchsorted(controls[j][:, 0], index * dt + 1e-9, side="right") - 1,
            )
            cop = controls[j][row, [9, 10]]
            assert np.all(np.abs(cop - foot) <= np.array([0.095, 0.045]) + 1e-8)
            phase = min(int(t / cfg.step_time), cfg.steps - 1)
            d.rounded_rectangle(
                (left, 62, left + 440, 731), radius=12, fill="white", outline="#d0dbe4"
            )
            d.text((left + 15, 76), label, font=font(23), fill=col)
            d.text(
                (left + 15, 113),
                "约束失效，模型已终止" if terminated else "模型运行中",
                font=font(19),
                fill="#bb443d" if terminated else "#4a6377",
            )
            center = reference.state[min(index, reference.n), 0, 0]

            d.text(
                (left + 15, 145),
                "侧视 x-z · 金点为CoP命令",
                font=font(15),
                fill="#8394a2",
            )

            def side(x, z):
                return (left + 220 + (x - center) * 310, 455 - z * 340)

            floor = side(0, 0)[1]
            d.line((left + 12, floor, left + 427, floor), fill="#95a7b7", width=2)
            fx, fy = side(foot[0], 0)
            d.rectangle((fx - 31, fy - 8, fx + 31, fy + 4), fill=col)
            px, py = side(cop[0], 0)
            cx, cz = side(state[0, 0], cfg.height)
            d.line((px, py, cx, cz), fill="#7891a4", width=5)
            d.ellipse((cx - 18, cz - 18, cx + 18, cz + 18), fill=col)
            d.ellipse((px - 5, py - 5, px + 5, py + 5), fill="#d19c2f")
            if (
                cfg.push_time <= t < cfg.push_time + cfg.push_duration
                and not terminated
            ):
                d.line((cx - 85, cz, cx - 25, cz), fill="#d64d42", width=7)
                d.polygon(
                    [(cx - 25, cz), (cx - 39, cz - 10), (cx - 39, cz + 10)],
                    fill="#d64d42",
                )
            err = all_errors[j][index]
            d.text(
                (left + 15, 478),
                f"跟踪误差 {err*100:5.1f} cm",
                font=font(19),
                fill="#40576b",
            )
            d.text(
                (left + 15, 511),
                f"路线偏移 {np.linalg.norm(shift)*100:5.1f} cm",
                font=font(19),
                fill="#40576b",
            )
            # Top view: planned footprints are gray; actual landings are colored.
            d.text(
                (left + 15, 551),
                "俯视：计划落脚与实际路径",
                font=font(17),
                fill="#64798a",
            )

            def top(x, y):
                return (left + 35 + (x + 0.2) / 2.7 * 370, 650 - y / 0.8 * 120)

            for f in reference.feet:
                a, b = top(*f)
                d.rectangle((a - 12, b - 7, a + 12, b + 7), outline="#c6d0da", width=1)
            landing = np.array(results[key(cfg)]["landings"])[
                : min(
                    index // reference.phase_steps + 1,
                    len(results[key(cfg)]["landings"]),
                )
            ]
            for f in landing:
                a, b = top(*f)
                d.rectangle((a - 12, b - 7, a + 12, b + 7), outline=col, width=2)
            history = v["state"][: index + 1 : 20, :, 0]
            pts = [top(*q) for q in history]
            if len(pts) > 1:
                d.line(pts, fill=col, width=2)
        box = (80, 812, 1350, 1006)
        x0, y0, x1, y1 = box
        d.rectangle(box, fill="white", outline="#c7d3de")

        def chart(tt, e):
            return (
                x0 + tt / reference.duration * (x1 - x0),
                y1 - min(e, 0.4) / 0.4 * (y1 - y0),
            )

        for value in [0, 0.1, 0.2, 0.3, 0.4]:
            yy = chart(0, value)[1]
            d.line((x0, yy, x1, yy), fill="#dbe3ea")
            d.text((18, yy - 11), f"{value:.1f}m", font=font(16), fill="#64798a")
        for j, (errors, col) in enumerate(zip(all_errors, colors)):
            stop = min(round(t / dt), len(errors) - 1)
            pts = [chart(i * dt, errors[i]) for i in range(0, stop + 1, 10)]
            if len(pts) > 1:
                d.line(pts, fill=col, width=3)
        xx = chart(t, 0)[0]
        d.line((xx, y0, xx, y1), fill="#344c60", width=2)
        d.text(
            (80, 764),
            "误差相对调整后的路线计算；路线平移另外显示，不隐藏绕行。",
            font=font(20),
            fill="#40576b",
        )
        for tick in [0, 2, 4, 6, 8]:
            a, b = chart(tick, 0)
            d.text((a - 8, 1013), f"{tick}s", font=font(16), fill="#64798a")
        d.text(
            (28, 1057),
            "简化模型：恒定质心高度、理想瞬时换支撑；不包含全身关节与双足接触动力学。",
            font=font(19),
            fill="#64798a",
        )
        if abs(t - 3.5) < 1e-9:
            im.save(R / "figures/lipm_keyframe.png")
        p.stdin.write(im.tobytes())
    close(p)
    return {
        "file": output.name,
        "frames": len(times),
        "fps": 20,
        "model_duration_s": reference.duration,
    }


def physical_video():
    dirs = [R / "recordings" / ("physical_" + m) for m in ["pd", "lqr"]]
    frames = [np.load(p / "frames.npy", mmap_mode="r") for p in dirs]
    traces = [np.load(p / "trace.npz")["data"] for p in dirs]
    colors = ["#cd754c", "#7759aa"]
    labels = ["PD", "LQR"]
    W, H = 1400, 1080
    output = R / "physical_comparison.mp4"
    enc = encoder(output, W, H, 20)
    n = min(len(f) for f in frames)
    for k in range(n):
        t = k * 0.05
        im = Image.new("RGB", (W, H), "#f2f5f8")
        d = ImageDraw.Draw(im)
        d.text(
            (25, 12),
            "有限脚掌非线性站立：40 N、0.1 s推扰",
            font=font(28),
            fill="#19394f",
        )
        for j, (images, x, col, label) in enumerate(
            zip(frames, traces, colors, labels)
        ):
            left = 20 + j * 695
            d.text((left + 12, 59), label, font=font(25), fill=col)
            im.paste(Image.fromarray(images[k]).resize((675, 506)), (left, 98))
            row = x[min(round(t / 0.001), len(x) - 1)]
            d.text(
                (left + 12, 614),
                f"倾角 {np.rad2deg(row[1]):+.2f}°  ·  力矩 {row[6]:+.2f} Nm",
                font=font(21),
                fill=col,
            )
            d.text(
                (left + 12, 650),
                f"地面法向合力 {row[8]:.1f} N",
                font=font(20),
                fill="#4d6477",
            )
        for top, bottom, column, scale, title in [
            (748, 866, 1, 2.0, "世界倾角 / °"),
            (918, 1036, 6, 25.0, "踝关节力矩 / Nm"),
        ]:
            x0, x1 = 85, 1350
            d.rectangle((x0, top, x1, bottom), fill="white", outline="#c6d2dc")

            def point(tt, val):
                return (
                    x0 + tt / 6 * (x1 - x0),
                    (top + bottom) / 2 - val / scale * (bottom - top) / 2,
                )

            yy = point(0, 0)[1]
            d.line((x0, yy, x1, yy), fill="#d1dce6")
            d.text((20, top - 28), title, font=font(18), fill="#4d6477")
            for x, col in zip(traces, colors):
                end = min(round(t / 0.001), len(x) - 1)
                pts = []
                for i in range(0, end + 1, 10):
                    value = np.rad2deg(x[i, column]) if column == 1 else x[i, column]
                    pts.append(point(x[i, 0], float(value)))
                if len(pts) > 1:
                    d.line(pts, fill=col, width=3)
            xx = point(t, 0)[0]
            d.line((xx, top, xx, bottom), fill="#344c60", width=2)
        d.text(
            (26, 1050),
            "MuJoCo真实刚体接触；此模型验证单脚站立平衡，不是双足行走。",
            font=font(19),
            fill="#64798a",
        )
        if k == 50:
            im.save(R / "figures/physical_keyframe.png")
        enc.stdin.write(im.tobytes())
    close(enc)
    return {"file": output.name, "frames": n, "fps": 20, "model_duration_s": 6}


if __name__ == "__main__":
    result = {"lipm": lipm_video(), "physical": physical_video()}
    (R / "video_metadata.json").write_text(json.dumps(result, indent=2))
    print(result)
