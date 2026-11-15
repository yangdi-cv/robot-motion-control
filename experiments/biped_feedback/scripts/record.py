"""Record one successful 20-step physical rerun; compare it against the frozen trace."""

from dataclasses import replace
import json
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import imageio_ffmpeg
from biped import BipedConfig, simulate

R = Path(__file__).resolve().parents[1]
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def main():
    manifest = json.loads((R / "manifest.json").read_text())
    key = next(
        k
        for k, c in manifest["configs"].items()
        if c["steps"] == 20 and c["push"] == 40
    )
    config = BipedConfig(**manifest["configs"][key])
    out = R / "recording"
    if not (out / "result.json").exists():
        simulate(replace(config, record=True), out)
    physical = np.load(out / "trace.npz")["data"]
    original = np.load(R / manifest["root"] / key / "trace.npz")["data"]
    assert np.array_equal(physical, original), "Recording differs from benchmark"
    frames = np.load(out / "frames.npy", mmap_mode="r")
    result = json.loads((out / "result.json").read_text())
    assert result["success"] and all(result["verified_landings"])
    fps = 20
    width, height = 1120, 900
    target = R / "连续二十步_实际动力学.mp4"
    proc = subprocess.Popen(
        [
            imageio_ffmpeg.get_ffmpeg_exe(),
            "-y",
            "-f",
            "rawvideo",
            "-vcodec",
            "rawvideo",
            "-s",
            f"{width}x{height}",
            "-pix_fmt",
            "rgb24",
            "-r",
            str(fps),
            "-i",
            "-",
            "-an",
            "-vcodec",
            "libx264",
            "-crf",
            "20",
            "-pix_fmt",
            "yuv420p",
            str(target),
        ],
        stdin=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    fonts = {n: ImageFont.truetype(FONT, n) for n in [20, 24, 30]}
    for index, frame in enumerate(frames):
        t = index / fps
        row = physical[min(np.searchsorted(physical[:, 0], t), len(physical) - 1)]
        image = Image.new("RGB", (width, height), "#edf2f6")
        image.paste(Image.fromarray(frame).resize((1040, 780)), (40, 50))
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, width, 66), fill="#edf2f6")
        draw.text(
            (28, 14),
            "平面双足连续行走：真实接触、六关节力矩驱动",
            font=fonts[30],
            fill="#17334d",
        )
        draw.rectangle((25, 704, 1095, 885), fill="#edf2f6")
        draw.text(
            (40, 715),
            f"仿真时间 {t:.2f} s    质心位置 {row[1]:.3f} m    躯干俯仰 {np.degrees(row[3]):.2f}°",
            font=fonts[24],
            fill="#17334d",
        )
        landed = sum(
            t >= 0.5 + (s + 1) * config.step_time + 0.10 for s in range(config.steps)
        )
        draw.text(
            (40, 755),
            f"已通过落脚时段 {landed}/20    左/右足实际法向力 {row[8]:.0f} / {row[9]:.0f} N",
            font=fonts[24],
            fill="#17334d",
        )
        draw.text(
            (40, 797),
            "2.70–2.80 s 施加前向推力 40 N；100 Hz 控制、1 ms 物理积分",
            font=fonts[20],
            fill="#45566a",
        )
        draw.text(
            (40, 836),
            "平面模型 · 理想状态反馈 · 无基座执行器 · 录制轨迹与正式实验逐元素一致",
            font=fonts[20],
            fill="#45566a",
        )
        if index in [14, 65, 142, 255]:
            image.save(R / "figures" / f"frame_{index:03d}.png")
        proc.stdin.write(image.tobytes())
    proc.stdin.close()
    error = proc.stderr.read()
    assert proc.wait() == 0, error.decode()[-1000:]
    proc.stderr.close()
    # Decode every encoded frame independently; frame count must match the renderer.
    reader = imageio_ffmpeg.read_frames(str(target))
    next(reader)
    count = sum(1 for _ in reader)
    assert count == len(frames)
    (R / "video_metadata.json").write_text(
        json.dumps(
            dict(
                frames=count,
                fps=fps,
                size=[width, height],
                duration=count / fps,
                benchmark_case=key,
                trace_exact_equal=True,
                success=True,
                landings=20,
                progress=result["progress_m"],
            ),
            indent=2,
        )
    )
    print(target)


if __name__ == "__main__":
    main()
