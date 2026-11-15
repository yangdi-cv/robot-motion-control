"""Physical rerun recorded from two real simulation cameras, not prescribed poses."""

import json
import subprocess
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont
import imageio_ffmpeg
from model import R
from run import Config, simulate

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def main():
    m = json.loads((R / "manifest.json").read_text())
    key = next(
        k
        for k, c in m["configs"].items()
        if c["scenario"] == "walk"
        and c["backend"] == "clarabel"
        and c["walk_steps"] == 4
        and c["hz"] == 100
        and c["step_distance"] == 0.06
        and not c["push_x"]
        and not c["push_y"]
    )
    cfg = Config(**m["configs"][key])
    out = R / "recording"
    out.mkdir(exist_ok=True)
    (R / "figures").mkdir(exist_ok=True)
    target = R / "G1_三维四步准静态行走.mp4"
    width, height = 1440, 900
    fps = 20
    encoder = subprocess.Popen(
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
        stderr=subprocess.DEVNULL,
    )
    renderer = None
    fonts = {n: ImageFont.truetype(FONT, n) for n in [21, 25, 30]}
    count = 0

    def camera_frame(model, data, t):
        nonlocal renderer, count
        if renderer is None:
            renderer = mujoco.Renderer(model, height=640, width=720)
        canvas = Image.new("RGB", (width, height), "#eef3f7")
        draw = ImageDraw.Draw(canvas)
        draw.text(
            (25, 15),
            "Unitree G1：三维浮动基座 · 29关节力矩控制 · 慢速四步行走",
            font=fonts[30],
            fill="#17344d",
        )
        for j, azimuth in enumerate([135, 180]):
            camera = mujoco.MjvCamera()
            camera.distance = 2.1
            camera.elevation = -15
            camera.azimuth = azimuth
            camera.lookat[:] = [data.qpos[0], data.qpos[1], 0.65]
            renderer.update_scene(data, camera)
            canvas.paste(Image.fromarray(renderer.render()), (j * 720, 75))
        draw = ImageDraw.Draw(canvas)
        draw.rectangle((0, 716, width, height), fill="#eef3f7")
        root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        c = data.subtree_com[root]
        phase = min(int(t / 7), 4)
        draw.text(
            (25, 730),
            f"仿真时间 {t:.2f} s    已经过的步态周期 {phase}/4    CoM x/y/z = {c[0]:.3f} / {c[1]:.3f} / {c[2]:.3f} m",
            font=fonts[25],
            fill="#17344d",
        )
        draw.text(
            (25, 777),
            "左：斜前方；右：正前方，可观察横向重心转移与左右脚实际离地",
            font=fonts[25],
            fill="#17344d",
        )
        draw.text(
            (25, 824),
            "完整六自由度基座 · 无根部执行器/拉绳/外部支撑力 · 理想状态反馈 · 非动态高速步行",
            font=fonts[21],
            fill="#465d72",
        )
        if count in [18, 60, 180, 340, 560]:
            canvas.save(R / "figures" / f"camera_{count:03d}.png")
        encoder.stdin.write(canvas.tobytes())
        count += 1

    try:
        result = simulate(cfg, out, on_frame=camera_frame)
    finally:
        if renderer is not None:
            renderer.close()
        encoder.stdin.close()
        assert encoder.wait() == 0
    assert result["success"]
    recorded = np.load(out / "trace.npz")
    original = np.load(R / m["root"] / key / "trace.npz")
    comparisons = {
        field: float(
            np.max(
                np.abs(recorded[field].astype(float) - original[field].astype(float))
            )
        )
        for field in ["qpos", "qvel", "ctrl", "com", "feet", "actual_wrench"]
    }
    assert all(value < 1e-10 for value in comparisons.values()), comparisons
    decoder = imageio_ffmpeg.read_frames(str(target))
    next(decoder)
    decoded = sum(1 for _ in decoder)
    assert decoded == count
    (R / "video_metadata.json").write_text(
        json.dumps(
            dict(
                frames=count,
                fps=fps,
                duration=count / fps,
                decoded_frames=decoded,
                benchmark_case=key,
                rerun_max_difference=comparisons,
                physical_success=True,
                landings=result["landings"],
                progress=result["forward_progress"],
            ),
            indent=2,
        )
    )
    print(target)


if __name__ == "__main__":
    main()
