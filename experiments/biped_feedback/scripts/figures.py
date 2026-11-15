"""Charts from saved trajectories; no synthetic success annotations."""

import json
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

R = Path(__file__).resolve().parents[1]
plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False


def main():
    manifest = json.loads((R / "manifest.json").read_text())
    summary = json.loads((R / "summary.json").read_text())
    cases = summary["cases"]
    root = R / manifest["root"]
    out = R / "figures"
    out.mkdir(exist_ok=True)

    def find(**values):
        return next(
            k
            for k, c in manifest["configs"].items()
            if all(c.get(n) == v for n, v in values.items())
        )

    def audit(**values):
        return np.load(root / find(**values) / "audit.npz")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    x = np.arange(3)
    for mode, label, offset in [
        ("legacy", "仅CoM任务", -0.18),
        ("coupled", "CoM＋ZMP映射", 0.18),
    ]:
        rows = [
            cases[
                find(
                    mapping=mode,
                    hz=hz,
                    push=0.0,
                    torque_rate=0.0,
                    feedback="ideal",
                    steps=8,
                    ds=0.12,
                    dt=0.001,
                )
            ]
            for hz in [50, 100, 200]
        ]
        axes[0].bar(
            x + offset,
            [r["cop_measured_rmse_mm"] for r in rows],
            width=0.36,
            label=label,
        )
        axes[1].bar(
            x + offset, [r["com_rmse_mm"] for r in rows], width=0.36, label=label
        )
    for ax, title in zip(axes, ["实际压力中心RMSE（mm）", "质心位置RMSE（mm）"]):
        ax.set_xticks(x, ["50 Hz", "100 Hz", "200 Hz"])
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.2)
    axes[0].legend()
    fig.savefig(out / "mapping.png", dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(
        2, 1, figsize=(11, 6), sharex=True, constrained_layout=True
    )
    for mode, label in [("legacy", "仅CoM任务"), ("coupled", "CoM＋ZMP映射")]:
        a = audit(
            mapping=mode,
            hz=100,
            push=40.0,
            torque_rate=0.0,
            feedback="ideal",
            steps=8,
            ds=0.12,
            dt=0.001,
        )
        axes[0].plot(a["time"], a["measured_cop"], label=label)
        axes[1].plot(
            a["time"], (a["com"][:, 0] - a["target_com"][:, 0]) * 1000, label=label
        )
    axes[0].plot(a["time"], a["cop_command"], "k--", lw=1, label="新映射ZMP目标")
    axes[0].set_ylabel("地面CoP x（m）")
    axes[1].set_ylabel("质心跟踪误差（mm）")
    axes[1].set_xlabel("仿真时间（s）")
    for ax in axes:
        ax.axvspan(2.7, 2.8, color="orange", alpha=0.2)
        ax.grid(alpha=0.2)
        ax.legend()
    fig.savefig(out / "cop_com.png", dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    for ax, mode, title in zip(
        axes, ["hold", "predict"], ["保持过去测量", "常速度预测"]
    ):
        matrix = np.zeros((3, 3))
        for i, hz in enumerate([50, 100, 200]):
            for j, delay in enumerate([0.02, 0.04, 0.08]):
                rows = [
                    cases[k]
                    for k in manifest["groups"]["feedback"]
                    if cases[k]["config"]["hz"] == hz
                    and cases[k]["config"]["delay"] == delay
                    and cases[k]["config"]["feedback"] == mode
                ]
                matrix[i, j] = sum(r["success"] for r in rows)
        ax.imshow(matrix, vmin=0, vmax=3, cmap="Blues")
        for i in range(3):
            for j in range(3):
                ax.text(
                    j,
                    i,
                    f"{int(matrix[i, j])}/3",
                    ha="center",
                    va="center",
                    color="white" if matrix[i, j] >= 2 else "black",
                    fontsize=15,
                )
        ax.set_xticks(range(3), ["20 ms", "40 ms", "80 ms"])
        ax.set_yticks(range(3), ["50 Hz", "100 Hz", "200 Hz"])
        ax.set_title(title)
    fig.suptitle("40 N前向推扰；1 mm位置噪声、0.01 m/s速度噪声；每格3个噪声种子")
    fig.savefig(out / "delay.png", dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(
        1, 3, figsize=(12, 3.6), sharey=True, constrained_layout=True
    )
    for ax, ds in zip(axes, [0.06, 0.12, 0.20]):
        a = audit(
            mapping="coupled",
            hz=100,
            push=40.0,
            ds=ds,
            steps=8,
            torque_rate=0.0,
            feedback="ideal",
            dt=0.001,
        )
        mask = (a["time"] >= 1.08) & (a["time"] <= 1.4)
        w = a["measured_wrench"]
        fraction = w[:, 0, 1] / np.maximum(w[:, :, 1].sum(axis=1), 1)
        ax.plot(
            a["time"][mask],
            a["desired_loads"][mask, 0],
            "k--",
            label="计划左足负载比例",
        )
        ax.plot(a["time"][mask], fraction[mask], label="实际比例")
        ax.set_title(f"双支撑 {ds:.2f} s")
        ax.grid(alpha=0.2)
        ax.set_xlabel("时间（s）")
    axes[0].set_ylabel("左足法向力 / 两足法向力")
    axes[0].legend(fontsize=8)
    fig.savefig(out / "double_support.png", dpi=160)
    plt.close(fig)
    key = find(steps=20, push=40.0)
    x = np.load(root / key / "trace.npz")["data"]
    a = np.load(root / key / "audit.npz")
    fig, axes = plt.subplots(
        3, 1, figsize=(11, 8), sharex=True, constrained_layout=True
    )
    for side, label in enumerate(["左足", "右足"]):
        axes[0].plot(x[:, 0], x[:, 4 + side * 2], label=label + "前进位置")
        axes[1].plot(x[:, 0], 100 * x[:, 5 + side * 2], label=label + "离地高度")
        axes[2].plot(x[:, 0], x[:, 8 + side], label=label + "实际载荷")
    for ax, ylabel in zip(axes, ["足底x（m）", "足底z（cm）", "法向力（N）"]):
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.2)
        ax.legend(loc="upper left", fontsize=8)
    axes[-1].set_xlabel("时间（s）")
    fig.savefig(out / "twenty_steps.png", dpi=160)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 3.6), constrained_layout=True)
    for hz in [50, 100, 200]:
        rows = [
            cases[
                find(
                    hz=hz,
                    push=40.0,
                    torque_rate=rate,
                    steps=8,
                    feedback="ideal",
                    ds=0.12,
                    dt=0.001,
                )
            ]
            for rate in [0.0, 600.0, 1500.0]
        ]
        ax.plot(
            ["不限制", "600 Nm/s", "1500 Nm/s"],
            [r["torque_rate_max_Nm_s"] for r in rows],
            marker="o",
            label=f"{hz} Hz",
        )
    ax.set_ylabel("实际施加指令最大变化率（Nm/s）")
    ax.set_title("0.4秒后启用限制，启动静态承重阶段单独处理")
    ax.grid(alpha=0.2)
    ax.legend()
    fig.savefig(out / "slew.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
