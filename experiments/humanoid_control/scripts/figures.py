"""Plots from physical state and contact records, including failed stress cases."""

import json
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from model import R

plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False


def main():
    m = json.loads((R / "manifest.json").read_text())
    root = R / m["root"]
    dest = R / "figures"
    dest.mkdir(exist_ok=True)

    def find(**values):
        return next(
            k
            for k, c in m["configs"].items()
            if all(c.get(name) == value for name, value in values.items())
        )

    def load(**values):
        return np.load(root / find(**values) / "trace.npz")

    a = load(
        scenario="walk",
        backend="clarabel",
        walk_steps=4,
        step_distance=0.06,
        hz=100,
        push_x=0.0,
        push_y=0.0,
    )
    fig, axes = plt.subplots(
        3, 1, figsize=(11, 7.5), sharex=True, constrained_layout=True
    )
    for side, label in enumerate(["左足", "右足"]):
        axes[0].plot(a["time"], a["feet"][:, side, 0], label=label)
        axes[1].plot(a["time"], 100 * a["feet"][:, side, 2], label=label)
        axes[2].plot(a["time"], a["actual_wrench"][:, side, 2], label=label)
    for ax, label in zip(axes, ["足底x（m）", "离地高度（cm）", "实际法向力（N）"]):
        ax.set_ylabel(label)
        ax.legend()
        ax.grid(alpha=0.2)
    axes[-1].set_xlabel("仿真时间（s）")
    fig.savefig(dest / "walking_contacts.png", dpi=160)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 4.5), constrained_layout=True)
    ax.plot(a["com"][:, 0], a["com"][:, 1], label="实际质心轨迹", color="#254f88")
    ax.plot(
        a["reference_com"][:, 0],
        a["reference_com"][:, 1],
        "--",
        label="质心参考",
        alpha=0.6,
    )
    for k in range(5):
        at = min(np.searchsorted(a["time"], 7 * k + 6.5), len(a["time"]) - 1)
        for side, color in [(0, "#3885ba"), (1, "#cb7744")]:
            x, y = a["feet"][at, side, :2]
            ax.add_patch(
                Rectangle(
                    (x - 0.075, y - 0.02),
                    0.15,
                    0.04,
                    facecolor=color,
                    alpha=0.12,
                    edgecolor=color,
                )
            )
            ax.plot(x, y, "o", color=color)
    ax.set_aspect("equal")
    ax.set_xlabel("前进x（m）")
    ax.set_ylabel("横向y（m）")
    ax.grid(alpha=0.2)
    ax.legend()
    fig.savefig(dest / "com_top.png", dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(
        2, 2, figsize=(11, 6), sharex=True, constrained_layout=True
    )
    for col, support in enumerate([0, 1]):
        a = load(
            scenario="single",
            backend="clarabel",
            support_side=support,
            hz=100,
            push_x=0.0,
            push_y=0.0,
        )
        for side, label in enumerate(["左足", "右足"]):
            axes[0, col].plot(a["time"], 100 * a["feet"][:, side, 2], label=label)
            axes[1, col].plot(a["time"], a["actual_wrench"][:, side, 2], label=label)
        axes[0, col].set_title(["左脚支撑，右脚抬起", "右脚支撑，左脚抬起"][col])
        axes[0, col].set_ylabel("高度（cm）")
        axes[1, col].set_ylabel("实际法向力（N）")
        axes[1, col].set_xlabel("时间（s）")
        for ax in axes[:, col]:
            ax.grid(alpha=0.2)
            ax.legend()
    fig.savefig(dest / "single_support.png", dpi=160)
    plt.close(fig)
    stress_m = json.loads((R / "stress_manifest.json").read_text())
    fig, axes = plt.subplots(
        2, 1, figsize=(10, 6), sharex=True, constrained_layout=True
    )
    for force in [40, 150, 250]:
        if force == 40:
            a = load(
                scenario="stand",
                backend="clarabel",
                hz=100,
                push_x=0.0,
                push_y=force,
                friction=0.6,
                initial_yaw=0.0,
            )
        else:
            key = next(
                k
                for k, c in stress_m["configs"].items()
                if c["scenario"] == "stand" and c["push_y"] == force
            )
            a = np.load(R / stress_m["root"] / key / "trace.npz")
        axes[0].plot(a["time"], 100 * a["com"][:, 1], label=f"{force} N")
        force_z = a["actual_wrench"][:, :, 2]
        axes[1].plot(
            a["time"],
            force_z[:, 0] / np.maximum(force_z.sum(axis=1), 1),
            label=f"{force} N",
        )
    for ax in axes:
        ax.axvspan(2.7, 2.8, color="orange", alpha=0.2)
        ax.legend()
        ax.grid(alpha=0.2)
    axes[0].set_ylabel("横向CoM y（cm）")
    axes[1].set_ylabel("左足实际负载比例")
    axes[1].set_xlabel("时间（s）")
    fig.savefig(dest / "push_boundary.png", dpi=160)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(9, 4.5), constrained_layout=True)
    for backend, label in [("osqp", "OSQP"), ("clarabel", "Clarabel")]:
        key = find(
            scenario="single",
            backend=backend,
            hz=100,
            support_side=0,
            push_x=0.0,
            push_y=0.0,
        )
        c = np.load(root / key / "controller.npz")
        milliseconds = np.sort(c["solve_seconds"] * 1000)
        ax.plot(
            milliseconds,
            np.arange(1, len(milliseconds) + 1) / len(milliseconds),
            label=label,
        )
    ax.axvline(10, color="red", ls="--", lw=1, label="100 Hz时的10 ms参考预算")
    ax.set_xlabel("全身控制一次调用耗时（ms）")
    ax.set_ylabel("累计比例")
    ax.grid(alpha=0.2)
    ax.legend()
    fig.savefig(dest / "backend_timing.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
