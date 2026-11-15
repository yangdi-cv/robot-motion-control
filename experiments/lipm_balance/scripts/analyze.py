"""Grouped statistics, delayed closed-loop poles and publication figures."""

from pathlib import Path
import json
from collections import Counter
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from core import Config, CONTROLLERS, transition, gains, key

R = Path(__file__).resolve().parents[1]
F = R / "figures"
F.mkdir(exist_ok=True)
LABELS = {
    "pd": "PD",
    "lqr": "LQR",
    "capture": "LQR＋落脚调整",
    "lqr_predict": "LQR＋延迟预测",
    "capture_predict": "LQR＋预测＋落脚调整",
    "open_loop": "无反馈",
}
COLORS = {
    "pd": "#cd754c",
    "lqr": "#7759aa",
    "capture": "#21849b",
    "lqr_predict": "#a19635",
    "capture_predict": "#22834c",
    "open_loop": "#999999",
}
plt.rcParams.update(
    {"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "font.size": 10}
)


def summarize(values):
    success = [x for x in values if x["success"]]
    recovery = [x["recovery_s"] for x in success if x["recovery_s"] is not None]
    return {
        "n": len(values),
        "successes": len(success),
        "rate": len(success) / len(values) if values else None,
        "recovery_mean_s": float(np.mean(recovery)) if recovery else None,
        "recovery_n": len(recovery),
        "successful_route_error_mean_m": (
            float(np.mean([x["terminal_route_error_m"] for x in success]))
            if success
            else None
        ),
        "reasons": dict(Counter(x["reason"] for x in values if not x["success"])),
    }


def select(values, **criteria):
    return [
        v for v in values if all(v["config"].get(k) == x for k, x in criteria.items())
    ]


def spectral_radius(method, hz, delay):
    A, B = transition(0.8, 1 / hz)
    lag = round(delay * hz)
    if method == "pd":
        w2 = 9.81 / 0.8
        L = np.array([1 + 9 / w2, 5.4 / w2])
    else:
        L = -gains(0.8)[0]
    M = np.zeros((2 * (lag + 1), 2 * (lag + 1)))
    M[:2, :2] = A
    M[:2, -2:] += B[:, None] * L
    if lag:
        M[2:, :-2] = np.eye(2 * lag)
    return float(np.max(np.abs(np.linalg.eigvals(M))))


def trace(manifest, cfg):
    return np.load(R / manifest["root"] / key(cfg) / "trace.npz")["data"]


def savefig(fig, name):
    fig.tight_layout()
    fig.savefig(F / name, dpi=170)
    plt.close(fig)


if __name__ == "__main__":
    data = json.loads((R / "results.json").read_text())
    manifest = json.loads((R / "manifest.json").read_text())
    groups = {g: [data[k] for k in ids] for g, ids in manifest["groups"].items()}
    summary = {
        "unique_runs": len(data),
        "successes": sum(x["success"] for x in data.values()),
        "integrated_time_s": sum(x["duration_s"] for x in data.values()),
        "groups": {g: summarize(v) for g, v in groups.items()},
        "grid": {},
        "boundary": {},
        "domain": {},
        "phase": {},
        "gait": {},
        "nominal": {},
    }
    grid = groups["grid"]
    boundary = groups["boundary"]
    for m in CONTROLLERS:
        summary["grid"][m] = {}
        summary["boundary"][m] = {}
        for hz in [50, 100, 200]:
            for delay in [0.0, 0.02, 0.04, 0.08, 0.12]:
                for j in [0.0, 6.0, 12.0]:
                    summary["grid"][m][f"{hz}_{delay}_{j}"] = summarize(
                        select(grid, controller=m, hz=hz, delay=delay, impulse_x=j)
                    )
        for delay in [0.0, 0.04, 0.08, 0.12]:
            cells = {
                str(j): summarize(
                    select(boundary, controller=m, delay=delay, impulse_x=j)
                )
                for j in [0.0, 3.0, 6.0, 9.0, 12.0, 15.0]
            }
            robust = None
            for j in [0.0, 3.0, 6.0, 9.0, 12.0, 15.0]:
                if cells[str(j)]["successes"] == cells[str(j)]["n"]:
                    robust = j
                else:
                    break
            summary["boundary"][m][str(delay)] = {
                "cells": cells,
                "contiguous_all_seed_impulse_Ns": robust,
            }
        for j in [0.0, 6.0, 12.0]:
            summary["nominal"][m + "_" + str(j)] = summarize(
                select(grid, controller=m, hz=100, delay=0.0, impulse_x=j)
            )
        summary["domain"][m] = summarize(select(groups["domain"], controller=m))
        summary["phase"][m] = summarize(
            select(groups["phase_validation"], controller=m)
        )
        summary["gait"][m] = summarize(select(groups["gait_validation"], controller=m))
    summary["spectral"] = {
        m: {
            str(hz): {
                str(d): spectral_radius(m, hz, d) for d in [0.0, 0.02, 0.04, 0.08, 0.12]
            }
            for hz in [50, 100, 200]
        }
        for m in ["pd", "lqr"]
    }
    (R / "summary.json").write_text(json.dumps(summary, indent=2))
    # Continuous stepping with one matched push; failed traces stop at their real end.
    fig, axs = plt.subplots(3, 1, figsize=(11, 9))
    for m in ["pd", "lqr", "capture"]:
        x = trace(
            manifest,
            Config(
                controller=m, impulse_x=12.0, noise_position=0.0, noise_velocity=0.0
            ),
        )
        axs[0].plot(x[:, 0], x[:, 1], c=COLORS[m], label=LABELS[m])
        axs[0].plot(x[:, 0], x[:, 11], c=COLORS[m], ls=":", lw=0.7)
        axs[1].plot(
            x[:, 0],
            x[:, 13] - (x[:, 5] + x[:, 6] / np.sqrt(9.81 / 0.8) + x[:, 15]),
            c=COLORS[m],
            label=LABELS[m],
        )
        axs[2].plot(x[:, 0], x[:, 15], c=COLORS[m], label=LABELS[m])
    for ax in axs:
        ax.axvspan(2.7, 2.8, color="red", alpha=0.12)
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8)
    axs[0].set(ylabel="前向位置 / m", title="同一12N·s推扰：实线为质心，虚线为支撑脚")
    axs[1].set(ylabel="前向DCM误差 / m")
    axs[2].set(ylabel="路线平移 / m", xlabel="模型时间 / s")
    savefig(fig, "stepping.png")
    fig, axs = plt.subplots(1, 5, figsize=(16, 3.9), sharey=True)
    for ax, m in zip(axs, CONTROLLERS):
        arr = np.array(
            [
                [
                    summarize(
                        select(grid, controller=m, hz=hz, delay=d, impulse_x=12.0)
                    )["rate"]
                    for d in [0.0, 0.02, 0.04, 0.08, 0.12]
                ]
                for hz in [50, 100, 200]
            ]
        )
        image = ax.imshow(arr, vmin=0, vmax=1, cmap="RdYlGn", aspect="auto")
        for i in range(3):
            for j in range(5):
                ax.text(
                    j, i, f"{arr[i,j]*100:.0f}", ha="center", va="center", fontsize=8
                )
        ax.set_xticks(range(5), [0, 20, 40, 80, 120])
        ax.set_yticks(range(3), [50, 100, 200])
        ax.set_title(LABELS[m], fontsize=9)
        ax.set_xlabel("反馈延迟 / ms")
    axs[0].set_ylabel("控制频率 / Hz")
    fig.suptitle("12N·s推扰下成功率（%；每格3个噪声种子）", fontsize=12)
    savefig(fig, "frequency_delay.png")
    fig, axs = plt.subplots(1, 4, figsize=(14, 3.8), sharey=True)
    for ax, delay in zip(axs, [0.0, 0.04, 0.08, 0.12]):
        for m in CONTROLLERS:
            y = [
                summary["boundary"][m][str(delay)]["cells"][str(float(j))]["rate"]
                for j in [0, 3, 6, 9, 12, 15]
            ]
            ax.plot(
                [0, 3, 6, 9, 12, 15], y, "o-", c=COLORS[m], label=LABELS[m], ms=3, lw=1
            )
        ax.set(
            title=f"延迟 {delay*1000:.0f}ms",
            xlabel="前向推扰冲量 / N·s",
            ylim=(-0.05, 1.05),
        )
        ax.grid(alpha=0.2)
    axs[0].set_ylabel("验证种子成功比例")
    axs[-1].legend(fontsize=6)
    savefig(fig, "disturbance_boundary.png")
    fig, axs = plt.subplots(1, 2, figsize=(10, 4))
    for m in ["pd", "lqr"]:
        for hz in [50, 100, 200]:
            ds = np.arange(0, 0.141, 1 / hz)
            axs[0].plot(
                ds * 1000,
                [spectral_radius(m, hz, d) for d in ds],
                label=f"{LABELS[m]} {hz}Hz",
                lw=1,
            )
    axs[0].axhline(1, c="red", ls="--")
    axs[0].set(
        xlabel="反馈延迟 / ms",
        ylabel="离散闭环谱半径",
        title="固定支撑、无限幅的线性分析",
    )
    axs[0].legend(fontsize=7)
    axs[0].grid(alpha=0.2)
    for m in ["capture", "capture_predict"]:
        x = trace(manifest, Config(controller=m, impulse_x=12.0, delay=0.08, seed=1))
        axs[1].plot(x[:, 0], x[:, 1] - x[:, 5] - x[:, 15], label=LABELS[m], c=COLORS[m])
    axs[1].axvspan(2.7, 2.8, color="red", alpha=0.12)
    axs[1].set(
        xlabel="模型时间 / s",
        ylabel="前向跟踪误差 / m",
        title="80ms延迟：落脚调整与预测的组合",
    )
    axs[1].legend(fontsize=8)
    axs[1].grid(alpha=0.2)
    savefig(fig, "delay_analysis.png")
    # Domain, push phase and cadence validation, all on held-out seeds.
    fig, axs = plt.subplots(1, 3, figsize=(13, 4), sharey=True)
    for ax, name, title in zip(
        axs,
        ["domain", "phase", "gait"],
        ["高度/方向/摩擦/噪声/偏置", "推扰发生时刻", "支撑周期0.5/0.7s"],
    ):
        vals = [summary[name][m]["rate"] for m in CONTROLLERS]
        ax.bar(range(5), vals, color=[COLORS[m] for m in CONTROLLERS])
        ax.set_xticks(
            range(5),
            [LABELS[m] for m in CONTROLLERS],
            rotation=25,
            ha="right",
            fontsize=8,
        )
        ax.set(title=title, ylim=(0, 1.05))
        ax.grid(axis="y", alpha=0.2)
    axs[0].set_ylabel("验证成功比例")
    savefig(fig, "heldout.png")
    physical = json.loads((R / "physical_results.json").read_text())
    for v in physical.values():
        v["standing_success"] = bool(
            v["success"]
            and v["max_foot_translation_m"] <= 0.05
            and v["max_foot_pitch_deg"] <= 5.0
        )
        v["standing_reason"] = (
            (v["reason"] or "terminal_posture_not_recovered")
            if not v["success"]
            else (
                "foot_translation_limit"
                if v["max_foot_translation_m"] > 0.05
                else ("foot_pitch_limit" if v["max_foot_pitch_deg"] > 5 else None)
            )
        )
    psummary = {
        "n": len(physical),
        "posture_successes": sum(v["success"] for v in physical.values()),
        "standing_successes": sum(v["standing_success"] for v in physical.values()),
        "nominal": {},
        "delay": {},
    }
    for m in ["pd", "lqr"]:
        for force in [0.0, 40.0, 80.0, 120.0]:
            vals = [
                v
                for v in physical.values()
                if v["config"]["controller"] == m
                and v["config"]["hz"] == 100
                and v["config"]["delay"] == 0.0
                and v["config"]["force"] == force
                and v["config"]["noise"] == 0.002
                and v["config"]["friction"] == 0.8
            ]
            recoveries = [
                v["recovery_s"]
                for v in vals
                if v["standing_success"] and v["recovery_s"] is not None
            ]
            psummary["nominal"][m + "_" + str(force)] = {
                "n": len(vals),
                "successes": sum(v["standing_success"] for v in vals),
                "recovery_mean_s": float(np.mean(recoveries)) if recoveries else None,
                "foot_max_m": max(v["max_foot_translation_m"] for v in vals),
                "max_angle_deg": max(v["max_angle_deg"] for v in vals),
            }
        psummary["delay"][m] = {}
        for delay in [0.0, 0.04, 0.08, 0.12]:
            vals = [
                v
                for v in physical.values()
                if v["config"]["controller"] == m
                and v["config"]["hz"] == 100
                and v["config"]["delay"] == delay
                and v["config"]["force"] == 80.0
                and v["config"]["noise"] == 0.002
                and v["config"]["friction"] == 0.8
            ]
            psummary["delay"][m][str(delay)] = {
                "n": len(vals),
                "successes": sum(v["standing_success"] for v in vals),
            }
    (R / "physical_evaluation.json").write_text(json.dumps(physical, indent=2))
    (R / "physical_summary.json").write_text(json.dumps(psummary, indent=2))
    pm = json.loads((R / "physical_manifest.json").read_text())
    fig, axs = plt.subplots(4, 1, figsize=(10, 10))
    for method in ["pd", "lqr"]:
        candidates = [
            k
            for k, v in physical.items()
            if v["config"]["controller"] == method
            and v["config"]["hz"] == 100
            and v["config"]["delay"] == 0
            and v["config"]["force"] == 80.0
            and v["config"]["noise"] == 0.002
            and v["config"]["seed"] == 0
            and v["config"]["friction"] == 0.8
        ]
        x = np.load(R / pm["root"] / candidates[0] / "trace.npz")["data"]
        axs[0].plot(
            x[:, 0], np.rad2deg(x[:, 1]), label=LABELS[method], c=COLORS[method]
        )
        axs[1].plot(x[:, 0], x[:, 6], c=COLORS[method])
        axs[2].plot(x[:, 0], x[:, 3] * 1000, c=COLORS[method])
        axs[3].plot(x[:, 0], np.rad2deg(x[:, 4]), c=COLORS[method])
    for ax in axs:
        ax.axvspan(2, 2.1, color="red", alpha=0.12)
        ax.grid(alpha=0.2)
    axs[0].set(ylabel="世界倾角 / °", title="非线性有限脚掌站立：80N、0.1s推扰")
    axs[0].legend()
    axs[1].set_ylabel("踝关节力矩 / Nm")
    axs[2].set(ylabel="脚掌前后移动 / mm")
    axs[3].set(ylabel="脚掌翻转 / °", xlabel="MuJoCo仿真时间 / s")
    axs[3].axhline(5, c="red", ls="--", lw=0.8)
    axs[3].axhline(-5, c="red", ls="--", lw=0.8)
    savefig(fig, "physical_balance.png")
    print(
        "LIPM",
        summary["unique_runs"],
        "successes",
        summary["successes"],
        "physical",
        psummary["n"],
        "standing success",
        psummary["standing_successes"],
    )
