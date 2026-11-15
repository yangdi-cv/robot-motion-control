"""Publication plots from measured CSV; no generated or interpolated force samples."""

from pathlib import Path
import json, numpy as np, matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

R = Path(__file__).resolve().parents[1]
F = R / "figures"
F.mkdir(exist_ok=True)
plt.rcParams.update(
    {"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "font.size": 10}
)


def data(n):
    d = R / "runs" / n
    x = np.genfromtxt(d / "physics_force.csv", delimiter=",", names=True)
    return x["t"], x["fz_N"]


def plot_group(names, title, stem):
    fig, ax = plt.subplots(3, 1, figsize=(10, 8))
    for n in names:
        if not (R / "runs" / n / "COMPLETE").exists():
            continue
        t, f = data(n)
        stride = round(
            0.01
            / json.loads((R / "runs" / n / "metadata.json").read_text())["physics_dt"]
        )
        b = f.reshape(-1, stride).mean(axis=1)
        bt = t.reshape(-1, stride)[:, -1]
        ax[0].plot(bt, b, label=n, lw=0.9)
        hit = np.flatnonzero(b > 0.5)
        if len(hit):
            contact = bt[hit[0]]
            mask = (t >= contact - 0.05) & (t <= contact + 1)
            ax[1].plot(t[mask] - contact, f[mask], label=n, lw=0.65)
        mask = bt > bt[-1] - 2
        ax[2].plot(bt[mask] - bt[-1], b[mask], label=n, lw=0.8)
    for a in ax:
        a.axhspan(4.5, 5.5, color="green", alpha=0.12)
        a.axhline(5, c="black", ls="--", lw=0.7)
        a.grid(alpha=0.2)
        a.legend(fontsize=7)
    ax[0].set(ylabel="10ms 均力 / N", xlabel="仿真时间 / s", title=title)
    ax[1].set(ylabel="1ms 原始力 / N", xlabel="相对首次接触时间 / s")
    ax[2].set(ylabel="末段 10ms 均力 / N", xlabel="相对结束时间 / s")
    fig.tight_layout()
    fig.savefig(F / f"{stem}.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    plot_group(
        ["no_gravity_probe", "gravity_probe"], "重力与补偿：相同探针场景", "gravity"
    )
    plot_group(
        ["gravity_gripper", "gripper_ramp", "gripper_brake"],
        "刚性夹爪：力缓升与接近减速",
        "approach",
    )
    plot_group(["gripper_brake", "gripper_soft"], "刚性与柔顺材质分组对照", "material")
    plot_group(
        [
            "validation_higher",
            "validation_lower",
            "validation_friction_low",
            "validation_friction_high",
        ],
        "刚性场景的参数扰动",
        "validation",
    )

    plot_group(
        [
            "refine_velocity",
            "refine_braking",
            "refine_combined",
            "refine_damping100",
            "refine_damping300",
        ],
        "修正速度、提前减速和法向阻尼：并非越大越好",
        "refinement",
    )
    plot_group(
        [
            "final_selected",
            "final_higher",
            "final_lower",
            "final_higher5mm",
            "final_lower5mm",
            "final_friction_low",
            "final_friction_high",
        ],
        "统一方案：刚性材质的高度与摩擦扰动",
        "final_validation",
    )
    plot_group(
        ["final_selected", "final_half_step"],
        "物理步长减半检查：评价仍按固定10ms分箱",
        "timestep",
    )
