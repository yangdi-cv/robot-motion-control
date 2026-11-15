"""MuJoCo finite-foot standing tests. No walking or artificial support relocation."""

import os

os.environ.setdefault("MUJOCO_GL", "egl")
from pathlib import Path
import json, hashlib
from dataclasses import dataclass, asdict
import numpy as np
import mujoco
from scipy.linalg import solve_continuous_are

R = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class PhysicalConfig:
    controller: str = "lqr"
    hz: int = 100
    delay: float = 0.0
    force: float = 40.0
    friction: float = 0.8
    seed: int = 0
    noise: float = 0.0
    initial_angle: float = 0.02
    dt: float = 0.001
    duration: float = 6.0
    record: bool = False


def physical_gains():
    # Approximate pinned-ankle inertia, including pole and finite-size torso.
    moment = 20 * 0.8**2 + 20 * (0.1**2 + 0.08**2) / 3 + 0.5 * 0.8**2 / 3
    gravity_moment = 9.81 * (20 * 0.8 + 0.5 * 0.4)
    A = np.array([[0.0, 1.0], [gravity_moment / moment, 0.0]])
    B = np.array([[0.0], [1 / moment]])
    P = solve_continuous_are(A, B, np.diag([4000.0, 100.0]), np.array([[0.03]]))
    return (B.T @ P / 0.03)[0], moment, gravity_moment


def simulate_physical(cfg, out=None):
    model = mujoco.MjModel.from_xml_path(str(R / "models/ankle.xml"))
    model.opt.timestep = cfg.dt
    model.geom_friction[:, 0] = cfg.friction
    data = mujoco.MjData(model)
    data.qpos[3] = cfg.initial_angle
    mujoco.mj_forward(model, data)
    gain, inertia, gravity_moment = physical_gains()
    if cfg.controller == "pd":
        gain = np.array([400.0, 100.0])
    if cfg.controller == "none":
        gain = np.zeros(2)
    torso = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso")
    stride = round(1 / (cfg.hz * cfg.dt))
    lag = round(cfg.delay / cfg.dt)
    n = round(cfg.duration / cfg.dt)
    rng = np.random.default_rng(cfg.seed)
    noise = rng.normal(size=(n + 1, 2)) * np.array([cfg.noise, 10 * cfg.noise])
    observed = np.zeros((n + 1, 2))
    logs = []
    torque = 0.0
    reason = None
    renderer = None
    frames = []
    if cfg.record:
        renderer = mujoco.Renderer(model, height=720, width=960)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [0, 0, 0.45]
        camera.distance = 2.1
        camera.azimuth = 90
        camera.elevation = -12
    for k in range(n):
        t = k * cfg.dt
        angle = data.qpos[2] + data.qpos[3]
        velocity = data.qvel[2] + data.qvel[3]
        observed[k] = [angle, velocity]
        if k % stride == 0:
            sample = observed[max(0, k - lag)] + noise[max(0, k - lag)]
            torque = float(np.clip(-gain @ sample, -25, 25))
        data.ctrl[0] = torque
        data.xfrc_applied[:] = 0
        if 2.0 <= t < 2.1 - 1e-10:
            data.xfrc_applied[torso, 0] = cfg.force
        fz = fx = moment = 0.0
        for j in range(data.ncon):
            con = data.contact[j]
            if 0 not in (con.geom1, con.geom2):
                continue
            contact = np.zeros(6)
            mujoco.mj_contactForce(model, data, j, contact)
            world = con.frame.reshape(3, 3).T @ contact[:3]
            if world[2] < 0:
                world = -world
            fz += world[2]
            fx += world[0]
            moment += con.pos[0] * world[2]
        cop = moment / fz if fz > 1e-8 else np.nan
        logs.append(
            [
                t,
                angle,
                velocity,
                data.qpos[0],
                data.qpos[2],
                data.xpos[torso, 2],
                torque,
                fx,
                fz,
                cop,
                cfg.force if 2 <= t < 2.1 - 1e-10 else 0.0,
            ]
        )
        if cfg.record and k % round(0.05 / cfg.dt) == 0:
            renderer.update_scene(data, camera)
            frames.append(renderer.render().copy())
        if abs(angle) > 0.35 or data.xpos[torso, 2] < 0.55:
            reason = "tilt_or_height_limit"
            break
        mujoco.mj_step(model, data)
        if abs(data.time - (k + 1) * cfg.dt) > 1e-9:
            raise RuntimeError("MuJoCo clock mismatch")
    if renderer:
        # MuJoCo 2.3.7 predates Renderer.close(). Free the GL resources explicitly.
        renderer._gl_context.make_current()
        renderer._mjr_context.free()
        renderer._gl_context.free()
    x = np.array(logs)
    tail = x[:, 0] >= x[-1, 0] - 0.5
    recovered = None
    good = (np.abs(x[:, 1]) < 0.02) & (np.abs(x[:, 2]) < 0.05)
    for i in np.flatnonzero(x[:, 0] >= 2.1):
        end = i + round(0.5 / cfg.dt)
        if end < len(x) and np.all(good[i : end + 1]):
            recovered = float(x[i, 0] - 2.1)
            break
    result = {
        "config": asdict(cfg),
        "completed": reason is None,
        "success": reason is None and bool(np.all(good[tail])),
        "reason": reason,
        "duration_s": float(data.time),
        "max_angle_deg": float(np.rad2deg(np.max(np.abs(x[:, 1])))),
        "tail_angle_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(x[tail, 1] ** 2)))),
        "max_foot_translation_m": float(np.max(np.abs(x[:, 3]))),
        "max_foot_pitch_deg": float(np.rad2deg(np.max(np.abs(x[:, 4])))),
        "peak_torque_Nm": float(np.max(np.abs(x[:, 6]))),
        "recovery_s": recovered,
        "peak_contact_force_N": float(np.max(x[:, 8])),
        "last_normal_force_N": float(np.mean(x[tail, 8])),
        "gain": gain.tolist(),
        "pinned_ankle_inertia_kgm2": inertia,
        "gravity_moment_per_rad_Nm": gravity_moment,
        "source": "MuJoCo nonlinear finite-foot standing; not walking",
    }
    if out:
        out = Path(out)
        out.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            out / "trace.npz",
            data=x,
            columns=np.array(
                [
                    "t",
                    "angle",
                    "angular_velocity",
                    "foot_x",
                    "foot_pitch",
                    "torso_z",
                    "torque",
                    "Fx",
                    "Fz",
                    "cop_x",
                    "push_force",
                ]
            ),
        )
        (out / "result.json").write_text(json.dumps(result, indent=2))
        (out / "model.xml").write_text((R / "models/ankle.xml").read_text())
        if frames:
            np.save(out / "frames.npy", np.stack(frames))
    return result


if __name__ == "__main__":
    for ctl in ["none", "pd", "lqr"]:
        for force in [0.0, 40.0, 80.0]:
            r = simulate_physical(
                PhysicalConfig(controller=ctl, force=force),
                R / "runs" / f"physical_pilot_{ctl}_{int(force)}",
            )
            print(
                ctl,
                force,
                r["success"],
                r["reason"],
                r["max_angle_deg"],
                r["recovery_s"],
                flush=True,
            )
