"""Physical G1 trials; no base actuators, state resets or applied support wrenches."""

from dataclasses import dataclass, asdict
from pathlib import Path
import argparse, json, time
import numpy as np
from scipy.spatial.transform import Rotation
import mujoco, osqp
from model import R, build, initialize
from controller import WholeBody
from storage import atomic_json, chunk


@dataclass(frozen=True)
class Config:
    scenario: str = "stand"
    duration: float = 6.0
    hz: int = 100
    dt: float = 0.001
    push_x: float = 0.0
    push_y: float = 0.0
    push_time: float = 2.7
    push_duration: float = 0.1
    friction: float = 0.6
    shift: float = 0.06
    lift: float = 0.05
    step_distance: float = 0.06
    walk_steps: int = 4
    backend: str = "clarabel"
    support_side: int = 0
    initial_yaw: float = 0.0


def smooth(t, start, duration):
    u = float(np.clip((t - start) / duration, 0, 1))
    return (
        u**3 * (10 - 15 * u + 6 * u * u),
        30 * u * u * (1 - u) ** 2 / duration,
        (60 * u - 180 * u * u + 120 * u**3) / duration**2 if 0 < u < 1 else 0.0,
    )


class Reference:
    def __init__(self, data, ctl, cfg):
        self.cfg = cfg
        self.com = data.subtree_com[ctl.root].copy()
        self.feet = data.site_xpos[ctl.sites].copy()
        self.rotations = [data.site_xmat[i].reshape(3, 3).copy() for i in ctl.sites]

    def at(self, t):
        cfg = self.cfg
        c = self.com.copy()
        v = np.zeros(3)
        a = np.zeros(3)
        feet = self.feet.copy()
        fv = np.zeros((2, 3))
        fa = fv.copy()
        contacts = [0, 1]
        rotations = [r.copy() for r in self.rotations]
        if cfg.scenario == "shift":
            if t > 0.5:
                # Smooth finite lateral move and return, no sinusoidal startup jump.
                s, ds, dds = smooth(t, 0.5, 1.5)
                r, dr, ddr = smooth(t, 3.5, 1.5)
                c[1] += cfg.shift * (s - r)
                v[1] = cfg.shift * (ds - dr)
                a[1] = cfg.shift * (dds - ddr)
        elif cfg.scenario in ["single", "step"]:
            stance = cfg.support_side
            swing = 1 - stance
            s, ds, dds = smooth(t, 0.5, 1.5)
            r, dr, ddr = smooth(t, 5.2, 1.3)
            target = self.feet[stance, :2] - self.com[:2]
            c[:2] += target * (s - r)
            v[:2] = target * (ds - dr)
            a[:2] = target * (dds - ddr)
            up, dup, ddup = smooth(t, 1.6, 0.6)
            down, ddown, dddown = smooth(t, 4.0, 0.6)
            feet[swing, 2] += cfg.lift * (up - down)
            fv[swing, 2] = cfg.lift * (dup - ddown)
            fa[swing, 2] = cfg.lift * (ddup - dddown)
            if cfg.scenario == "step":
                progress, dprogress, ddprogress = smooth(t, 1.6, 3.0)
                feet[swing, 0] += cfg.step_distance * progress
                fv[swing, 0] = cfg.step_distance * dprogress
                fa[swing, 0] = cfg.step_distance * ddprogress
                c[0] += 0.5 * cfg.step_distance * r
                v[0] += 0.5 * cfg.step_distance * dr
                a[0] += 0.5 * cfg.step_distance * ddr
            if 1.6 <= t < 4.6:
                contacts = [stance]
                rotations[swing] = Rotation.from_rotvec(
                    [(0.12 if swing == 0 else -0.12) * (up - down), 0, 0]
                ).as_matrix()
        elif cfg.scenario == "walk":
            phase = min(int(t / 7.0), cfg.walk_steps)
            for k in range(phase):
                side = 1 - k % 2
                feet[side, 0] = self.feet[side, 0] + (k + 1) * cfg.step_distance
            offset = self.com[:2] - self.feet[:, :2].mean(axis=0)
            baseline = feet[:, :2].mean(axis=0) + offset
            c[:2] = baseline
            if phase < cfg.walk_steps:
                local = t - 7 * phase
                swing = 1 - phase % 2
                stance = 1 - swing
                s, ds, dds = smooth(local, 0.5, 1.5)
                r, dr, ddr = smooth(local, 5.2, 1.3)
                target = feet[stance, :2] - baseline
                delta = (
                    (phase + 1) * cfg.step_distance
                    + self.feet[swing, 0]
                    - feet[swing, 0]
                )
                c[:2] += target * (s - r)
                v[:2] = target * (ds - dr)
                a[:2] = target * (dds - ddr)
                c[0] += 0.5 * delta * r
                v[0] += 0.5 * delta * dr
                a[0] += 0.5 * delta * ddr
                progress, dp, ddp = smooth(local, 1.6, 3.0)
                up, dup, ddup = smooth(local, 1.6, 0.6)
                down, ddown, dddown = smooth(local, 4.0, 0.6)
                feet[swing, 0] += delta * progress
                fv[swing, 0] = delta * dp
                fa[swing, 0] = delta * ddp
                feet[swing, 2] += cfg.lift * (up - down)
                fv[swing, 2] = cfg.lift * (dup - ddown)
                fa[swing, 2] = cfg.lift * (ddup - dddown)
                if 1.6 <= local < 4.6:
                    contacts = [stance]
                    rotations[swing] = Rotation.from_rotvec(
                        [(0.12 if swing == 0 else -0.12) * (up - down), 0, 0]
                    ).as_matrix()
        elif cfg.scenario != "stand":
            raise ValueError(cfg.scenario)
        fraction = float(
            np.clip(
                (c[1] - self.feet[1, 1]) / (self.feet[0, 1] - self.feet[1, 1]), 0, 1
            )
        )
        return c, v, a, feet, fv, fa, rotations, contacts, [fraction, 1 - fraction]


def wrenches(model, data, ctl):
    out = np.zeros((2, 6))
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    foot_bodies = [model.site_bodyid[s] for s in ctl.sites]
    for i in range(data.ncon):
        contact = data.contact[i]
        for side, body in enumerate(foot_bodies):
            pair = [contact.geom1, contact.geom2]
            if floor not in pair:
                continue
            other = pair[1] if pair[0] == floor else pair[0]
            if model.geom_bodyid[other] != body:
                continue
            local = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, local)
            sign = 1 if contact.geom2 == other else -1
            rot = contact.frame.reshape(3, 3).T
            f = sign * rot @ local[:3]
            moment = sign * rot @ local[3:] + np.cross(
                contact.pos - data.site_xpos[ctl.sites[side]], f
            )
            out[side] += np.r_[f, moment]
    return out


def simulate(cfg, out, on_frame=None):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    if (out / "result.json").exists():
        raise FileExistsError(out)
    model = mujoco.MjModel.from_xml_path(str(R / "models/scene.xml"))
    model.opt.timestep = cfg.dt
    # Match all contact geom friction; the upstream foot priority otherwise overrides floor friction.
    model.geom_friction[:, 0] = cfg.friction
    data = initialize(model)
    if cfg.initial_yaw:
        data.qpos[3:7] = [
            np.cos(cfg.initial_yaw / 2),
            0,
            0,
            np.sin(cfg.initial_yaw / 2),
        ]
        mujoco.mj_forward(model, data)
    ctl = WholeBody(model, data, cfg.hz, cfg.friction, cfg.backend)
    ref = Reference(data, ctl, cfg)
    tau = np.zeros(model.nu)
    predicted = np.zeros((2, 6))
    failure = None
    trace = []
    updates = []
    times = []
    initial = np.r_[data.qpos.copy(), data.qvel.copy()]
    atomic_json(
        out / "result.json", dict(status="running", config=asdict(cfg), steps=0)
    )
    checkpoint_at = 0
    for k in range(round(cfg.duration / cfg.dt)):
        t = k * cfg.dt
        mujoco.mj_forward(model, data)
        actual = wrenches(model, data, ctl)
        contact_ctrl = data.ctrl.copy()
        contact_external = data.xfrc_applied[ctl.root].copy()
        goals = ref.at(t)
        if k % round(1 / (cfg.hz * cfg.dt)) == 0:
            started = time.perf_counter()
            try:
                tau, predicted = ctl.solve(*goals)
            except (RuntimeError, ValueError, osqp.OSQPException) as exc:
                failure = str(exc)
                break
            times.append(time.perf_counter() - started)
            updates.append(dict(time=t, solve_seconds=times[-1], **ctl.last))
        data.ctrl[:] = tau
        data.xfrc_applied[:] = 0
        if cfg.push_time <= t < cfg.push_time + cfg.push_duration:
            data.xfrc_applied[ctl.root, :2] = [cfg.push_x, cfg.push_y]
        # Only the declared perturbation is externally applied; no support forces.
        trace.append(
            dict(
                time=t,
                qpos=data.qpos.copy(),
                qvel=data.qvel.copy(),
                ctrl=tau.copy(),
                contact_ctrl=contact_ctrl,
                contact_external=contact_external,
                com=data.subtree_com[ctl.root].copy(),
                feet=data.site_xpos[ctl.sites].copy(),
                actual_wrench=actual.copy(),
                predicted_wrench=predicted.copy(),
                reference_com=goals[0].copy(),
                reference_feet=goals[3].copy(),
                contacts=np.array([i in goals[7] for i in range(2)]),
            )
        )
        tilt = np.linalg.norm(data.qpos[4:6])
        if tilt > 0.25 or data.subtree_com[ctl.root, 2] < 0.48:
            failure = "body_tilt_or_height"
            break
        if np.max(np.abs(data.qvel[6:])) > 12:
            failure = "joint_velocity"
            break
        if on_frame is not None and k % round(0.05 / cfg.dt) == 0:
            on_frame(model, data, t)
        mujoco.mj_step(model, data)
        if len(trace) - checkpoint_at >= round(1 / cfg.dt):
            chunk(out / "chunks", checkpoint_at, trace[checkpoint_at:])
            checkpoint_at = len(trace)
            atomic_json(
                out / "result.json",
                dict(
                    status="running",
                    config=asdict(cfg),
                    steps=len(trace),
                    duration=float(data.time),
                ),
            )
    mujoco.mj_forward(model, data)
    result = dict(
        config=asdict(cfg),
        success=failure is None,
        failure=failure,
        duration=float(data.time),
        mass=float(model.body_mass.sum()),
        nq=model.nq,
        nv=model.nv,
        nu=model.nu,
        steps=len(trace),
        updates=len(updates),
        qp_p95_ms=float(np.quantile(times, 0.95) * 1000) if times else None,
        max_qp_residual=max([u["dynamics_residual"] for u in updates], default=None),
    )
    if trace:
        arrays = {key: np.array([r[key] for r in trace]) for key in trace[0]}
        np.savez_compressed(out / "trace.npz", initial=initial, **arrays)
        from metrics import evaluate

        result.update(evaluate(arrays, cfg, float(model.body_mass.sum())))
        if result["success"] and not result["criteria_passed"]:
            result["success"] = False
            result["failure"] = "physical_acceptance"
        result["com_rmse"] = np.sqrt(
            np.mean((arrays["com"] - arrays["reference_com"]) ** 2, axis=0)
        ).tolist()
        result["max_foot_height"] = arrays["feet"][:, :, 2].max(axis=0).tolist()
        result["min_com_height"] = float(arrays["com"][:, 2].min())
        result["max_joint_speed"] = float(np.max(np.abs(arrays["qvel"][:, 6:])))
    if updates:
        np.savez_compressed(
            out / "controller.npz",
            **{key: np.array([u[key] for u in updates]) for key in updates[0]},
        )
    result["status"] = "success" if result["success"] else "failed"
    if len(trace) > checkpoint_at:
        chunk(out / "chunks", checkpoint_at, trace[checkpoint_at:])
    atomic_json(out / "result.json", result)
    print(json.dumps(result, indent=2), flush=True)
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("name")
    p.add_argument("--config", default="{}")
    a = p.parse_args()
    if not (R / "models/scene.xml").exists():
        build()
    simulate(Config(**json.loads(a.config)), R / "pilots" / a.name)
