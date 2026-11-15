"""Planar floating-base biped: inverse-dynamics QP commands only joint torques.

Contact wrenches are decision variables, never externally applied to the simulator.
"""

import os

os.environ.setdefault("MUJOCO_GL", "egl")
from dataclasses import dataclass, asdict
from pathlib import Path
import numpy as np, json, time
from scipy import sparse
import mujoco, osqp
from trajectory import quintic, evaluate

R = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class BipedConfig:
    walk: bool = True
    step_length: float = 0.12
    step_time: float = 0.6
    ds: float = 0.12
    steps: int = 8
    hz: int = 100
    dt: float = 0.001
    duration: float = 6.0
    push: float = 0.0
    friction: float = 0.8
    torque_scale: float = 1.0
    record: bool = False
    seed: int = 0
    mapping: str = "coupled"
    feedback: str = "ideal"
    delay: float = 0.0
    sensor_hz: int = 100
    position_noise: float = 0.0
    velocity_noise: float = 0.0
    torque_rate: float = 0.0
    contact_guard: bool = True
    clearance: float = 0.06


def leg_ik(hip, sole, pitch=0.0):
    dx = sole[0] - 0.04 - hip[0]
    dz = hip[1] - (sole[1] + 0.05)
    r2 = dx * dx + dz * dz
    if r2 > (0.8 - 1e-5) ** 2 or r2 < 0.05**2:
        raise ValueError("Leg target outside reach")
    knee = np.arccos(np.clip((r2 - 0.32) / 0.32, -1, 1))
    hip_angle = np.arctan2(-dx, dz) - knee / 2 - pitch
    ankle = -(pitch + hip_angle + knee)
    return np.array([hip_angle, knee, ankle])


class Plan:
    def __init__(self, cfg, height, c0):
        self.cfg = cfg
        self.height = height
        self.start = 0.5
        self.end = self.start + cfg.steps * cfg.step_time
        self.times = np.arange(round(cfg.duration / cfg.dt) + 1) * cfg.dt
        self.p = np.zeros(len(self.times))
        for k, t in enumerate(self.times):
            if not cfg.walk or t < self.start:
                continue
            phase = min(int((t - self.start + 1e-10) / cfg.step_time), cfg.steps)
            if phase == 0:
                continue
            old = (phase - 1) * cfg.step_length
            new = (
                phase * cfg.step_length
                if phase < cfg.steps
                else (cfg.steps - 0.5) * cfg.step_length
            )
            age = t - self.start - phase * cfg.step_time
            u = np.clip(age / cfg.ds, 0, 1) if cfg.ds else 1
            self.p[k] = old + (new - old) * u**3 * (10 - 15 * u + 6 * u * u)
        omega = np.sqrt(9.81 / height)
        a = np.exp(-omega * cfg.dt)
        xi = np.empty(len(self.times))
        xi[-1] = self.p[-1]
        for k in range(len(xi) - 2, -1, -1):
            xi[k] = a * xi[k + 1] + (1 - a) * self.p[k]
        eta = 2 * c0 - xi[0]
        self.state = np.empty((len(xi), 2))
        for k in range(len(xi)):
            self.state[k] = [(xi[k] + eta) / 2, omega * (xi[k] - eta) / 2]
            eta = a * eta + (1 - a) * self.p[k]

    def loads(self, t):
        if not self.cfg.walk or t < self.start:
            return [0.5, 0.5]
        phase = min(int((t - self.start + 1e-10) / self.cfg.step_time), self.cfg.steps)
        age = t - self.start - phase * self.cfg.step_time
        u = np.clip(age / self.cfg.ds, 0, 1) if self.cfg.ds > 0 else 1.0
        alpha = u**3 * (10 - 15 * u + 6 * u * u)
        if phase == 0:
            alpha = 0.5 + 0.5 * alpha
        if phase >= self.cfg.steps:
            alpha *= 0.5
        stance = phase % 2
        values = [0.0, 0.0]
        values[stance] = alpha
        values[1 - stance] = 1 - alpha
        return values

    def feet(self, t):
        # During DS both feet keep contact; during SS only the old foot swings.
        if not self.cfg.walk or t < self.start:
            return (
                [np.array([0.0, 0.0]), np.array([0.0, 0.0])],
                [np.zeros(2), np.zeros(2)],
                [np.zeros(2), np.zeros(2)],
                [0, 1],
            )
        cfg = self.cfg
        phase = min(int((t - self.start + 1e-10) / cfg.step_time), cfg.steps)
        age = t - self.start - phase * cfg.step_time
        if phase >= cfg.steps:
            positions = [None, None]
            for side in [0, 1]:
                last = cfg.steps if cfg.steps % 2 == side else cfg.steps - 1
                positions[side] = np.array([max(0, last) * cfg.step_length, 0.0])
            return (
                positions,
                [np.zeros(2), np.zeros(2)],
                [np.zeros(2), np.zeros(2)],
                [0, 1],
            )
        stance = phase % 2
        swing = 1 - stance
        stance_x = phase * cfg.step_length
        previous = max(0, phase - 1) * cfg.step_length
        future = (phase + 1) * cfg.step_length
        positions = [None, None]
        vel = [np.zeros(2), np.zeros(2)]
        acc = [np.zeros(2), np.zeros(2)]
        positions[stance] = np.array([stance_x, 0.0])
        if age < cfg.ds:
            positions[swing] = np.array([previous, 0.0])
            return positions, vel, acc, [0, 1]
        u = (age - cfg.ds) / (cfg.step_time - cfg.ds)
        T = cfg.step_time - cfg.ds
        c = quintic([previous, 0.0], [0.0, 0.0], [0.0, 0.0], [future, 0.0], T)
        p, v, a = evaluate(c, age - cfg.ds, T)
        # Smooth bump with zero endpoint velocity and acceleration.
        p[1] = cfg.clearance * 64 * u**3 * (1 - u) ** 3
        v[1] = cfg.clearance * 64 * 3 * u**2 * (1 - u) ** 2 * (1 - 2 * u) / T
        a[1] = cfg.clearance * 64 * (6 * u - 36 * u * u + 60 * u**3 - 30 * u**4) / T**2
        positions[swing] = p
        vel[swing] = v
        acc[swing] = a
        return positions, vel, acc, [stance]


class WBC:
    def __init__(self, model, data, cfg):
        self.m = model
        self.d = data
        self.cfg = cfg
        self.scratch = mujoco.MjData(model)
        self.sites = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, s + "_sole")
            for s in ["left", "right"]
        ]
        self.root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.q0 = data.qpos.copy()
        self.mass = model.body_mass.sum()
        self.previous_torque = None
        self.current_time = 0.0
        self.feedback_state = None
        self.last_mapping = {}
        self.torque_limits = np.array([80, 80, 30, 80, 80, 30]) * cfg.torque_scale

    def jacobians(self, data):
        Jc = np.zeros((3, self.m.nv))
        mujoco.mj_jacSubtreeCom(self.m, data, Jc, self.root)
        out = [Jc[[0, 2]]]
        for site in self.sites:
            Jp = np.zeros((3, self.m.nv))
            Jr = np.zeros_like(Jp)
            mujoco.mj_jacSite(self.m, data, Jp, Jr, site)
            out.append(np.vstack([Jp[[0, 2]], Jr[[1]]]))
        return out

    def solve(
        self, desired_com, desired_vel, desired_acc, feet, vel, acc, contacts, loads
    ):
        m = self.m
        d = self.d
        mujoco.mj_forward(m, d)
        nv = m.nv
        nc = len(contacts)
        n = nv + 6 + 3 * nc
        Js = self.jacobians(d)
        eps = 1e-6
        self.scratch.qpos[:] = d.qpos
        self.scratch.qvel[:] = d.qvel
        mujoco.mj_integratePos(m, self.scratch.qpos, d.qvel, eps)
        mujoco.mj_forward(m, self.scratch)
        advanced = self.jacobians(self.scratch)
        jdot = [(b - a) @ d.qvel / eps for a, b in zip(Js, advanced)]
        self.last_jdot = jdot
        H = np.eye(n) * 1e-5
        g = np.zeros(n)

        def task(J, target, weight):
            nonlocal H, g
            A = np.zeros((len(target), n))
            A[:, :nv] = J
            H += weight * A.T @ A
            g -= weight * A.T @ target

        com = d.subtree_com[self.root, [0, 2]]
        vcom = Js[0] @ d.qvel
        # One outer-loop acceleration and one pair of gains for both tasks.
        sensed_com, sensed_vel = (
            (com, vcom) if self.feedback_state is None else self.feedback_state
        )
        target_acc = (
            desired_acc
            + 80 * (desired_com - sensed_com)
            + 18 * (desired_vel - sensed_vel)
        )
        task(Js[0], target_acc - jdot[0], 30.0)
        total_fz = self.mass * (9.81 + target_acc[1])
        raw_cop = sensed_com[0] - sensed_com[1] * target_acc[0] / max(
            9.81 + target_acc[1], 1.0
        )
        lo = min(feet[side][0] - 0.09 for side in contacts)
        hi = max(feet[side][0] + 0.09 for side in contacts)
        target_cop = float(np.clip(raw_cop, lo, hi))
        if self.cfg.mapping == "coupled":
            # World-y moment balance at z=0:
            # sum((x_i-p) Fz_i - z_i Fx_i - My_i) = 0.
            # Wrenches are at sole sites in world axes, NOT ankle torques.
            moment = np.zeros(n)
            for i, side in enumerate(contacts):
                col = nv + 6 + 3 * i
                x, z = d.site_xpos[self.sites[side], [0, 2]]
                moment[col : col + 3] = [-z, x - target_cop, -1]
            A = moment / (self.mass * 9.81)
            H += 3e5 * np.outer(A, A)
            # Total vertical force follows the same vertical acceleration target.
            F = np.zeros(n)
            for i in range(nc):
                F[nv + 6 + 3 * i + 1] = 1 / (self.mass * 9.81)
            H += 300 * np.outer(F, F)
            g -= 300 * F * total_fz / (self.mass * 9.81)
        self.last_mapping = {
            "com": com.copy(),
            "vcom": vcom.copy(),
            "sensed_com": sensed_com.copy(),
            "sensed_vel": sensed_vel.copy(),
            "target_acc": target_acc.copy(),
            "cop_raw": raw_cop,
            "cop_command": target_cop,
            "support": [lo, hi],
        }
        Jpitch = np.zeros((1, nv))
        Jpitch[0, 2] = 1
        task(Jpitch, np.array([-100 * d.qpos[2] - 20 * d.qvel[2]]), 30.0)
        for side in range(2):
            J = Js[side + 1]
            site = self.sites[side]
            pitch = d.qpos[2] + sum(d.qpos[3 + 3 * side : 6 + 3 * side])
            pose = np.r_[d.site_xpos[site, [0, 2]], pitch]
            v = J @ d.qvel
            target = (
                np.r_[acc[side], 0.0]
                + 160 * (np.r_[feet[side], 0.0] - pose)
                + 25 * (np.r_[vel[side], 0.0] - v)
                - jdot[side + 1]
            )
            task(J, target, 100.0 if side not in contacts else 3.0)
        Jq = np.zeros((6, nv))
        Jq[:, 3:] = np.eye(6)
        task(Jq, 15 * (self.q0[3:] - d.qpos[3:]) - 4 * d.qvel[3:], 0.01)
        H[nv : nv + 6, nv : nv + 6] += np.eye(6) * 1e-4
        for i, side in enumerate(contacts):
            index = nv + 6 + 3 * i + 1
            H[index, index] += 0.01
            g[index] -= 0.01 * self.mass * 9.81 * loads[side]
        mass = np.zeros((nv, nv))
        mujoco.mj_fullM(m, mass, d.qM)
        dyn = np.zeros((nv, n))
        dyn[:, :nv] = mass
        dyn[3:, nv : nv + 6] = -np.eye(6)
        for i, side in enumerate(contacts):
            dyn[:, nv + 6 + 3 * i : nv + 9 + 3 * i] = -Js[side + 1].T
        constraints = [dyn]
        low = [-d.qfrc_bias + d.qfrc_passive]
        high = [low[0].copy()]
        for side in contacts:
            J = Js[side + 1]
            site = self.sites[side]
            pitch = d.qpos[2] + sum(d.qpos[3 + 3 * side : 6 + 3 * side])
            pose = np.r_[d.site_xpos[site, [0, 2]], pitch]
            target = (
                160 * (np.r_[feet[side], 0.0] - pose)
                - 25 * (J @ d.qvel)
                - jdot[side + 1]
            )
            C = np.zeros((3, n))
            C[:, :nv] = J
            constraints.append(C)
            low.append(target)
            high.append(target.copy())
        # Stance acceleration, dynamics and wrench cone are hard constraints.
        for i, side in enumerate(contacts):
            col = nv + 6 + 3 * i
            C = np.zeros((5, n))
            C[0, col] = 1
            C[0, col + 1] = -self.cfg.friction
            C[1, col] = -1
            C[1, col + 1] = -self.cfg.friction
            C[2, col + 2] = 1
            C[2, col + 1] = -0.09
            C[3, col + 2] = -1
            C[3, col + 1] = -0.09
            C[4, col + 1] = 1
            constraints.append(C)
            low.append(np.array([-np.inf] * 4 + [0.0]))
            high.append(np.array([0.0] * 4 + [2 * self.mass * 9.81]))
        C = np.eye(n)
        lower = np.full(n, -np.inf)
        upper = np.full(n, np.inf)
        lower[:nv] = -100
        upper[:nv] = 100
        lower[3:nv] = np.maximum(lower[3:nv], (-8 - d.qvel[3:]) * self.cfg.hz)
        upper[3:nv] = np.minimum(upper[3:nv], (8 - d.qvel[3:]) * self.cfg.hz)
        ranges = m.jnt_range[3:9]
        T = 1 / self.cfg.hz
        lower[3:nv] = np.maximum(
            lower[3:nv], 2 * (ranges[:, 0] + 0.01 - d.qpos[3:] - T * d.qvel[3:]) / T**2
        )
        upper[3:nv] = np.minimum(
            upper[3:nv], 2 * (ranges[:, 1] - 0.01 - d.qpos[3:] - T * d.qvel[3:]) / T**2
        )
        lower[nv : nv + 6] = -self.torque_limits
        upper[nv : nv + 6] = self.torque_limits
        if (
            self.cfg.torque_rate > 0
            and self.previous_torque is not None
            and self.current_time >= 0.4
        ):
            delta = self.cfg.torque_rate / self.cfg.hz
            lower[nv : nv + 6] = np.maximum(
                lower[nv : nv + 6], self.previous_torque - delta
            )
            upper[nv : nv + 6] = np.minimum(
                upper[nv : nv + 6], self.previous_torque + delta
            )
        normals = np.array([[np.cos(a), np.sin(a)] for a in np.arange(8) * np.pi / 4])
        for side in range(2):
            J = Js[side + 1][:2]
            Cspeed = np.zeros((8, n))
            Cspeed[:, :nv] = T * normals @ J
            rhs = 1.8 * np.cos(np.pi / 8) - normals @ (
                J @ d.qvel + T * jdot[side + 1][:2]
            )
            constraints.append(Cspeed)
            low.append(np.full(8, -np.inf))
            high.append(rhs)
        constraints.append(C)
        low.append(lower)
        high.append(upper)
        if np.any(np.concatenate(low) > np.concatenate(high)):
            raise RuntimeError("infeasible_acceleration_or_torque_bounds")
        qp = osqp.OSQP()
        qp.setup(
            P=sparse.csc_matrix(H),
            q=g,
            A=sparse.csc_matrix(np.vstack(constraints)),
            l=np.concatenate(low),
            u=np.concatenate(high),
            verbose=False,
            eps_abs=1e-5,
            eps_rel=1e-5,
            max_iter=3000,
            polishing=True,
            adaptive_rho_interval=50,
        )
        sol = qp.solve(raise_error=False)
        if sol.info.status_val not in [1, 2]:
            raise RuntimeError("QP " + sol.info.status)
        residual = np.max(np.abs(dyn @ sol.x - low[0]))
        tau = sol.x[nv : nv + 6]
        if np.max(np.abs(tau) - self.torque_limits) > 1e-3:
            raise RuntimeError("Torque constraint violation")
        tau = np.clip(tau, -self.torque_limits, self.torque_limits)
        applied = sol.x.copy()
        applied[nv : nv + 6] = tau
        residual = np.max(np.abs(dyn @ applied - low[0]))
        self.previous_torque = tau.copy()
        return tau, sol.x[:nv], sol.x[nv + 6 :], residual


def simulate(cfg, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    if (out / "result.json").exists():
        raise FileExistsError(f"Refusing overwrite: {out}")
    model = mujoco.MjModel.from_xml_path(str(R / "models/biped.xml"))
    model.opt.timestep = cfg.dt
    model.geom_friction[:, 0] = cfg.friction
    data = mujoco.MjData(model)
    q = leg_ik([0, 0.76], [0, 0])
    data.qpos[3:6] = q
    data.qpos[6:9] = q
    mujoco.mj_forward(model, data)
    ctl = WBC(model, data, cfg)
    h = float(data.subtree_com[ctl.root, 2])
    c0 = float(data.subtree_com[ctl.root, 0])
    plan = Plan(cfg, h, c0)
    torque = np.zeros(6)
    residual = 0.0
    predicted_wrench = np.zeros(6)
    logs = []
    audits = []
    from feedback import CoMChannel, contact_wrenches, resultant_cop

    channel = CoMChannel(cfg)
    missing_since = [None, None]
    failure = None
    frames = []
    renderer = None
    if cfg.record:
        renderer = mujoco.Renderer(model, height=720, width=960)
        camera = mujoco.MjvCamera()
        camera.distance = 2.5
        camera.azimuth = 75
        camera.elevation = -12
        camera.lookat[:] = [0, 0, 0.55]
    contact_counts = [0, 0]
    qp_times = []
    for k in range(round(cfg.duration / cfg.dt)):
        t = k * cfg.dt
        # mj_step integrates qpos after producing kinematic caches; refresh them
        # before finite-difference Jdot at the new qpos, without advancing time.
        mujoco.mj_forward(model, data)
        feet, vel, acc, contacts = plan.feet(t)
        actual_wrench = contact_wrenches(model, data, ctl.sites)
        measured_cop = resultant_cop(
            data.site_xpos[ctl.sites][:, [0, 2]], actual_wrench
        )
        Jcom = ctl.jacobians(data)[0]
        actual_com = data.subtree_com[ctl.root, [0, 2]].copy()
        actual_velocity = Jcom @ data.qvel
        channel.sample(t, actual_com, actual_velocity)
        # Check the support assumption using measured forces. Allow contact solver
        # settling for at most 40 ms after scheduled touchdown, then reject it.
        expected_loads = plan.loads(t)
        for side in contacts:
            if expected_loads[side] < 0.05:
                missing_since[side] = None
                continue
            if actual_wrench[side, 1] < 5:
                if missing_since[side] is None:
                    missing_since[side] = t
                if cfg.contact_guard and t - missing_since[side] > 0.04:
                    failure = "planned_support_not_measured"
            else:
                missing_since[side] = None
        for side in set([0, 1]) - set(contacts):
            missing_since[side] = None
        if failure:
            break
        if k % round(1 / (cfg.hz * cfg.dt)) == 0:
            c = plan.state[k, 0]
            v = plan.state[k, 1]
            a = 9.81 / h * (c - plan.p[k])
            before = time.monotonic()
            try:
                ctl.current_time = t
                estimated, estimated_velocity, age = channel.read(t)
                if cfg.feedback == "ideal":
                    ctl.feedback_state = None
                else:
                    ctl.feedback_state = (
                        np.array([estimated[0], actual_com[1]]),
                        np.array([estimated_velocity[0], actual_velocity[1]]),
                    )
                torque, ddq, wrench, residual = ctl.solve(
                    np.array([c, h]),
                    np.array([v, 0]),
                    np.array([a, 0]),
                    feet,
                    vel,
                    acc,
                    contacts,
                    plan.loads(t),
                )
            except (RuntimeError, ValueError, osqp.OSQPException) as e:
                failure = str(e)
                break
            predicted_wrench[:] = 0
            for i, side in enumerate(contacts):
                predicted_wrench[side * 3 : side * 3 + 3] = wrench[i * 3 : i * 3 + 3]
            qp_times.append(time.monotonic() - before)
            audit = dict(
                time=t,
                sensor_age=age,
                measured_cop=measured_cop,
                predicted_cop=resultant_cop(
                    data.site_xpos[ctl.sites][:, [0, 2]], predicted_wrench.reshape(2, 3)
                ),
                predicted_wrench=predicted_wrench.copy(),
                measured_wrench=actual_wrench.copy(),
                contact_sides=contacts.copy(),
                desired_loads=plan.loads(t),
                contact_ctrl=data.ctrl.copy(),
                qpos=data.qpos.copy(),
                qvel=data.qvel.copy(),
                torque=torque.copy(),
                target_com=np.array([c, h]),
                target_velocity=np.array([v, 0]),
                **ctl.last_mapping,
            )
            audits.append(audit)
        data.ctrl[:] = torque
        data.xfrc_applied[:] = 0
        if 2.7 <= t < 2.8:
            data.xfrc_applied[ctl.root, 0] = cfg.push
        normal = np.zeros(2)
        for i in range(data.ncon):
            contact = data.contact[i]
            for side in range(2):
                geom = mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_GEOM,
                    ["left_sole_geom", "right_sole_geom"][side],
                )
                if geom in [contact.geom1, contact.geom2] and 0 in [
                    contact.geom1,
                    contact.geom2,
                ]:
                    force = np.zeros(6)
                    mujoco.mj_contactForce(model, data, i, force)
                    normal[side] += abs((contact.frame.reshape(3, 3).T @ force[:3])[2])
        contact_counts[0] += int(normal[0] > 5)
        contact_counts[1] += int(normal[1] > 5)
        sole = np.concatenate([data.site_xpos[s, [0, 2]] for s in ctl.sites])
        logs.append(
            [
                t,
                *data.subtree_com[ctl.root, [0, 2]],
                data.qpos[2],
                *sole,
                *normal,
                *torque,
                residual,
                *data.qpos,
                *data.qvel,
                *np.concatenate(feet),
                *predicted_wrench,
            ]
        )
        if renderer and k % round(0.05 / cfg.dt) == 0:
            camera.lookat[0] = data.qpos[0]
            renderer.update_scene(data, camera)
            frames.append(renderer.render().copy())
        if abs(data.qpos[2]) > 0.35 or data.subtree_com[ctl.root, 2] < 0.55:
            failure = "body_tilt_or_height"
            break
        if np.max(np.abs(data.qvel[3:])) > 10:
            failure = "joint_velocity"
            break
        mujoco.mj_step(model, data)
    if renderer:
        renderer._gl_context.make_current()
        renderer._mjr_context.free()
        renderer._gl_context.free()
    x = np.array(logs)
    mujoco.mj_forward(model, data)
    progress = float(data.subtree_com[ctl.root, 0] - c0)
    result = {
        "config": asdict(cfg),
        "success": failure is None,
        "failure": failure,
        "duration_s": float(data.time),
        "progress_m": progress,
        "com_height_initial_m": h,
        "mass_kg": float(model.body_mass.sum()),
        "max_body_pitch_deg": (
            float(np.rad2deg(np.max(np.abs(x[:, 3])))) if len(x) else None
        ),
        "contact_samples_above5N": contact_counts,
        "max_dynamics_qp_residual": float(np.max(x[:, 16])) if len(x) else None,
        "qp_solve_p95_ms": (
            float(np.quantile(qp_times, 0.95) * 1000) if qp_times else None
        ),
        "source": "Torque-driven planar articulated biped, no prescribed qpos trajectory and no applied contact wrench",
    }
    verified = []
    if len(x) and cfg.walk:
        for step in range(1, cfg.steps + 1):
            touchdown = plan.start + step * cfg.step_time
            side = step % 2
            mask = (x[:, 0] >= touchdown - 0.04) & (x[:, 0] <= touchdown + 0.10)
            correct = (
                mask
                & (np.abs(x[:, 4 + side * 2] - step * cfg.step_length) < 0.025)
                & (np.abs(x[:, 5 + side * 2]) < 0.008)
                & (x[:, 8 + side] > 10)
            )
            run = 0
            found = False
            for good in correct:
                run = run + 1 if good else 0
                if run * cfg.dt >= 0.015:
                    found = True
                    break
            verified.append(found)
    result["verified_landings"] = verified
    result["max_joint_velocity_rad_s"] = (
        float(np.max(np.abs(x[:, 29:35]))) if len(x) else None
    )
    finalfeet = plan.feet(cfg.duration)[0]
    final_error = max(
        np.linalg.norm(data.site_xpos[ctl.sites[i], [0, 2]] - finalfeet[i])
        for i in range(2)
    )
    result["terminal_foot_error_m"] = float(final_error)
    if (
        result["success"]
        and cfg.walk
        and (
            not all(verified)
            or progress < cfg.steps * cfg.step_length - 0.10
            or final_error > 0.04
        )
    ):
        result["success"] = False
        result["failure"] = "walking_completion_checks"
    if audits:
        fields = [k for k in audits[0] if k != "contact_sides"]
        np.savez_compressed(
            out / "audit.npz",
            **{k: np.asarray([a[k] for a in audits]) for k in fields},
            contact_mask=np.asarray(
                [[i in a["contact_sides"] for i in range(2)] for a in audits]
            ),
        )
    result["controller_updates"] = len(audits)
    result["sensor_contract"] = (
        "Only horizontal outer-loop CoM feedback delayed; joint encoders, kinematics and contact forces ideal"
    )
    (out / "result.json").write_text(json.dumps(result, indent=2))
    np.savez_compressed(out / "trace.npz", data=x)
    if frames:
        np.save(out / "frames.npy", np.stack(frames))
    return result


if __name__ == "__main__":
    for walk in [False, True]:
        r = simulate(
            BipedConfig(walk=walk),
            R / "runs" / ("biped_pilot_walk" if walk else "biped_pilot_stand"),
        )
        print(r, flush=True)
