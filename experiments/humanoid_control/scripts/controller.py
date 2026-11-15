"""3-D floating-base inverse dynamics: only the 29 joint motor torques are applied."""

import numpy as np
from scipy.spatial.transform import Rotation
import mujoco
from qp import solve_qp


def orientation_error(actual, target):
    return Rotation.from_matrix(target @ actual.T).as_rotvec()


class WholeBody:
    def __init__(self, model, data, hz=100, friction=0.6, backend="clarabel"):
        self.backend = backend
        self.m = model
        self.d = data
        self.hz = hz
        self.friction = friction
        self.scratch = mujoco.MjData(model)
        self.root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.torso = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
        self.sites = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, s + "_sole")
            for s in ["left", "right"]
        ]
        self.qref = data.qpos.copy()
        self.mass = model.body_mass.sum()
        self.torque_limits = model.actuator_ctrlrange[:, 1].copy()
        self.joints = model.actuator_trnid[:, 0]
        self.dofs = model.jnt_dofadr[self.joints]
        self.qindices = model.jnt_qposadr[self.joints]
        self.S = np.zeros((model.nv, model.nu))
        self.S[self.dofs, np.arange(model.nu)] = 1
        assert model.nv == 35 and model.nu == 29 and min(self.dofs) == 6
        assert np.all(model.actuator_gainprm[:, 0] == 1) and np.all(
            model.actuator_biasprm == 0
        )
        self.torso_target = data.xmat[self.torso].reshape(3, 3).copy()
        self.pelvis_target = data.xmat[self.root].reshape(3, 3).copy()
        self.last = {}

    def jacobians(self, data):
        m = self.m
        jc = np.zeros((3, m.nv))
        mujoco.mj_jacSubtreeCom(m, data, jc, self.root)
        result = [jc]
        for site in self.sites:
            jp = np.zeros((3, m.nv))
            jr = np.zeros_like(jp)
            mujoco.mj_jacSite(m, data, jp, jr, site)
            result.append(np.vstack([jp, jr]))
        for body in [self.torso, self.root]:
            jp = np.zeros((3, m.nv))
            jr = np.zeros_like(jp)
            mujoco.mj_jacBody(m, data, jp, jr, body)
            result.append(jr)
        return result

    def solve(
        self,
        com_target,
        com_velocity,
        com_acceleration,
        feet,
        foot_velocity,
        foot_acceleration,
        rotations,
        contacts,
        loads,
    ):
        m = self.m
        d = self.d
        mujoco.mj_forward(m, d)
        nv, nu = m.nv, m.nu
        nc = len(contacts)
        n = nv + nu + 6 * nc
        js = self.jacobians(d)
        eps = 1e-6
        self.scratch.qpos[:] = d.qpos
        self.scratch.qvel[:] = d.qvel
        mujoco.mj_integratePos(m, self.scratch.qpos, d.qvel, eps)
        mujoco.mj_forward(m, self.scratch)
        advanced = self.jacobians(self.scratch)
        jdot = [(b - a) @ d.qvel / eps for a, b in zip(js, advanced)]
        H = np.eye(n) * 1e-6
        g = np.zeros(n)

        def objective(A, target, weight):
            nonlocal H, g
            H += weight * A.T @ A
            g -= weight * A.T @ target

        def acceleration(J, target, weight):
            A = np.zeros((len(target), n))
            A[:, :nv] = J
            objective(A, target, weight)

        com = d.subtree_com[self.root].copy()
        vcom = js[0] @ d.qvel
        cmd = com_acceleration + 80 * (com_target - com) + 18 * (com_velocity - vcom)
        acceleration(js[0], cmd - jdot[0], 40.0)
        for index, body, target in [
            (3, self.torso, self.torso_target),
            (4, self.root, self.pelvis_target),
        ]:
            err = orientation_error(d.xmat[body].reshape(3, 3), target)
            acceleration(
                js[index],
                100 * err - 20 * js[index] @ d.qvel - jdot[index],
                30.0 if index == 3 else 10.0,
            )
        foot_cmd = []
        for side in range(2):
            site = self.sites[side]
            J = js[side + 1]
            err = np.r_[
                feet[side] - d.site_xpos[site],
                orientation_error(d.site_xmat[site].reshape(3, 3), rotations[side]),
            ]
            desired = (
                np.r_[foot_acceleration[side], np.zeros(3)]
                + 200 * err
                + 28 * (np.r_[foot_velocity[side], np.zeros(3)] - J @ d.qvel)
                - jdot[side + 1]
            )
            foot_cmd.append(desired)
            if side not in contacts:
                acceleration(J, desired, 100.0)
        posture = np.zeros((nu, nv))
        posture[np.arange(nu), self.dofs] = 1
        acceleration(
            posture,
            25 * (self.qref[self.qindices] - d.qpos[self.qindices])
            - 8 * d.qvel[self.dofs],
            0.15,
        )
        # Low regularization of torque and smooth target vertical load sharing.
        H[nv : nv + nu, nv : nv + nu] += np.eye(nu) * 1e-5
        for i, side in enumerate(contacts):
            row = np.zeros((1, n))
            row[0, nv + nu + 6 * i + 2] = 1 / (self.mass * 9.81)
            objective(row, np.array([loads[side]]), 5.0)
        # Consistent LIPM CoP reference on the ground, optimized through 6-D wrenches.
        p = com[:2] - com[2] * cmd[:2] / max(9.81 + cmd[2], 1)
        lowxy = np.min(
            [feet[i][:2] - np.array([0.075, 0.02]) for i in contacts], axis=0
        )
        highxy = np.max(
            [feet[i][:2] + np.array([0.075, 0.02]) for i in contacts], axis=0
        )
        p = np.clip(p, lowxy, highxy)
        moment = np.zeros((2, n))
        for i, side in enumerate(contacts):
            col = nv + nu + 6 * i
            x, y, z = d.site_xpos[self.sites[side]]
            moment[0, col : col + 6] = [-z, 0, x - p[0], 0, -1, 0]
            moment[1, col : col + 6] = [0, -z, y - p[1], 1, 0, 0]
        objective(moment / (self.mass * 9.81), np.zeros(2), 1e5)
        M = np.zeros((nv, nv))
        mujoco.mj_fullM(m, M, d.qM)
        dynamics = np.zeros((nv, n))
        dynamics[:, :nv] = M
        dynamics[:, nv : nv + nu] = -self.S
        for i, side in enumerate(contacts):
            dynamics[:, nv + nu + 6 * i : nv + nu + 6 * i + 6] = -js[side + 1].T
        rhs = -d.qfrc_bias + d.qfrc_passive
        matrices = [dynamics]
        lower = [rhs]
        upper = [rhs.copy()]
        for side in contacts:
            A = np.zeros((6, n))
            A[:, :nv] = js[side + 1]
            matrices.append(A)
            lower.append(foot_cmd[side])
            upper.append(foot_cmd[side].copy())
        for i, side in enumerate(contacts):
            col = nv + nu + 6 * i
            # Cones expressed in current foot coordinates, not a fixed world frame.
            world_to_foot = d.site_xmat[self.sites[side]].reshape(3, 3).T
            transform = np.zeros((6, 6))
            transform[:3, :3] = world_to_foot
            transform[3:, 3:] = world_to_foot
            rows = []
            for angle in np.arange(8) * np.pi / 4:
                rows.append(
                    [
                        np.cos(angle),
                        np.sin(angle),
                        -self.friction * np.cos(np.pi / 8),
                        0,
                        0,
                        0,
                    ]
                )
            for axis, half in [(3, 0.02), (4, 0.075), (5, 0.02)]:
                for sign in [-1, 1]:
                    v = np.zeros(6)
                    v[axis] = sign
                    v[2] = -half
                    rows.append(v)
            rows.append([0, 0, 1, 0, 0, 0])
            C = np.zeros((len(rows), n))
            C[:, col : col + 6] = np.asarray(rows) @ transform
            matrices.append(C)
            lower.append(np.r_[np.full(len(rows) - 1, -np.inf), 0])
            upper.append(np.r_[np.zeros(len(rows) - 1), 2 * self.mass * 9.81])
        bounds = np.eye(n)
        lo = np.full(n, -np.inf)
        hi = np.full(n, np.inf)
        lo[:nv] = -150
        hi[:nv] = 150
        T = 1 / self.hz
        ranges = m.jnt_range[self.joints]
        q = d.qpos[self.qindices]
        v = d.qvel[self.dofs]
        lo[self.dofs] = np.maximum(lo[self.dofs], (-8 - v) / T)
        hi[self.dofs] = np.minimum(hi[self.dofs], (8 - v) / T)
        lo[self.dofs] = np.maximum(
            lo[self.dofs], 2 * (ranges[:, 0] + 0.002 - q - T * v) / T**2
        )
        hi[self.dofs] = np.minimum(
            hi[self.dofs], 2 * (ranges[:, 1] - 0.002 - q - T * v) / T**2
        )
        lo[nv : nv + nu] = -self.torque_limits
        hi[nv : nv + nu] = self.torque_limits
        matrices.append(bounds)
        lower.append(lo)
        upper.append(hi)
        l = np.concatenate(lower)
        u = np.concatenate(upper)
        if np.any(l > u):
            raise RuntimeError("conflicting_joint_or_torque_bounds")
        self.last_problem = (H, g, np.vstack(matrices), l, u)
        sol = solve_qp(*self.last_problem, self.backend)
        tau = sol.x[nv : nv + nu]
        if np.max(np.abs(tau) - self.torque_limits) > 1e-3:
            raise RuntimeError("torque_limit_violation")
        tau = np.clip(tau, -self.torque_limits, self.torque_limits)
        executed = sol.x.copy()
        executed[nv : nv + nu] = tau
        violation = max(
            float(np.max(l - np.vstack(matrices) @ executed)),
            float(np.max(np.vstack(matrices) @ executed - u)),
        )
        # Check physical units separately: N, N.m, m/s² and rad/s² do not
        # share a meaningful single absolute tolerance.
        allowed = np.r_[
            np.full(nv, 0.005),
            np.tile(np.full(6, 0.05), nc),
            np.tile(np.r_[np.full(8, 0.1), np.full(6, 0.05), 0.1], nc),
            np.full(nv, 0.05),
            np.full(nu, 0.001),
            np.tile(np.r_[np.full(3, 0.1), np.full(3, 0.05)], nc),
        ]
        excess = np.maximum(
            l - np.vstack(matrices) @ executed, np.vstack(matrices) @ executed - u
        )
        normalized_violation = float(np.max(excess / allowed))
        if normalized_violation > 1:
            raise RuntimeError(
                f"constraint_residual row={np.argmax(excess / allowed)} value={excess[np.argmax(excess / allowed)]:.4g}"
            )
        self.last = dict(
            normalized_constraint_violation=normalized_violation,
            constraint_violation=violation,
            com=com,
            vcom=vcom,
            cmd=cmd,
            cop_command=p,
            dynamics_residual=float(np.max(np.abs(dynamics @ executed - rhs))),
            iterations=int(sol.info.iter),
            qp_status=sol.info.status,
        )
        wrench = np.zeros((2, 6))
        for i, side in enumerate(contacts):
            wrench[side] = sol.x[nv + nu + 6 * i : nv + nu + 6 * i + 6]
        return tau, wrench
