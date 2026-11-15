"""Free-base, actuator, mass, kinematic and contact-force contracts."""

import unittest
import numpy as np
import mujoco
from model import R, SOURCE, initialize
from controller import WholeBody, orientation_error
from run import Config, Reference, wrenches, smooth


class Contracts(unittest.TestCase):
    def setup_model(self):
        m = mujoco.MjModel.from_xml_path(str(R / "models/scene.xml"))
        d = initialize(m)
        c = WholeBody(m, d)
        return m, d, c

    def test_free_base_and_motor_transmission(self):
        m, d, c = self.setup_model()
        self.assertEqual((m.nq, m.nv, m.nu), (36, 35, 29))
        self.assertEqual(m.neq, 0)
        self.assertEqual(m.jnt_type[0], mujoco.mjtJoint.mjJNT_FREE)
        d.ctrl[:] = np.arange(29) * 0.01
        mujoco.mj_forward(m, d)
        np.testing.assert_allclose(d.qfrc_actuator[:6], 0, atol=1e-12)
        np.testing.assert_allclose(d.qfrc_actuator[c.dofs], d.ctrl, atol=1e-12)
        np.testing.assert_allclose(m.actuator_biasprm, 0)

    def test_original_mechanics_preserved(self):
        original = mujoco.MjModel.from_xml_path(str(SOURCE / "scene.xml"))
        m, d, c = self.setup_model()
        for key in [
            "body_mass",
            "body_inertia",
            "body_iquat",
            "body_pos",
            "jnt_range",
            "jnt_type",
            "geom_type",
            "geom_size",
            "geom_pos",
            "geom_contype",
            "geom_conaffinity",
        ]:
            np.testing.assert_allclose(
                getattr(m, key), getattr(original, key), atol=1e-12, err_msg=key
            )

    def test_signed_contact_virtual_work(self):
        m, d, c = self.setup_model()
        w = wrenches(m, d, c)
        J = c.jacobians(d)
        self.assertGreater(w[:, 2].sum(), 0)
        actual = sum(J[i + 1].T @ w[i] for i in [0, 1])
        efc = d.efc_force.copy()
        mask = d.efc_type >= int(mujoco.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS)
        efc[~mask] = 0
        expected = np.zeros(m.nv)
        mujoco.mj_mulJacTVec(m, d, expected, efc)
        np.testing.assert_allclose(actual, expected, atol=1e-8)

    def test_spatial_jacobian_and_so3_sign(self):
        m, d, c = self.setup_model()
        J = c.jacobians(d)[1]
        v = np.random.default_rng(11).normal(size=m.nv) * 0.02
        plus = mujoco.MjData(m)
        minus = mujoco.MjData(m)
        eps = 1e-6
        for temp, sign in [(plus, 1), (minus, -1)]:
            temp.qpos[:] = d.qpos
            mujoco.mj_integratePos(m, temp.qpos, v, sign * eps)
            mujoco.mj_forward(m, temp)
        position = (plus.site_xpos[c.sites[0]] - minus.site_xpos[c.sites[0]]) / (
            2 * eps
        )
        rotation = orientation_error(
            minus.site_xmat[c.sites[0]].reshape(3, 3),
            plus.site_xmat[c.sites[0]].reshape(3, 3),
        ) / (2 * eps)
        np.testing.assert_allclose(J @ v, np.r_[position, rotation], atol=1e-8)

    def test_reference_continuity(self):
        m, d, c = self.setup_model()
        for scenario in ["single", "step", "walk"]:
            ref = Reference(d, c, Config(scenario=scenario, duration=29))
            for t in [
                0.5,
                1.6,
                2.0,
                2.2,
                4.0,
                4.6,
                5.2,
                6.5,
                7.0,
                8.6,
                14.0,
                21.0,
                28.0,
            ]:
                before = ref.at(t - 1e-8)
                after = ref.at(t + 1e-8)
                for i in range(6):
                    np.testing.assert_allclose(
                        before[i], after[i], atol=1e-6, err_msg=f"{scenario}:{t}:{i}"
                    )

    def test_c2_profile(self):
        for t in [-1, 0, 1, 2]:
            p, v, a = smooth(t, 0, 1)
            self.assertEqual(v, 0)
            self.assertEqual(a, 0)
        dt = 1e-6
        t = 0.4
        p, v, a = smooth(t, 0, 1)
        self.assertAlmostEqual(
            (smooth(t + dt, 0, 1)[0] - smooth(t - dt, 0, 1)[0]) / (2 * dt), v, places=8
        )

    def test_backend_bound_conversion(self):
        from qp import solve_qp

        H = np.diag([2.0, 4.0])
        g = np.array([-2.0, -4.0])
        A = np.array([[1.0, 1.0], [1.0, 0.0], [0.0, 1.0]])
        l = np.array([1.0, 0.0, 0.0])
        u = np.array([1.0, 0.8, np.inf])
        for backend in ["osqp", "clarabel"]:
            result = solve_qp(H, g, A, l, u, backend)
            np.testing.assert_allclose(result.x, [1 / 3, 2 / 3], atol=1e-5)

    def test_single_support_needs_actual_load(self):
        from metrics import evaluate

        cfg = Config(scenario="single", duration=8, dt=0.01)
        t = np.arange(800) * 0.01
        feet = np.zeros((800, 2, 3))
        feet[:, 0, 1] = 0.1
        feet[:, 1, 1] = -0.1
        force = np.zeros((800, 2, 6))
        force[:, :, 2] = 100.0
        hold = (t >= 2.4) & (t < 3.8)
        feet[hold, 1, 2] = 0.04
        force[hold, 0, 2] = 200.0
        force[hold, 1, 2] = 0.0
        qpos = np.zeros((800, 36))
        qpos[:, 3] = 1.0
        arrays = dict(
            time=t,
            feet=feet,
            reference_feet=feet.copy(),
            com=np.zeros((800, 3)),
            reference_com=np.zeros((800, 3)),
            qpos=qpos,
            actual_wrench=force,
        )
        self.assertTrue(evaluate(arrays, cfg, 200 / 9.81)["criteria_passed"])
        force[hold, 0, 2] = 0.0
        self.assertFalse(evaluate(arrays, cfg, 200 / 9.81)["criteria_passed"])

    def test_support_can_be_mirrored(self):
        m, d, c = self.setup_model()
        ref = Reference(d, c, Config(scenario="single", support_side=1))
        goals = ref.at(3.0)
        self.assertEqual(goals[7], [1])
        self.assertGreater(goals[3][0, 2], 0.03)


if __name__ == "__main__":
    unittest.main()
