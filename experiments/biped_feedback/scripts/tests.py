"""Physical and sign-convention contracts, independent of benchmark outcomes."""

from pathlib import Path
import unittest
import mujoco
import numpy as np
from biped import BipedConfig, Plan, WBC, leg_ik
from feedback import CoMChannel, contact_wrenches, resultant_cop

R = Path(__file__).resolve().parents[1]


class Contracts(unittest.TestCase):
    def test_cop_signed_moment(self):
        # Positive world-y moment moves CoP in negative x.
        positions = np.array([[0.3, 0], [0.1, 0]])
        wrench = np.array([[0, 100, 2], [0, 100, 0]])
        self.assertAlmostEqual(resultant_cop(positions, wrench), 0.19)
        # Off-ground reference must include z Fx.
        self.assertAlmostEqual(
            resultant_cop(np.array([[0.3, 0.1]]), np.array([[10, 100, 2]])), 0.27
        )

    def test_cop_unloaded_is_unknown(self):
        self.assertTrue(np.isnan(resultant_cop(np.zeros((2, 2)), np.zeros((2, 3)))))

    def test_ds_matches_global_reference(self):
        cfg = BipedConfig()
        p = Plan(cfg, 0.735, 0)
        for t in np.arange(0, 6, 0.001):
            feet, _, _, _ = p.feet(t)
            expected = sum(a * f[0] for a, f in zip(p.loads(t), feet))
            self.assertAlmostEqual(expected, p.p[round(t / 0.001)], places=10)
        np.testing.assert_allclose(p.loads(5.8), [0.5, 0.5])

    def test_swing_is_continuous_and_physically_moving(self):
        cfg = BipedConfig()
        p = Plan(cfg, 0.735, 0)
        feet, velocity, acc, _ = p.feet(0.9)
        self.assertGreater(max(f[1] for f in feet), 0.04)
        self.assertGreater(max(abs(v[0]) for v in velocity), 0.1)
        for t in [0.5, 0.62, 1.1, 1.22, 5.3]:
            before = p.feet(t - 1e-8)
            after = p.feet(t + 1e-8)
            for a, b in zip(before[:3], after[:3]):
                np.testing.assert_allclose(a, b, atol=1e-4)

    def test_sensor_is_causal_and_prediction_uses_past_only(self):
        cfg = BipedConfig(delay=0.04, feedback="predict", sensor_hz=100)
        c = CoMChannel(cfg)
        for k in range(101):
            t = k * 0.001
            c.sample(t, np.array([t, 0]), np.array([1.0, 0]))
            position, velocity, age = c.read(t)
            self.assertGreaterEqual(age, -1e-10)
            np.testing.assert_allclose(position, [t, 0], atol=1e-12)
            if t > 0.05:
                self.assertGreaterEqual(age, 0.04 - 1e-10)

    def test_ground_wrench_matches_generalized_constraint_force(self):
        m = mujoco.MjModel.from_xml_path(str(R / "models/biped.xml"))
        d = mujoco.MjData(m)
        q = leg_ik([0, 0.76], [0, 0])
        d.qpos[3:6] = q
        d.qpos[6:9] = q
        d.qpos[1] = -0.0005
        mujoco.mj_forward(m, d)
        ctl = WBC(m, d, BipedConfig())
        w = contact_wrenches(m, d, ctl.sites)
        self.assertGreater(w[:, 1].sum(), 0)
        J = ctl.jacobians(d)
        generalized = sum(J[i + 1].T @ w[i] for i in range(2))
        np.testing.assert_allclose(generalized, d.qfrc_constraint, atol=1e-8)

    def test_same_com_gains_define_cop_and_acceleration(self):
        m = mujoco.MjModel.from_xml_path(str(R / "models/biped.xml"))
        d = mujoco.MjData(m)
        q = leg_ik([0, 0.76], [0, 0])
        d.qpos[3:6] = q
        d.qpos[6:9] = q
        mujoco.mj_forward(m, d)
        ctl = WBC(m, d, BipedConfig())
        com = d.subtree_com[ctl.root, [0, 2]].copy()
        ctl.solve(
            com + np.array([0.001, 0]),
            np.zeros(2),
            np.zeros(2),
            [np.zeros(2)] * 2,
            [np.zeros(2)] * 2,
            [np.zeros(2)] * 2,
            [0, 1],
            [0.5, 0.5],
        )
        a = ctl.last_mapping["target_acc"]
        p = ctl.last_mapping["cop_raw"]
        self.assertAlmostEqual(a[0], 0.08)
        self.assertAlmostEqual(p, com[0] - com[1] * a[0] / 9.81)


if __name__ == "__main__":
    unittest.main()
