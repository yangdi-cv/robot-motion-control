"""Regression checks for packet causality, trajectory continuity and rigid-body math."""

import unittest
from pathlib import Path
import numpy as np
import mujoco
from engine import Config, simulate
from network import NetworkConfig, Observer, run
from gait import GaitConfig, simulate as gait_sim, project, split
from trajectory import Swing
from biped import BipedConfig, WBC, leg_ik

R = Path(__file__).resolve().parents[1]


class Checks(unittest.TestCase):
    def test_zero_delay_sensor_equivalence(self):
        _, base = simulate(Config(controller="capture_predict", impulse_x=12, seed=1))
        _, packets = run(
            NetworkConfig(
                controller="capture_predict", impulse_x=12, seed=1, sensor_hz=1000
            )
        )
        np.testing.assert_array_equal(base, packets)

    def test_fixed_sensor_repeats_and_age(self):
        r, _ = run(
            NetworkConfig(
                controller="capture_predict",
                sensor_hz=25,
                hz=200,
                impulse_x=0,
                noise_position=0,
                noise_velocity=0,
            )
        )
        self.assertTrue(r["success"])
        self.assertAlmostEqual(r["observer"]["reused_fraction"], 0.875, delta=0.01)
        self.assertLessEqual(r["observer"]["age_max_s"], 0.0400001)

    def test_reordering_never_uses_future(self):
        cfg = NetworkConfig(sensor_hz=200, delay=0.02, jitter=0.02)
        observer = Observer(cfg)
        sources = []
        for k in range(300):
            observer.update(k, np.zeros((2, 2)))
            source, _ = observer.read(k)
            sources.append(source)
            self.assertLessEqual(source, k)
        self.assertTrue(np.all(np.diff(sources) >= 0))
        self.assertGreater(observer.reordered, 1)

    def test_stale_data_guard(self):
        r, _ = run(
            NetworkConfig(
                controller="capture_predict",
                sensor_hz=25,
                delay=0.04,
                burst_duration=0.4,
                max_age=0.15,
            )
        )
        self.assertEqual(r["reason"], "stale_feedback")
        self.assertLess(r["duration_s"], 3.0)

    def test_colored_noise_is_not_white(self):
        a = Observer(NetworkConfig(color_tau=0)).noise[:, 0, 0]
        b = Observer(NetworkConfig(color_tau=0.08)).noise[:, 0, 0]
        self.assertLess(abs(np.corrcoef(a[:-1], a[1:])[0, 1]), 0.05)
        self.assertGreater(np.corrcoef(b[:-1], b[1:])[0, 1], 0.95)

    def test_swing_replan_c2_and_extrema(self):
        s = Swing(np.array([-0.18, -0.07]), np.array([0.18, 0.07]), 0.0, 0.5)
        proposal = s.proposal(0.2, 0.6, np.array([0.2, 0.07]))
        for x, y in zip(s.sample(0.2), proposal.sample(0.2)):
            np.testing.assert_allclose(x, y, atol=1e-12)
        p, v, a = proposal.sample(0.6)
        np.testing.assert_allclose(p, [0.2, 0.07, 0], atol=1e-10)
        np.testing.assert_allclose(v, 0, atol=1e-10)
        bv, ba = proposal.bounds()
        samples = [proposal.sample(t) for t in np.linspace(0.2, 0.6, 501)]
        self.assertLessEqual(max(np.linalg.norm(x[1]) for x in samples), bv + 1e-8)
        self.assertLessEqual(max(np.linalg.norm(x[2]) for x in samples), ba + 1e-7)

    def test_retiming_cannot_push_swing_below_ground(self):
        s = Swing(np.array([-0.18, -0.07]), np.array([0.18, 0.07]), 0.12, 0.6)
        trial = s.proposal(0.50, 0.75, np.array([0.18, 0.07]))
        dense = np.array([trial.sample(t)[0][2] for t in np.linspace(0.5, 0.75, 1001)])
        low, high = trial.height_range()
        self.assertLessEqual(low, float(dense.min()) + 1e-10)
        self.assertGreaterEqual(high, float(dense.max()) - 1e-10)
        self.assertLess(low, 0.0)
        self.assertFalse(trial.clearance_ok())

    def test_double_support_has_feasible_individual_cops(self):
        old = np.array([0.0, 0.07])
        new = np.array([0.18, -0.07])
        cop = project(np.array([0.3, 0.2]), [old, new])
        alpha, p0, p1 = split(cop, old, new, 0.5)
        np.testing.assert_allclose(cop, (1 - alpha) * p0 + alpha * p1, atol=1e-12)
        self.assertTrue(0 <= alpha <= 1)

    def test_nominal_timing_does_not_drift(self):
        a = gait_sim(
            GaitConfig(
                method="fixed", steps=4, impulse_x=0, noise_position=0, noise_velocity=0
            )
        )
        b = gait_sim(
            GaitConfig(
                method="timing",
                steps=4,
                impulse_x=0,
                noise_position=0,
                noise_velocity=0,
            )
        )
        self.assertTrue(a["success"] and b["success"])
        self.assertEqual(a["duration_s"], b["duration_s"])
        self.assertLess(b["terminal_original_route_error_m"], 0.001)

    def test_planar_inverse_kinematics(self):
        model = mujoco.MjModel.from_xml_path(str(R / "models/biped.xml"))
        data = mujoco.MjData(model)
        data.qpos[0] = 0.06
        for i, x in enumerate([0.12, 0.0]):
            data.qpos[3 + 3 * i : 6 + 3 * i] = leg_ik([0.06, 0.76], [x, 0])
        mujoco.mj_forward(model, data)
        for name, x in [("left_sole", 0.12), ("right_sole", 0.0)]:
            site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
            np.testing.assert_allclose(data.site_xpos[site, [0, 2]], [x, 0], atol=1e-12)
        self.assertEqual(model.nu, 6)
        self.assertTrue(np.all(model.actuator_trnid[:, 0] >= 3))

    def test_jdot_uses_synchronized_caches(self):
        model = mujoco.MjModel.from_xml_path(str(R / "models/biped.xml"))
        data = mujoco.MjData(model)
        q = leg_ik([0, 0.76], [0, 0])
        data.qpos[3:6] = q
        data.qpos[6:9] = q
        data.qvel[:] = np.random.default_rng(1).normal(size=9) * 0.001
        mujoco.mj_forward(model, data)
        mujoco.mj_step(model, data)
        ctl = WBC(model, data, BipedConfig(walk=False))
        mujoco.mj_forward(model, data)
        com = data.subtree_com[ctl.root, [0, 2]].copy()
        ctl.solve(
            com,
            np.zeros(2),
            np.zeros(2),
            [np.zeros(2), np.zeros(2)],
            [np.zeros(2), np.zeros(2)],
            [np.zeros(2), np.zeros(2)],
            [0, 1],
            [0.5, 0.5],
        )
        plus = mujoco.MjData(model)
        minus = mujoco.MjData(model)
        eps = 5e-7
        for d, sign in [(plus, 1), (minus, -1)]:
            d.qpos[:] = data.qpos
            d.qvel[:] = data.qvel
            mujoco.mj_integratePos(model, d.qpos, data.qvel, sign * eps)
            mujoco.mj_forward(model, d)
        for measured, a, b in zip(
            ctl.last_jdot, ctl.jacobians(plus), ctl.jacobians(minus)
        ):
            np.testing.assert_allclose(
                measured, (a - b) @ data.qvel / (2 * eps), atol=1e-6
            )


if __name__ == "__main__":
    unittest.main()
