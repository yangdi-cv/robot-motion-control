"""Mathematical and interface checks, separate from empirical success statistics."""

import unittest
import numpy as np
from core import Config, Reference, transition, predictor, simulate, SAFE_HALF


class Dynamics(unittest.TestCase):
    def test_reference_dynamics_and_terminal_stop(self):
        cfg = Config()
        r = Reference(cfg)
        A, B = transition(0.8, 0.001)
        for k in [0, 599, 600, 1999, 4200, 6599, 6600, 9199]:
            phase = min(k // r.phase_steps, cfg.steps - 1)
            np.testing.assert_allclose(
                r.state[k + 1],
                r.state[k] @ A.T + r.feet[phase, :, None] * B,
                atol=1e-12,
                rtol=0,
            )
        self.assertLess(np.linalg.norm(r.state[-1, :, 1]), 0.001)
        self.assertLess(np.linalg.norm(r.state[-1, :, 0] - r.feet[-1]), 0.001)

    def test_zoh_and_force_units(self):
        A, B = transition(0.8, 0.01)
        a, b = transition(0.8, 0.005)
        np.testing.assert_allclose(A, a @ a, atol=1e-14)
        np.testing.assert_allclose(B, a @ b + b, atol=1e-14)
        omega = np.sqrt(9.81 / 0.8)
        F = 60.0
        mass = 30.0
        dt = 0.01
        response = -F / (mass * omega**2) * B
        self.assertAlmostEqual(
            response[1], F / (mass * omega) * np.sinh(omega * dt), places=12
        )

    def test_prediction_is_causal(self):
        cfg = Config()
        history = np.random.default_rng(2).normal(size=(40, 2)) * 0.1
        state = np.array([[0.02, 0.1], [-0.03, 0.05]])
        expected = state.copy()
        A, B = transition(0.8, 0.001)
        for p in history[3:23]:
            expected = expected @ A.T + p[:, None] * B
        actual = predictor(state, history, 3, 23, cfg, {})
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        history[23:] += 1000
        np.testing.assert_allclose(
            predictor(state, history, 3, 23, cfg, {}), actual, atol=1e-12
        )

    def test_capture_and_no_plant_teleport(self):
        cfg = Config(
            controller="capture", impulse_x=12, noise_position=0, noise_velocity=0
        )
        r, x = simulate(cfg)
        self.assertTrue(r["success"])
        self.assertLessEqual(r["max_route_shift_m"], 0.35)
        np.testing.assert_array_less(
            np.max(np.abs(x[:, [9, 10]] - x[:, [11, 12]]), axis=0), SAFE_HALF + 1e-8
        )
        for left, right in zip(x[:-1], x[1:]):
            dt = right[0] - left[0]
            A, B = transition(0.8, dt)
            state = np.array([left[1:3], left[3:5]])
            cop = left[[9, 10]] - left[[20, 21]] / (30 * (9.81 / 0.8))
            expected = state @ A.T + cop[:, None] * B
            actual = np.array([right[1:3], right[3:5]])
            np.testing.assert_allclose(actual, expected, atol=1e-10, rtol=0)

    def test_external_push_is_not_ground_friction(self):
        cfg = Config(
            controller="lqr",
            impulse_x=30.0,
            friction=0.4,
            noise_position=0,
            noise_velocity=0,
        )
        r, _ = simulate(cfg)
        self.assertGreater(r["duration_s"], cfg.push_time + 0.001)

    def test_zero_delay_prediction_equivalence_and_repeat(self):
        _, a = simulate(Config(controller="lqr", impulse_x=6, seed=7))
        _, b = simulate(Config(controller="lqr_predict", impulse_x=6, seed=7))
        _, c = simulate(Config(controller="lqr", impulse_x=6, seed=7))
        np.testing.assert_array_equal(a, b)
        np.testing.assert_array_equal(a, c)


if __name__ == "__main__":
    unittest.main()
