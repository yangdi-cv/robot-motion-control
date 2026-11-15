"""Constrained planar LIPM with scheduled support exchange and causal feedback.

Rows of a state are horizontal x/y axes; columns are position/velocity.
Support exchange is an ideal hybrid event. This is not a rigid-body biped.
"""

from dataclasses import dataclass, asdict
from pathlib import Path
import hashlib, json
from functools import lru_cache
import numpy as np
from scipy.linalg import solve_continuous_are


@dataclass(frozen=True)
class Config:
    controller: str = "lqr"
    hz: int = 100
    delay: float = 0.0
    impulse_x: float = 0.0
    impulse_y: float = 0.0
    push_time: float = 2.70
    push_duration: float = 0.10
    seed: int = 0
    noise_position: float = 0.001
    noise_velocity: float = 0.01
    bias_position: float = 0.0
    height: float = 0.8
    model_height: float = 0.8
    mass: float = 30.0
    friction: float = 0.6
    dt: float = 0.001
    step_time: float = 0.6
    step_length: float = 0.18
    step_width: float = 0.14
    steps: int = 12
    settle: float = 2.0


G = 9.81
HALF_FOOT = np.array([0.10, 0.05])
SAFE_HALF = HALF_FOOT - 0.005
CONTROLLERS = ["pd", "lqr", "capture", "lqr_predict", "capture_predict"]


def transition(height, dt):
    omega = np.sqrt(G / height)
    c = np.cosh(omega * dt)
    s = np.sinh(omega * dt)
    return np.array([[c, s / omega], [omega * s, c]]), np.array([1 - c, -omega * s])


def gains(height):
    w2 = G / height
    A = np.array([[0.0, 1.0], [w2, 0.0]])
    B = np.array([[0.0], [-w2]])
    Q = np.diag([20.0, 2.0])
    R = np.array([[1.0]])
    P = solve_continuous_are(A, B, Q, R)
    K = (B.T @ P)[0]
    return K, np.linalg.eigvals(A - B @ K[None, :])


class Reference:
    def __init__(self, cfg):
        self.cfg = cfg
        self.omega = np.sqrt(G / cfg.model_height)
        self.duration = cfg.steps * cfg.step_time + cfg.settle
        self.n = round(self.duration / cfg.dt)
        self.phase_steps = round(cfg.step_time / cfg.dt)
        self.feet = np.array(
            [
                [i * cfg.step_length, ((-1) ** i) * cfg.step_width / 2]
                for i in range(cfg.steps)
            ]
        )
        self.xi_boundary = np.empty_like(self.feet)
        self.xi_boundary[-1] = self.feet[-1]
        for i in range(cfg.steps - 2, -1, -1):
            self.xi_boundary[i] = self.feet[i] + np.exp(-self.omega * cfg.step_time) * (
                self.xi_boundary[i + 1] - self.feet[i]
            )
        self.state = np.zeros((self.n + 1, 2, 2))
        eta = self.xi_boundary[0].copy()
        for k in range(self.n + 1):
            phase = min(k // self.phase_steps, cfg.steps - 1)
            tau = k * cfg.dt - phase * cfg.step_time
            foot = self.feet[phase]
            xi = foot + np.exp(self.omega * tau) * (self.xi_boundary[phase] - foot)
            self.state[k, :, 0] = (xi + eta) / 2
            self.state[k, :, 1] = self.omega * (xi - eta) / 2
            eta = foot + np.exp(-self.omega * cfg.dt) * (eta - foot)
        self.xi = self.state[:, :, 0] + self.state[:, :, 1] / self.omega


@lru_cache(maxsize=32)
def cached_reference(
    model_height, dt, step_time, step_length, step_width, steps, settle
):
    ref = Reference(
        Config(
            model_height=model_height,
            dt=dt,
            step_time=step_time,
            step_length=step_length,
            step_width=step_width,
            steps=steps,
            settle=settle,
        )
    )
    ref.state.setflags(write=False)
    ref.feet.setflags(write=False)
    ref.xi_boundary.setflags(write=False)
    return ref


def reference_for(cfg):
    return cached_reference(
        cfg.model_height,
        cfg.dt,
        cfg.step_time,
        cfg.step_length,
        cfg.step_width,
        cfg.steps,
        cfg.settle,
    )


class Controller:
    def __init__(self, cfg, reference):
        self.cfg = cfg
        self.ref = reference
        self.K, self.poles = gains(cfg.model_height)
        self.omega = np.sqrt(G / cfg.model_height)
        self.shift = np.zeros(2)
        self.phase = 0
        self.foot = reference.feet[0].copy()
        self.next_foot = reference.feet[1].copy()
        self.landings = [self.foot.copy()]
        self.saturation = 0
        self.step_adjustments = []

    def touchdown(self, phase):
        if phase == self.phase:
            return
        assert phase == self.phase + 1
        previous = self.foot.copy()
        if "capture" in self.cfg.controller:
            self.foot = self.next_foot.copy()
            self.shift = self.foot - self.ref.feet[phase]
        else:
            self.foot = self.ref.feet[phase].copy()
        delta = self.foot - previous
        assert 0.04 - 1e-9 <= delta[0] <= 0.34 + 1e-9
        assert 0.10 - 1e-9 <= abs(delta[1]) <= 0.24 + 1e-9
        self.phase = phase
        self.landings.append(self.foot.copy())
        self.step_adjustments.append(
            float(np.linalg.norm(self.foot - self.ref.feet[phase]))
        )
        if phase < self.cfg.steps - 1:
            self.next_foot = self.ref.feet[phase + 1] + self.shift

    def command(self, state, k, control_dt):
        target = self.ref.state[k].copy()
        target[:, 0] += self.shift
        error = state - target
        if self.cfg.controller == "open_loop":
            offset = np.zeros(2)
        elif self.cfg.controller == "pd":
            w2 = self.omega**2
            offset = (1 + 3.0**2 / w2) * error[:, 0] + (2 * 0.9 * 3.0 / w2) * error[
                :, 1
            ]
        else:
            offset = -error @ self.K
        desired = self.foot + offset
        cop = np.clip(desired, self.foot - SAFE_HALF, self.foot + SAFE_HALF)
        clipped = bool(np.any(np.abs(cop - desired) > 1e-10))
        self.saturation += int(clipped)
        if "capture" in self.cfg.controller and self.phase < self.cfg.steps - 1:
            remaining = (self.phase + 1) * self.cfg.step_time - k * self.cfg.dt
            if 0.0 < remaining <= 0.15 + 1e-9:
                xi = state[:, 0] + state[:, 1] / self.omega
                predicted = cop + np.exp(self.omega * remaining) * (xi - cop)
                boundary = self.ref.xi_boundary[self.phase + 1] + self.shift
                correction = predicted - boundary
                correction[np.abs(correction) < 0.003] = 0.0
                correction = np.clip(correction, [-0.10, -0.06], [0.10, 0.06])
                desired_next = self.ref.feet[self.phase + 1] + self.shift + correction
                difference = desired_next - self.foot
                difference[0] = np.clip(difference[0], 0.04, 0.34)
                sign = (-1) ** (self.phase + 1)
                difference[1] = sign * np.clip(sign * difference[1], 0.10, 0.24)
                desired_next = self.foot + difference
                self.next_foot += np.clip(
                    desired_next - self.next_foot, -0.8 * control_dt, 0.8 * control_dt
                )
                # Re-project after rate limiting, keeping every planned landing feasible.
                diff = self.next_foot - self.foot
                diff[0] = np.clip(diff[0], 0.04, 0.34)
                diff[1] = sign * np.clip(sign * diff[1], 0.10, 0.24)
                self.next_foot = self.foot + diff
        return cop, clipped


def predictor(observed, history, start, end, cfg, kernels):
    lag = end - start
    if lag == 0:
        return observed.copy()
    if lag not in kernels:
        A_L, _ = transition(cfg.model_height, lag * cfg.dt)
        _, B = transition(cfg.model_height, cfg.dt)
        weights = np.stack(
            [
                transition(cfg.model_height, (lag - 1 - i) * cfg.dt)[0] @ B
                for i in range(lag)
            ],
            axis=1,
        )
        kernels[lag] = (A_L, weights)
    A_L, weights = kernels[lag]
    return observed @ A_L.T + history[start:end].T @ weights.T


COLUMNS = [
    "t",
    "cx",
    "vx",
    "cy",
    "vy",
    "ref_cx",
    "ref_vx",
    "ref_cy",
    "ref_vy",
    "cop_x",
    "cop_y",
    "foot_x",
    "foot_y",
    "xi_x",
    "xi_y",
    "shift_x",
    "shift_y",
    "age",
    "saturated",
    "phase",
    "force_x",
    "force_y",
    "next_foot_x",
    "next_foot_y",
]


def simulate(cfg, save=None):
    ref = reference_for(cfg)
    ctl = Controller(cfg, ref)
    stride = round(1 / (cfg.hz * cfg.dt))
    assert abs(stride * cfg.dt - 1 / cfg.hz) < 1e-10
    delay_steps = round(cfg.delay / cfg.dt)
    A, B = transition(cfg.height, cfg.dt)
    # Exact ZOH response to a force, expressed as an equivalent support displacement.
    rng = np.random.default_rng(cfg.seed)
    noise = rng.normal(size=(ref.n + 1, 2, 2)) * np.array(
        [cfg.noise_position, cfg.noise_velocity]
    )
    state = ref.state[0].copy()
    error_state = np.zeros((2, 2))
    states = np.zeros((ref.n + 1, 2, 2))
    states[0] = state
    cop_history = np.zeros((ref.n, 2))
    shift_history = np.zeros((ref.n + 1, 2))
    foot_history = np.zeros((ref.n + 1, 2))
    cop = ctl.foot.copy()
    kernels = {}
    logs = []
    reason = None
    failed_at = None
    push_end = cfg.push_time + cfg.push_duration
    clipped = False
    observed_age = 0.0
    for k in range(ref.n):
        t = k * cfg.dt
        phase = min(k // ref.phase_steps, cfg.steps - 1)
        ctl.touchdown(phase)
        shift_history[k] = ctl.shift
        foot_history[k] = ctl.foot
        if k % stride == 0:
            source = max(0, k - delay_steps)
            obs = states[source] + noise[source]
            obs[:, 0] += cfg.bias_position
            observed_age = (k - source) * cfg.dt
            if cfg.controller.endswith("_predict"):
                obs = predictor(obs, cop_history, source, k, cfg, kernels)
            cop, clipped = ctl.command(obs, k, stride * cfg.dt)
        # Control times align with every support transition in the main protocol.
        assert np.all(np.abs(cop - ctl.foot) <= SAFE_HALF + 1e-9)
        cop_history[k] = cop
        force = np.zeros(2)
        if cfg.push_time - 1e-10 <= t < push_end - 1e-10:
            force = np.array([cfg.impulse_x, cfg.impulse_y]) / cfg.push_duration
        contact_acc = G / cfg.height * (state[:, 0] - cop)
        # External force at CoM contributes acceleration, not ground friction demand.
        if np.linalg.norm(contact_acc) > cfg.friction * G + 1e-9:
            reason = "friction_infeasible"
            failed_at = t
        if np.linalg.norm(state[:, 0] - ctl.foot) > 0.35:
            reason = "support_reach_exceeded"
            failed_at = t
        if np.linalg.norm(state[:, 1]) > 2.5:
            reason = "velocity_exceeded"
            failed_at = t
        if np.linalg.norm(ctl.shift) > 0.35:
            reason = "route_shift_exceeded"
            failed_at = t
        if k % stride == 0 or reason:
            xi = state[:, 0] + state[:, 1] / ref.omega
            logs.append(
                [
                    t,
                    *state[0],
                    *state[1],
                    *ref.state[k, 0],
                    *ref.state[k, 1],
                    *cop,
                    *ctl.foot,
                    *xi,
                    *ctl.shift,
                    observed_age,
                    int(clipped),
                    phase,
                    *force,
                    *ctl.next_foot,
                ]
            )
        if reason:
            break
        nominal_cop = ref.feet[phase]
        relative_cop = cop - nominal_cop - force / (cfg.mass * (G / cfg.height))
        mismatch = (
            np.zeros((2, 2))
            if cfg.height == cfg.model_height
            else ref.state[k] @ A.T + nominal_cop[:, None] * B - ref.state[k + 1]
        )
        # Variation of constants avoids catastrophic absolute-coordinate cancellation
        # in a long unstable open-loop integration; the dynamics are unchanged.
        error_state = error_state @ A.T + relative_cop[:, None] * B + mismatch
        state = ref.state[k + 1] + error_state
        states[k + 1] = state
        shift_history[k + 1] = ctl.shift
        foot_history[k + 1] = ctl.foot
    actual_duration = (k + 1) * cfg.dt if reason is None else float(failed_at)
    # Terminal sample is a real propagated state, not a frozen padded trajectory.
    if reason is None:
        target = ref.state[-1]
        xi = state[:, 0] + state[:, 1] / ref.omega
        logs.append(
            [
                ref.duration,
                *state[0],
                *state[1],
                *target[0],
                *target[1],
                *cop,
                *ctl.foot,
                *xi,
                *ctl.shift,
                observed_age,
                int(clipped),
                ctl.phase,
                0.0,
                0.0,
                *ctl.next_foot,
            ]
        )
    logs = np.asarray(logs)
    used = ref.n + 1 if reason is None else k + 1
    times = np.arange(used) * cfg.dt
    measured_states = states[:used]
    position = measured_states[:, :, 0]
    velocity = measured_states[:, :, 1]
    nominal = ref.state[:used, :, 0]
    nominal_v = ref.state[:used, :, 1]
    shift = shift_history[:used]
    tracking = position - nominal - shift
    vm = velocity - nominal_v
    band = (np.linalg.norm(tracking, axis=1) < 0.02) & (
        np.linalg.norm(vm, axis=1) < 0.08
    )
    recovery = None
    if reason is None and (cfg.impulse_x or cfg.impulse_y):
        length = 0
        window = round(0.5 / cfg.dt)
        for j in range(int(np.ceil(push_end / cfg.dt)), used):
            length = length + 1 if band[j] else 0
            if length >= window + 1:
                recovery = float(times[j - window] - push_end)
                break
    success = reason is None
    if success and np.linalg.norm(position[-1] - nominal[-1]) > 0.35:
        success = False
        reason = "terminal_route_error"
    tail = times >= times[-1] - 0.5
    result = {
        "evaluation_dt_s": cfg.dt,
        "config": asdict(cfg),
        "success": success,
        "reason": reason,
        "duration_s": actual_duration,
        "steps_completed": ctl.phase,
        "recovery_s": recovery,
        "tracking_rmse_m": float(np.sqrt(np.mean(np.sum(tracking**2, axis=1)))),
        "nominal_route_rmse_m": float(
            np.sqrt(np.mean(np.sum((position - nominal) ** 2, axis=1)))
        ),
        "terminal_route_error_m": float(np.linalg.norm(position[-1] - nominal[-1])),
        "terminal_speed_m_s": float(np.linalg.norm(velocity[-1])),
        "tail_tracking_rmse_m": float(
            np.sqrt(np.mean(np.sum(tracking[tail] ** 2, axis=1)))
        ),
        "tail_velocity_error_m_s": float(
            np.sqrt(np.mean(np.sum(vm[tail] ** 2, axis=1)))
        ),
        "max_route_shift_m": float(np.max(np.linalg.norm(shift, axis=1))),
        "saturation_fraction": float(np.mean(logs[:, 18])),
        "max_cop_relative_m": np.max(
            np.abs(logs[:, [9, 10]] - logs[:, [11, 12]]), axis=0
        ).tolist(),
        "landings": np.array(ctl.landings).tolist(),
        "gain_lqr": ctl.K.tolist(),
        "nominal_closed_loop_poles": ctl.poles.tolist(),
    }
    # Completing the horizon without reaching the terminal balance tube is not success.
    if success and (
        result["tail_tracking_rmse_m"] > 0.02
        or result["tail_velocity_error_m_s"] > 0.08
    ):
        result["success"] = False
        result["reason"] = "terminal_balance_not_recovered"
    if save:
        save = Path(save)
        save.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(save / "trace.npz", data=logs, columns=np.array(COLUMNS))
        np.savez_compressed(
            save / "physics.npz",
            state=states[:used],
            cop=cop_history[: used - 1],
            shift=shift_history[:used],
            foot=foot_history[:used],
        )
        (save / "result.json").write_text(json.dumps(result, indent=2))
    return result, logs


def key(cfg):
    return hashlib.sha256(json.dumps(asdict(cfg), sort_keys=True).encode()).hexdigest()[
        :16
    ]
