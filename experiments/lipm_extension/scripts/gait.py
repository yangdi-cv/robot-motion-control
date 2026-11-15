"""DCM stepping with double support and swing-admissible timing/location search.

The plant is still a reduced-order LIPM; contact load split is a feasible example,
not measured ground force. Full-body execution is evaluated separately.
"""

from dataclasses import dataclass, asdict
from pathlib import Path
import numpy as np, json, hashlib
from scipy.spatial import ConvexHull
from engine import Config, transition, Reference, SAFE_HALF, G
from trajectory import Swing


@dataclass(frozen=True)
class GaitConfig:
    method: str = "fixed"
    ds: float = 0.12
    step_time: float = 0.6
    step_length: float = 0.18
    step_width: float = 0.14
    steps: int = 10
    settle: float = 2.0
    dt: float = 0.001
    hz: int = 100
    height: float = 0.8
    impulse_x: float = 12.0
    impulse_y: float = 0.0
    push_time: float = 2.70
    push_duration: float = 0.1
    seed: int = 0
    noise_position: float = 0.001
    noise_velocity: float = 0.01
    swing_speed: float = 1.8
    swing_acceleration: float = 30.0
    cop_speed: float = 5.0
    mass: float = 30.0
    friction: float = 0.6


class Nominal:
    def __init__(self, cfg):
        self.cfg = cfg
        self.omega = np.sqrt(G / cfg.height)
        self.feet = np.array(
            [
                [i * cfg.step_length, (-1) ** i * cfg.step_width / 2]
                for i in range(cfg.steps)
            ]
        )
        self.duration = (cfg.steps - 1) * 0.75 + cfg.step_time + cfg.settle
        self.n = round(self.duration / cfg.dt)
        p = []
        for k in range(self.n + 1):
            t = k * cfg.dt
            i = min(int((t + 1e-10) / cfg.step_time), cfg.steps - 1)
            age = t - i * cfg.step_time
            alpha = 1.0
            if i > 0 and cfg.ds > 0:
                u = np.clip(age / cfg.ds, 0, 1)
                alpha = u * u * (3 - 2 * u)
            p.append(
                self.feet[i]
                if i == 0
                else (1 - alpha) * self.feet[i - 1] + alpha * self.feet[i]
            )
        self.cop = np.array(p)
        decay = np.exp(-self.omega * cfg.dt)
        xi = np.empty_like(self.cop)
        xi[-1] = self.feet[-1]
        for k in range(self.n - 1, -1, -1):
            xi[k] = decay * xi[k + 1] + (1 - decay) * self.cop[k]
        eta = xi[0].copy()
        self.state = np.empty((self.n + 1, 2, 2))
        for k in range(self.n + 1):
            self.state[k, :, 0] = (xi[k] + eta) / 2
            self.state[k, :, 1] = self.omega * (xi[k] - eta) / 2
            eta = decay * eta + (1 - decay) * self.cop[k]
        self.boundaries = np.array(
            [
                xi[min(round(i * cfg.step_time / cfg.dt), self.n)]
                for i in range(cfg.steps)
            ]
        )


def project(desired, feet):
    if len(feet) == 1:
        return np.clip(desired, feet[0] - SAFE_HALF, feet[0] + SAFE_HALF)
    corners = np.array(
        [
            p + SAFE_HALF * np.array([x, y])
            for p in feet
            for x in [-1, 1]
            for y in [-1, 1]
        ]
    )
    hull = ConvexHull(corners)
    if np.all(hull.equations[:, :2] @ desired + hull.equations[:, 2] <= 1e-10):
        return desired.copy()
    pts = corners[hull.vertices]
    best = None
    for a, b in zip(pts, np.roll(pts, -1, axis=0)):
        q = a + np.clip((desired - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1) * (b - a)
        if best is None or np.linalg.norm(q - desired) < np.linalg.norm(best - desired):
            best = q
    return best


def split(cop, old, new, preferred):
    delta = new - old
    lo = 0.0
    hi = 1.0
    for j in range(2):
        if abs(delta[j]) < 1e-12:
            assert abs(cop[j] - old[j]) <= SAFE_HALF[j] + 1e-8
        else:
            a = (cop[j] - old[j] - SAFE_HALF[j]) / delta[j]
            b = (cop[j] - old[j] + SAFE_HALF[j]) / delta[j]
            lo = max(lo, min(a, b))
            hi = min(hi, max(a, b))
    assert lo <= hi + 1e-8, (cop, old, new, lo, hi)
    alpha = np.clip(preferred, lo, hi)
    offset = cop - ((1 - alpha) * old + alpha * new)
    assert np.all(np.abs(offset) <= SAFE_HALF + 1e-8)
    return float(alpha), old + offset, new + offset


def simulate(cfg, save=None):
    ref = Nominal(cfg)
    omega = ref.omega
    A, B = transition(cfg.height, cfg.dt)
    stride = round(1 / (cfg.hz * cfg.dt))
    rng = np.random.default_rng(cfg.seed)
    noise = rng.normal(size=(ref.n + 1, 2, 2)) * [
        cfg.noise_position,
        cfg.noise_velocity,
    ]
    state = ref.state[0].copy()
    err = np.zeros((2, 2))
    phase = 0
    start = 0.0
    end = cfg.step_time
    shift = np.zeros(2)
    foot = ref.feet[0].copy()
    previous = foot.copy()
    nextfoot = ref.feet[1].copy()
    swingstart = np.array([-cfg.step_length, -cfg.step_width / 2])
    swing = Swing(swingstart, nextfoot, cfg.ds, end)
    initial_bounds = swing.bounds()
    reason = None
    if (
        initial_bounds[0] > cfg.swing_speed
        or initial_bounds[1] > cfg.swing_acceleration
    ):
        reason = "initial_swing_infeasible"
    cop = foot.copy()
    history = []
    states = []
    cop_hist = []
    loads = []
    feet_hist = []
    swing_hist = []
    touchdowns = []
    plans = []
    largest_v = initial_bounds[0]
    largest_a = initial_bounds[1]
    last_replan = -1.0
    last_touchdown = 0.0
    total_end = None
    timing_committed = False
    for k in range(ref.n):
        t = k * cfg.dt
        if phase < cfg.steps - 1 and t >= end - 1e-10:
            final_p, final_v, final_a = swing.sample(end)
            assert np.linalg.norm(final_p[:2] - nextfoot) < 1e-7
            previous = foot.copy()
            foot = nextfoot.copy()
            shift = foot - ref.feet[phase + 1]
            phase += 1
            start = end
            last_touchdown = t
            timing_committed = False
            touchdowns.append([t, *foot, *shift])
            if phase == cfg.steps - 1:
                total_end = t + cfg.settle
                end = total_end
            else:
                nextfoot = ref.feet[phase + 1] + shift
                swingstart = previous.copy()
                options = (
                    [cfg.step_time]
                    if cfg.method != "timing"
                    else [0.6, 0.65, 0.7, 0.75]
                )
                admitted = []
                for duration in options:
                    trial = Swing(
                        swingstart, nextfoot, start + cfg.ds, start + duration
                    )
                    bv, ba = trial.bounds()
                    if (
                        bv <= cfg.swing_speed + 1e-8
                        and ba <= cfg.swing_acceleration + 1e-8
                    ):
                        admitted.append((duration, trial))
                if admitted:
                    duration, swing = admitted[0]
                    end = start + duration
                    bounds = swing.bounds()
                    largest_v = max(largest_v, bounds[0])
                    largest_a = max(largest_a, bounds[1])
                else:
                    end = start + cfg.step_time
                    swing = Swing(swingstart, nextfoot, start + cfg.ds, end)
                    reason = "new_swing_infeasible"
        age = t - start
        in_ds = phase > 0 and age < cfg.ds - 1e-10
        support = [previous, foot] if in_ds else [foot]
        if k % stride == 0 and reason is None:
            obs = state + noise[k]
            xi = obs[:, 0] + obs[:, 1] / omega
            u = 1.0 if not in_ds else np.clip(age / cfg.ds, 0, 1)
            alpha = u * u * (3 - 2 * u)
            pref = foot if not in_ds else previous * (1 - alpha) + foot * alpha
            remain = max(0.0, end - t)
            if phase == cfg.steps - 1:
                xi_ref = foot
            else:
                boundary = ref.boundaries[phase + 1] + shift
                xi_ref = foot + np.exp(-omega * remain) * (boundary - foot)
                if in_ds:
                    count = round((start + cfg.ds - t) / cfg.dt)
                    elapsed = (np.arange(count) * cfg.dt + t - start) / cfg.ds
                    blending = elapsed * elapsed * (3 - 2 * elapsed)
                    sequence = (
                        previous[None, :] * (1 - blending[:, None])
                        + foot[None, :] * blending[:, None]
                    )
                    decay = np.exp(-omega * cfg.dt)
                    xi_end = foot + np.exp(-omega * (end - start - cfg.ds)) * (
                        boundary - foot
                    )
                    xi_ref = decay**count * xi_end + (1 - decay) * np.sum(
                        decay ** np.arange(count)[:, None] * sequence, axis=0
                    )
            desired = pref + (1 + 3 / omega) * (xi - xi_ref)
            candidate = project(desired, support)
            limited = np.clip(
                candidate, cop - cfg.cop_speed / cfg.hz, cop + cfg.cop_speed / cfg.hz
            )
            old_cop = cop.copy()
            cop = project(limited, support)
            if np.max(np.abs(cop - old_cop)) > cfg.cop_speed / cfg.hz + 1e-8:
                reason = "cop_rate_infeasible"
            if (
                phase < cfg.steps - 1
                and age >= max(cfg.ds + 0.02, 0.25)
                and remain > 0.06
                and cfg.method != "fixed"
                and t - last_replan >= 0.02 - 1e-10
                and np.linalg.norm(xi - xi_ref) > 0.01
            ):
                durations = (
                    [cfg.step_time]
                    if cfg.method == "spatial"
                    else (
                        [end - start]
                        if timing_committed
                        else [0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75]
                    )
                )
                choices = []
                for duration in durations:
                    duration = round(duration * cfg.hz) / cfg.hz
                    finish = start + duration
                    r = finish - t
                    if r < 0.08 or finish < swing.apex + 0.05:
                        continue
                    predicted = cop + np.exp(omega * r) * (xi - cop)
                    b = ref.boundaries[phase + 1] - ref.feet[phase + 1]
                    wanted = predicted - b
                    delta = wanted - foot
                    delta[0] = np.clip(delta[0], 0.04, 0.34)
                    sign = (-1) ** (phase + 1)
                    delta[1] = sign * np.clip(sign * delta[1], 0.10, 0.24)
                    landing = foot + delta
                    trial = swing.proposal(t, finish, landing)
                    vmax, amax = trial.bounds()
                    if (
                        vmax > cfg.swing_speed + 1e-8
                        or amax > cfg.swing_acceleration + 1e-8
                        or not trial.clearance_ok()
                    ):
                        continue
                    if phase + 2 < cfg.steps:
                        future_landing = (
                            ref.feet[phase + 2] + landing - ref.feet[phase + 1]
                        )
                        future_times = (
                            [cfg.step_time]
                            if cfg.method != "timing"
                            else [0.6, 0.65, 0.7, 0.75]
                        )
                        future_ok = any(
                            (
                                lambda bounds: bounds[0] <= cfg.swing_speed + 1e-8
                                and bounds[1] <= cfg.swing_acceleration + 1e-8
                            )(
                                Swing(
                                    foot, future_landing, 0.0, duration - cfg.ds
                                ).bounds()
                            )
                            for duration in future_times
                        )
                        if not future_ok:
                            continue
                    dcm_error = predicted - (landing + b)
                    correction = landing - (ref.feet[phase + 1] + shift)
                    cost = (
                        100 * np.sum(dcm_error**2)
                        + np.sum(correction**2)
                        + 1.0 * (duration - cfg.step_time) ** 2
                        + 0.2 * (finish - end) ** 2
                    )
                    choices.append((cost, finish, landing, trial, vmax, amax))
                if choices:
                    _, end, nextfoot, swing, vmax, amax = min(
                        choices, key=lambda x: x[0]
                    )
                    if cfg.method == "timing":
                        timing_committed = True
                    largest_v = max(largest_v, vmax)
                    largest_a = max(largest_a, amax)
                    plans.append([t, end, *nextfoot, vmax, amax])
                last_replan = t
        if phase < cfg.steps - 1:
            if t < start + cfg.ds:
                sp = np.r_[swingstart, 0.0]
                sv = np.zeros(3)
                sa = np.zeros(3)
            else:
                sp, sv, sa = swing.sample(t)
        else:
            sp = np.r_[previous, 0.0]
            sv = np.zeros(3)
            sa = np.zeros(3)
        largest_v = max(largest_v, np.linalg.norm(sv))
        largest_a = max(largest_a, np.linalg.norm(sa))
        if in_ds:
            load, pold, pnew = split(cop, previous, foot, np.clip(age / cfg.ds, 0, 1))
        else:
            load = 1.0
            pold = previous
            pnew = cop
        force = (
            np.array([cfg.impulse_x, cfg.impulse_y]) / cfg.push_duration
            if cfg.push_time <= t < cfg.push_time + cfg.push_duration - 1e-10
            else np.zeros(2)
        )
        ground_acc = G / cfg.height * (state[:, 0] - cop)
        if np.linalg.norm(ground_acc) > cfg.friction * G + 1e-8:
            reason = "friction_infeasible"
        if min(np.linalg.norm(state[:, 0] - p) for p in support) > 0.35:
            reason = "support_reach_exceeded"
        if np.linalg.norm(state[:, 1]) > 2.5:
            reason = "velocity_exceeded"
        if np.linalg.norm(shift) > 0.35:
            reason = "route_shift_exceeded"
        if phase % 2 == 0:
            left = np.r_[foot, 0.0]
            right = sp.copy()
        else:
            left = sp.copy()
            right = np.r_[foot, 0.0]
        states.append(state.copy())
        cop_hist.append(cop.copy())
        loads.append([int(in_ds), load, *pold, *pnew])
        feet_hist.append([*previous, *foot])
        swing_hist.append([*sp, *sv, *sa, *left, *right])
        if k % stride == 0 or reason:
            history.append(
                [
                    t,
                    *state[0],
                    *state[1],
                    *cop,
                    phase,
                    int(in_ds),
                    end,
                    *foot,
                    *shift,
                    *sp,
                ]
            )
        if reason:
            break
        if total_end is not None and t >= total_end - 1e-10:
            break
        relative = cop - ref.cop[k] - force / (cfg.mass * (G / cfg.height))
        err = err @ A.T + relative[:, None] * B
        state = ref.state[k + 1] + err
    states = np.asarray(states)
    cop_hist = np.asarray(cop_hist)
    times = np.arange(len(states)) * cfg.dt
    tail = times > times[-1] - 0.5
    terminal_distance = float(np.linalg.norm(states[-1, :, 0] - foot))
    terminal_speed = float(np.linalg.norm(states[-1, :, 1]))
    success = (
        reason is None
        and phase == cfg.steps - 1
        and terminal_distance < 0.02
        and terminal_speed < 0.08
    )
    if reason is None and not success:
        reason = "terminal_not_recovered"
    cop_step = np.max(np.abs(np.diff(cop_hist, axis=0))) if len(cop_hist) > 1 else 0.0
    result = {
        "config": asdict(cfg),
        "success": success,
        "reason": reason,
        "duration_s": float(times[-1]),
        "touchdowns": touchdowns,
        "plans": len(plans),
        "terminal_distance_m": terminal_distance,
        "terminal_speed_m_s": terminal_speed,
        "terminal_original_route_error_m": float(
            np.linalg.norm(states[-1, :, 0] - ref.feet[-1])
        ),
        "max_swing_speed_bound_m_s": float(largest_v),
        "max_swing_acceleration_bound_m_s2": float(largest_a),
        "max_physics_cop_jump_m": float(cop_step),
        "initial_swing_bounds": list(initial_bounds),
        "tail_distance_rmse_m": float(
            np.sqrt(np.mean(np.sum((states[tail, :, 0] - foot) ** 2, axis=1)))
        ),
    }
    if save:
        save = Path(save)
        save.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            save / "physics.npz",
            state=states,
            cop=cop_hist,
            feet=np.asarray(feet_hist),
            load=np.asarray(loads),
            swing=np.asarray(swing_hist),
        )
        np.savez_compressed(
            save / "trace.npz", data=np.asarray(history), plans=np.asarray(plans)
        )
        (save / "result.json").write_text(json.dumps(result, indent=2))
    return result
