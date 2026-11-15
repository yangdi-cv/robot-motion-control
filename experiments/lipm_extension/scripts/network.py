"""Discrete sensor packets in model time, including reordering and stale data.

No socket or hardware connection: this module isolates timing/estimation assumptions.
"""

from dataclasses import dataclass, asdict
import heapq, json, hashlib
import numpy as np
from engine import Config, Reference, simulate


@dataclass(frozen=True)
class NetworkConfig(Config):
    sensor_hz: int = 25
    jitter: float = 0.0
    loss: float = 0.0
    burst_start: float = 2.60
    burst_duration: float = 0.0
    color_tau: float = 0.0
    drift_position: float = 0.0
    drift_velocity: float = 0.0
    max_age: float = 0.20


class Observer:
    def __init__(self, cfg):
        self.cfg = cfg
        ref = Reference(cfg)
        self.n = ref.n
        self.stride = round(1 / (cfg.sensor_hz * cfg.dt))
        assert abs(self.stride * cfg.dt - 1 / cfg.sensor_hz) < 1e-10
        rng = np.random.default_rng(cfg.seed)
        noise = rng.normal(size=(ref.n + 1, 2, 2)) * [
            cfg.noise_position,
            cfg.noise_velocity,
        ]
        if cfg.color_tau > 0:
            a = np.exp(-cfg.dt / cfg.color_tau)
            for k in range(1, len(noise)):
                noise[k] = a * noise[k - 1] + np.sqrt(1 - a * a) * noise[k]
        drift = (
            np.random.default_rng(cfg.seed + 10000).normal(size=noise.shape)
            * [cfg.drift_position, cfg.drift_velocity]
            * np.sqrt(cfg.dt)
        )
        self.noise = noise + np.cumsum(drift, axis=0)
        self.noise[:, :, 0] += cfg.bias_position
        self.random = np.random.default_rng(cfg.seed + 1000)
        self.queue = []
        self.latest = None
        self.generated = 0
        self.received = 0
        self.dropped = 0
        self.reordered = 0
        self.last_read = None
        self.reused = 0
        self.log = []

    def update(self, k, state):
        if self.latest is None:
            # One pre-start measured frame, noisy rather than a perfect initial state.
            self.latest = (0, state.copy() + self.noise[0])
        if k % self.stride == 0:
            self.generated += 1
            t = k * self.cfg.dt
            lose = (
                self.random.random() < self.cfg.loss
                or self.cfg.burst_start
                <= t
                < self.cfg.burst_start + self.cfg.burst_duration - 1e-10
            )
            jitter = self.random.uniform(-self.cfg.jitter, self.cfg.jitter)
            if lose:
                self.dropped += 1
            else:
                arrival = k + round(max(0.0, self.cfg.delay + jitter) / self.cfg.dt)
                heapq.heappush(self.queue, (arrival, k, state.copy() + self.noise[k]))
        while self.queue and self.queue[0][0] <= k:
            arrival, source, measurement = heapq.heappop(self.queue)
            if source > self.latest[0]:
                self.latest = (source, measurement)
                self.received += 1
            else:
                self.reordered += 1

    def read(self, k):
        source, obs = self.latest
        reused = self.last_read == source
        self.reused += int(reused)
        self.last_read = source
        self.log.append(
            [
                k * self.cfg.dt,
                source * self.cfg.dt,
                (k - source) * self.cfg.dt,
                int(reused),
                self.received,
                self.dropped,
                self.reordered,
            ]
        )
        assert source <= k
        return source, obs.copy()

    def metrics(self):
        a = np.asarray(self.log)
        return {
            "sensor_frames": self.generated,
            "accepted_frames": self.received,
            "dropped_frames": self.dropped,
            "ignored_old_or_duplicate_frames": self.reordered,
            "control_updates": len(a),
            "reused_fraction": self.reused / len(a),
            "age_p95_s": float(np.quantile(a[:, 2], 0.95)),
            "age_max_s": float(a[:, 2].max()),
            "warm_start": "one noisy pre-start frame; late duplicate of source0 ignored",
            "timing": "model time; no real network transport",
        }


def run(cfg, save=None):
    return simulate(cfg, save, Observer(cfg))


def key(cfg):
    return hashlib.sha256(json.dumps(asdict(cfg), sort_keys=True).encode()).hexdigest()[
        :16
    ]
