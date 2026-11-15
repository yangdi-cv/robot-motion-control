"""Continuous quintic swing trajectory and exact component extrema bounds."""

import math
import numpy as np


def quintic(p0, v0, a0, p1, duration):
    T = duration
    c = np.zeros((6, len(np.atleast_1d(p0))))
    c[0] = p0
    c[1] = np.asarray(v0) * T
    c[2] = np.asarray(a0) * T * T / 2
    rhs = np.stack([np.asarray(p1) - c[:3].sum(axis=0), -c[1] - 2 * c[2], -2 * c[2]])
    c[3:] = np.linalg.solve(np.array([[1.0, 1.0, 1.0], [3, 4, 5], [6, 12, 20]]), rhs)
    return c


def evaluate(c, t, duration):
    u = np.clip(t / duration, 0, 1)
    p = np.array([u**i for i in range(6)]) @ c
    v = np.array([i * u ** (i - 1) if i else 0 for i in range(6)]) @ c / duration
    a = (
        np.array([i * (i - 1) * u ** (i - 2) if i > 1 else 0 for i in range(6)])
        @ c
        / duration**2
    )
    return p, v, a


def component_bounds(c, duration):
    powers = np.arange(1, 6)
    v = c[1:] * powers[:, None] / duration
    a = v[1:] * np.arange(1, 5)[:, None] / duration
    vmax = []
    amax = []
    for axis in range(c.shape[1]):
        for poly, out in [(v[:, axis], vmax), (a[:, axis], amax)]:
            derivative = np.arange(1, len(poly)) * poly[1:]
            roots = np.polynomial.polynomial.polyroots(derivative)
            points = [0.0, 1.0] + [
                float(r.real) for r in roots if abs(r.imag) < 1e-9 and 0 < r.real < 1
            ]
            out.append(
                max(abs(np.polynomial.polynomial.polyval(u, poly)) for u in points)
            )
    return float(np.linalg.norm(vmax)), float(np.linalg.norm(amax))


class Swing:
    def __init__(self, start, landing, lift, end, clearance=0.06):
        self.lift = lift
        self.end = end
        self.apex = lift + (end - lift) / 2
        self.clearance = clearance
        self.position = np.r_[start, 0.0]
        self.velocity = np.zeros(3)
        self.acceleration = np.zeros(3)
        self.time = lift
        self.target = np.array(landing)
        self.build(self.position, self.velocity, self.acceleration, lift, end, landing)

    def build(self, p, v, a, now, end, landing):
        self.start = now
        self.end = end
        self.target = np.array(landing)
        self.xy = quintic(p[:2], v[:2], a[:2], landing, end - now)
        self.zparts = []
        if now < self.apex - 1e-6 and self.apex < end - 0.04:
            up = quintic(p[2:], v[2:], a[2:], [self.clearance], self.apex - now)
            down = quintic([self.clearance], [0.0], [0.0], [0.0], end - self.apex)
            self.zparts = [(now, self.apex, up), (self.apex, end, down)]
        else:
            self.zparts = [(now, end, quintic(p[2:], v[2:], a[2:], [0.0], end - now))]

    def sample(self, t):
        xy, vxy, axy = evaluate(self.xy, t - self.start, self.end - self.start)
        seg = next((s for s in self.zparts if t <= s[1] + 1e-10), self.zparts[-1])
        z, vz, az = evaluate(seg[2], t - seg[0], seg[1] - seg[0])
        return np.r_[xy, z], np.r_[vxy, vz], np.r_[axy, az]

    def bounds(self):
        v, a = component_bounds(self.xy, self.end - self.start)
        zb = [component_bounds(s[2], s[1] - s[0]) for s in self.zparts]
        return np.hypot(v, max(q[0] for q in zb)), np.hypot(a, max(q[1] for q in zb))

    def height_range(self):
        values = []
        for begin, end, c in self.zparts:
            derivative = np.arange(1, 6) * c[1:, 0]
            roots = np.polynomial.polynomial.polyroots(derivative)
            points = [0.0, 1.0] + [
                float(r.real) for r in roots if abs(r.imag) < 1e-9 and 0 < r.real < 1
            ]
            values.extend(np.polynomial.polynomial.polyval(points, c[:, 0]))
        return float(min(values)), float(max(values))

    def clearance_ok(self):
        low, high = self.height_range()
        return low >= -1e-9 and high <= self.clearance + 0.02 + 1e-9

    def proposal(self, now, end, landing):
        p, v, a = self.sample(now)
        trial = object.__new__(Swing)
        trial.lift = self.lift
        trial.apex = self.apex
        trial.clearance = self.clearance
        trial.build(p, v, a, now, end, landing)
        return trial
