"""Causal horizontal CoM feedback and signed world-frame ground-contact wrenches."""

import collections
import numpy as np
import mujoco


class CoMChannel:
    def __init__(self, cfg):
        self.cfg = cfg
        self.queue = collections.deque()
        self.last = None
        self.next_sample = 0.0
        self.rng = np.random.default_rng(cfg.seed)

    def sample(self, t, com, vel):
        if t + 1e-10 < self.next_sample:
            return
        noisy = com.copy() + self.rng.normal(0, self.cfg.position_noise, 2)
        velocity = vel.copy() + self.rng.normal(0, self.cfg.velocity_noise, 2)
        value = (t, noisy, velocity)
        self.queue.append((t + self.cfg.delay, value))
        # Pre-start bootstrap is timestamped and noisy, never replaced by truth.
        if self.last is None:
            self.last = value
        self.next_sample += 1 / self.cfg.sensor_hz

    def read(self, t):
        while self.queue and self.queue[0][0] <= t + 1e-10:
            _, self.last = self.queue.popleft()
        stamp, position, velocity = self.last
        age = t - stamp
        predicted = position.copy()
        if self.cfg.feedback == "predict":
            predicted += age * velocity
        return predicted, velocity.copy(), age


def contact_wrenches(model, data, sites):
    """[Fx,Fz,My] acting ON each foot, about its sole site, in world axes."""
    w = np.zeros((2, 3))
    geom_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, s + "_sole_geom")
        for s in ["left", "right"]
    ]
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    for i in range(data.ncon):
        con = data.contact[i]
        for side, foot in enumerate(geom_ids):
            if {con.geom1, con.geom2} != {ground, foot}:
                continue
            local = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, local)
            # MuJoCo convention: returned wrench acts on geom2.
            sign = 1 if con.geom2 == foot else -1
            rotation = con.frame.reshape(3, 3).T
            force = sign * (rotation @ local[:3])
            moment = sign * (rotation @ local[3:]) + np.cross(
                con.pos - data.site_xpos[sites[side]], force
            )
            w[side] += [force[0], force[2], moment[1]]
    return w


def resultant_cop(positions, wrenches):
    denominator = wrenches[:, 1].sum()
    if denominator < 1:
        return float("nan")
    return float(
        np.sum(
            positions[:, 0] * wrenches[:, 1]
            - positions[:, 1] * wrenches[:, 0]
            - wrenches[:, 2]
        )
        / denominator
    )
