"""Physical task criteria, independent of QP status and visual appearance."""

import numpy as np
from scipy.spatial.transform import Rotation


def longest(mask, dt):
    best = current = 0
    for value in mask:
        current = current + 1 if value else 0
        best = max(best, current)
    return best * dt


def evaluate(arrays, cfg, mass):
    t = arrays["time"]
    feet = arrays["feet"]
    forces = arrays["actual_wrench"][:, :, 2]
    initialfeet = feet[0]
    error = arrays["com"] - arrays["reference_com"]
    footerror = np.linalg.norm(feet - arrays["reference_feet"], axis=2)
    orientation = Rotation.from_quat(arrays["qpos"][:, [4, 5, 6, 3]]).as_euler("xyz")
    result = dict(
        com_rmse=np.sqrt(np.mean(error**2, axis=0)).tolist(),
        max_foot_tracking_error=float(footerror.max()),
        max_roll_pitch=float(np.max(np.abs(orientation[:, :2]))),
        forward_progress=float(arrays["com"][-1, 0] - arrays["com"][0, 0]),
        terminal_com_error=float(np.linalg.norm(error[-1])),
        landings=[],
        single_support_seconds=[],
        acceptance_failures=[],
    )
    if result["max_foot_tracking_error"] > 0.025:
        result["acceptance_failures"].append("foot_tracking")
    if result["max_roll_pitch"] > 0.2:
        result["acceptance_failures"].append("tilt")
    if result["terminal_com_error"] > 0.02:
        result["acceptance_failures"].append("terminal_com")
    if max(result["com_rmse"][:2]) > 0.02:
        result["acceptance_failures"].append("com_tracking")
    count = (
        cfg.walk_steps
        if cfg.scenario == "walk"
        else (1 if cfg.scenario in ["single", "step"] else 0)
    )
    for k in range(count):
        swing = (1 - k % 2) if cfg.scenario == "walk" else 1 - cfg.support_side
        stance = 1 - swing
        start = 7 * k if cfg.scenario == "walk" else 0
        hold = (t >= start + 2.4) & (t < start + 3.8)
        good = (
            hold
            & (feet[:, swing, 2] > 0.03)
            & (forces[:, swing] < 5)
            & (forces[:, stance] > 0.8 * mass * 9.81)
        )
        duration = longest(good, cfg.dt)
        result["single_support_seconds"].append(duration)
        if duration < 0.5:
            result["acceptance_failures"].append(f"single_support_{k}")
        target = initialfeet[swing].copy()
        if cfg.scenario in ["walk", "step"]:
            target[0] += (k + 1) * cfg.step_distance
        touchdown = start + 4.6
        # Confirm real load before the following step, not just a hovering foot.
        window = (t >= touchdown - 0.05) & (t < start + 6.9)
        landed = (
            window
            & (np.linalg.norm(feet[:, swing, :2] - target[:2], axis=1) < 0.015)
            & (np.abs(feet[:, swing, 2]) < 0.008)
            & (forces[:, swing] > 10)
        )
        ok = longest(landed, cfg.dt) >= 0.05
        result["landings"].append(bool(ok))
        if not ok:
            result["acceptance_failures"].append(f"landing_{k}")
    if count:
        final = t >= cfg.duration - 0.5
        if (
            not np.any(final)
            or longest(final & (forces.min(axis=1) > 10), cfg.dt) < 0.1
        ):
            result["acceptance_failures"].append("final_double_support")
    result["criteria_passed"] = not result["acceptance_failures"]
    return result
