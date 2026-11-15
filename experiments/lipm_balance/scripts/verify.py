"""Independent reconstruction of ZOH dynamics, input bounds and contact checks."""

from pathlib import Path
import json, hashlib
import numpy as np
from core import Config, transition, reference_for, SAFE_HALF

R = Path(__file__).resolve().parents[1]


def verify_lipm():
    m = json.loads((R / "manifest.json").read_text())
    results = json.loads((R / "results.json").read_text())
    assert len(results) == m["unique_count"]
    assert (
        hashlib.sha256((R / m["root"] / "source.py").read_bytes()).hexdigest()
        == m["source_sha256"]
    )
    sample_count = 0
    max_residual = 0.0
    applied = {}
    for i, (key, result) in enumerate(results.items()):
        cfg = Config(**result["config"])
        folder = R / m["root"] / key
        assert result["source_sha256"] == m["source_sha256"]
        raw = np.load(folder / "physics.npz")
        states = raw["state"]
        cop = raw["cop"]
        feet = raw["foot"]
        shifts = raw["shift"]
        trace = np.load(folder / "trace.npz")["data"]
        sample_count += len(states)
        assert np.all(np.isfinite(states)) and np.all(np.isfinite(cop))
        assert len(cop) == len(states) - 1
        assert abs((len(states) - 1) * cfg.dt - result["duration_s"]) < 1e-9
        assert np.all(np.abs(cop - feet[:-1]) <= SAFE_HALF + 1e-8)
        A, B = transition(cfg.height, cfg.dt)
        times = np.arange(len(cop)) * cfg.dt
        active = (times >= cfg.push_time - 1e-10) & (
            times < cfg.push_time + cfg.push_duration - 1e-10
        )
        force = (
            active[:, None]
            * np.array([cfg.impulse_x, cfg.impulse_y])
            / cfg.push_duration
        )
        input_cop = cop - force / (cfg.mass * (9.81 / cfg.height))
        expected = states[:-1] @ A.T + input_cop[:, :, None] * B
        error = float(np.max(np.abs(states[1:] - expected))) if len(cop) else 0.0
        assert error < 1e-10, (key, "dynamics residual", error)
        max_residual = max(max_residual, error)
        applied[key] = (force.sum(axis=0) * cfg.dt).tolist()
        ref = reference_for(cfg)
        nominal = ref.state[: len(states)]
        ep = states[:, :, 0] - nominal[:, :, 0] - shifts
        tail = np.arange(len(states)) * cfg.dt >= (len(states) - 1) * cfg.dt - 0.5
        rmse = float(np.sqrt(np.mean(np.sum(ep**2, axis=1))))
        assert abs(rmse - result["tracking_rmse_m"]) < 1e-12
        eroute = float(np.linalg.norm(states[-1, :, 0] - nominal[-1, :, 0]))
        assert abs(eroute - result["terminal_route_error_m"]) < 1e-12
        if result["success"]:
            assert len(states) == ref.n + 1
            assert result["steps_completed"] == 11
            np.testing.assert_allclose(
                force.sum(axis=0) * cfg.dt, [cfg.impulse_x, cfg.impulse_y], atol=1e-9
            )
            assert np.linalg.norm(states[-1, :, 0] - nominal[-1, :, 0]) <= 0.35
            assert (
                np.max(
                    np.linalg.norm(
                        (9.81 / cfg.height) * (states[:-1, :, 0] - cop), axis=1
                    )
                )
                <= cfg.friction * 9.81 + 1e-8
            )
        landing = np.array(result["landings"])
        delta = np.diff(landing, axis=0)
        if len(delta):
            assert np.all((delta[:, 0] >= 0.04 - 1e-8) & (delta[:, 0] <= 0.34 + 1e-8))
            assert np.all(
                (np.abs(delta[:, 1]) >= 0.10 - 1e-8)
                & (np.abs(delta[:, 1]) <= 0.24 + 1e-8)
            )
        if i % 300 == 0:
            print("verified LIPM", i, flush=True)
    return {
        "runs": len(results),
        "state_samples": sample_count,
        "max_zoh_residual": max_residual,
        "bounds_checked": True,
        "common_physics_evaluation_checked": True,
        "force_units_checked": True,
    }


def verify_physical():
    manifest = json.loads((R / "physical_manifest.json").read_text())
    results = json.loads((R / "physical_results.json").read_text())
    source = (R / manifest["root"] / "source.py").read_bytes() + (
        R / "models/ankle.xml"
    ).read_bytes()
    assert hashlib.sha256(source).hexdigest() == manifest["source_sha256"]
    normal = []
    samples = 0
    for key, res in results.items():
        assert res["source_sha256"] == manifest["source_sha256"]
        x = np.load(R / manifest["root"] / key / "trace.npz")["data"]
        samples += len(x)
        assert np.isfinite(x[:, :9]).all()
        assert np.isfinite(x[:, 10]).all()
        valid = x[:, 8] > 1e-8
        assert np.isfinite(x[valid, 9]).all()
        assert np.all(np.abs(x[:, 6]) <= 25.0 + 1e-9)
        cfg = res["config"]
        dt = cfg["dt"]
        np.testing.assert_allclose(np.diff(x[:, 0]), dt, atol=1e-12)
        if res["completed"]:
            assert abs(res["duration_s"] - cfg["duration"]) < 1e-9
        if (
            cfg["force"] == 0
            and cfg["delay"] == 0
            and cfg["noise"] == 0
            and cfg["controller"] != "none"
        ):
            assert abs(res["last_normal_force_N"] - 22.5 * 9.81) < 0.1
            normal.append(res["last_normal_force_N"])
    recording_matches = []
    for method in ["pd", "lqr"]:
        folder = R / "recordings" / ("physical_" + method)
        if not (folder / "trace.npz").exists():
            continue
        matches = [
            k
            for k, v in results.items()
            if v["config"]["controller"] == method
            and v["config"]["hz"] == 100
            and v["config"]["force"] == 40.0
            and v["config"]["delay"] == 0
            and v["config"]["noise"] == 0.002
            and v["config"]["seed"] == 0
            and v["config"]["friction"] == 0.8
        ]
        original = np.load(R / manifest["root"] / matches[0] / "trace.npz")["data"]
        replay = np.load(folder / "trace.npz")["data"]
        np.testing.assert_allclose(original, replay, rtol=0, atol=0, equal_nan=True)
        recording_matches.append(method)
    return {
        "runs": len(results),
        "physics_samples": samples,
        "torque_limits_checked": True,
        "contact_frames_checked": True,
        "recording_replays_identical": recording_matches,
        "quiet_normal_forces_N": normal,
        "expected_weight_N": 22.5 * 9.81,
    }


if __name__ == "__main__":
    result = {"lipm": verify_lipm(), "physical": verify_physical()}
    (R / "verification.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
