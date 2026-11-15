"""Recompute kinematics, contact-force signs, CoP and landing criteria from saved runs."""

import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from biped import BipedConfig, WBC
from feedback import contact_wrenches, resultant_cop

R = Path(__file__).resolve().parents[1]


def verify():
    manifest = json.loads((R / "manifest.json").read_text())
    root = R / manifest["root"]
    for name, digest in manifest["files"].items():
        assert hashlib.sha256((R / name).read_bytes()).hexdigest() == digest, name
        assert (root / "source" / name).read_bytes() == (R / name).read_bytes()
    errors = {
        name: 0.0
        for name in [
            "integration",
            "kinematics",
            "virtual_work",
            "contact_reconstruction",
            "cop_reconstruction",
            "mapping",
        ]
    }
    landings = 0
    for key, cfg in manifest["configs"].items():
        path = root / key
        r = json.loads((path / "result.json").read_text())
        x = np.load(path / "trace.npz")["data"]
        a = np.load(path / "audit.npz")
        assert r["config"] == cfg and r["source_sha256"] == manifest["source_sha256"]
        assert len(x) > 0 and len(a["time"]) > 0
        err = (
            np.max(np.abs(x[1:, 17:26] - x[:-1, 17:26] - cfg["dt"] * x[1:, 26:35]))
            if len(x) > 1
            else 0
        )
        errors["integration"] = max(errors["integration"], float(err))
        assert err < 1e-10, key
        assert np.all(
            np.abs(x[:, 10:16])
            <= np.array([80, 80, 30] * 2) * cfg["torque_scale"] + 1e-6
        )
        if cfg["torque_rate"] > 0:
            dt = np.diff(a["time"])
            mask = a["time"][:-1] >= 0.4
            delta = np.abs(np.diff(a["torque"], axis=0))
            assert np.all(delta[mask] <= cfg["torque_rate"] * dt[mask, None] + 1e-6), (
                key
            )
        m = mujoco.MjModel.from_xml_path(str(R / "models/biped.xml"))
        m.opt.timestep = cfg["dt"]
        m.geom_friction[:, 0] = cfg["friction"]
        d = mujoco.MjData(m)
        ctl = WBC(m, d, BipedConfig(**cfg))
        assert m.nv == 9 and m.nu == 6 and np.all(m.actuator_trnid[:, 0] >= 3)
        # Rebuild physical contact solver at a spread of recorded instants.
        for index in np.linspace(0, len(a["time"]) - 1, 12).astype(int):
            t = a["time"][index]
            d.qpos[:] = a["qpos"][index]
            d.qvel[:] = a["qvel"][index]
            d.ctrl[:] = a["contact_ctrl"][index]
            d.xfrc_applied[:] = 0
            if 2.7 + cfg["dt"] * 0.5 < t < 2.8 + cfg["dt"] * 0.5:
                d.xfrc_applied[ctl.root, 0] = cfg["push"]
            mujoco.mj_forward(m, d)
            w = contact_wrenches(m, d, ctl.sites)
            errors["contact_reconstruction"] = max(
                errors["contact_reconstruction"],
                float(np.max(np.abs(w - a["measured_wrench"][index]))),
            )
            J = ctl.jacobians(d)
            generalized = sum(J[i + 1].T @ w[i] for i in range(2))
            errors["virtual_work"] = max(
                errors["virtual_work"],
                float(np.max(np.abs(generalized - d.qfrc_constraint))),
            )
            cop = resultant_cop(d.site_xpos[ctl.sites][:, [0, 2]], w)
            if np.isfinite(cop):
                errors["cop_reconstruction"] = max(
                    errors["cop_reconstruction"], abs(cop - a["measured_cop"][index])
                )
        # Independently recompute all intended touchdown windows.
        checks = []
        for step in range(1, cfg["steps"] + 1):
            at = 0.5 + step * cfg["step_time"]
            side = step % 2
            good = (
                (x[:, 0] >= at - 0.04)
                & (x[:, 0] <= at + 0.10)
                & (np.abs(x[:, 4 + side * 2] - step * cfg["step_length"]) < 0.025)
                & (np.abs(x[:, 5 + side * 2]) < 0.008)
                & (x[:, 8 + side] > 10)
            )
            streak = 0
            ok = False
            for value in good:
                streak = streak + 1 if value else 0
                if streak * cfg["dt"] >= 0.015:
                    ok = True
                    break
            checks.append(ok)
        assert checks == r["verified_landings"]
        landings += sum(checks)
        if r["success"]:
            assert (
                all(checks) and abs(r["duration_s"] - cfg["duration"]) < cfg["dt"] * 1.1
            )
            assert r["progress_m"] >= cfg["steps"] * cfg["step_length"] - 0.10
        expected = a["sensed_com"][:, 0] - a["sensed_com"][:, 1] * a["target_acc"][
            :, 0
        ] / np.maximum(9.81 + a["target_acc"][:, 1], 1.0)
        errors["mapping"] = max(
            errors["mapping"], float(np.max(np.abs(expected - a["cop_raw"])))
        )
        # Physical foot positions from qpos, independent of runner success flag.
        for index in np.linspace(0, len(x) - 1, 8).astype(int):
            d.qpos[:] = x[index, 17:26]
            d.qvel[:] = x[index, 26:35]
            mujoco.mj_forward(m, d)
            actual = np.concatenate([d.site_xpos[s, [0, 2]] for s in ctl.sites])
            errors["kinematics"] = max(
                errors["kinematics"], float(np.max(np.abs(actual - x[index, 4:8])))
            )
    assert errors["mapping"] < 1e-12 and errors["kinematics"] < 1e-10
    # All contact forces are recomputed with independent solver warm starts.
    assert errors["contact_reconstruction"] < 0.05, errors
    assert errors["virtual_work"] < 1e-6, errors
    result = {
        "cases_verified": len(manifest["configs"]),
        "verified_landings_total": landings,
        "max_errors": errors,
        "source_unchanged": True,
        "scope": "planar model; no physical hardware; horizontal CoM delay only",
    }
    (R / "verification.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    verify()
