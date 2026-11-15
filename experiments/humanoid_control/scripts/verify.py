"""Rebuild full-body kinematics, contacts, quaternion integration and physical criteria."""

import hashlib
import json
import numpy as np
import mujoco
from model import R, initialize
from controller import WholeBody
from run import Config, wrenches
from metrics import evaluate


def main(manifest_name="manifest.json", output="verification.json"):
    manifest = json.loads((R / manifest_name).read_text())
    baseline = json.loads((R / "manifest.json").read_text())
    root = R / manifest["root"]
    for name, digest in baseline["files"].items():
        assert hashlib.sha256((R / name).read_bytes()).hexdigest() == digest, name
        assert (R / baseline["root"] / "source" / name).read_bytes() == (
            R / name
        ).read_bytes(), name
    source = json.loads((R / "research/model_source.json").read_text())
    for name, digest in source["sha256"].items():
        assert (
            hashlib.sha256(
                (R / "third_party/menagerie" / name).read_bytes()
            ).hexdigest()
            == digest
        )
    errors = dict(
        integration=0.0, feet=0.0, com=0.0, virtual_work=0.0, contact_rebuild=0.0
    )
    additional_contacts = []
    landings = 0
    single_support = 0.0
    cases = 0
    for key, cfg in manifest["configs"].items():
        path = root / key
        row = json.loads((path / "result.json").read_text())
        with np.load(path / "trace.npz") as saved:
            a = {name: saved[name] for name in saved.files}
        c = np.load(path / "controller.npz")
        config = Config(**cfg)
        assert (
            row["source_sha256"] == manifest["source_sha256"] and row["config"] == cfg
        )
        model = mujoco.MjModel.from_xml_path(str(R / "models/scene.xml"))
        model.opt.timestep = cfg["dt"]
        model.geom_friction[:, 0] = cfg["friction"]
        data = initialize(model)
        ctl = WholeBody(model, data, cfg["hz"], cfg["friction"], cfg["backend"])
        assert model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE and model.neq == 0
        assert (model.nv, model.nu) == (35, 29) and min(ctl.dofs) == 6
        assert np.all(np.abs(a["ctrl"]) <= ctl.torque_limits + 1e-8)
        assert float(c["normalized_constraint_violation"].max()) <= 1
        # Every integration interval, including the floating quaternion.
        predicted = a["qpos"][:-1].copy()
        for i in range(len(predicted)):
            mujoco.mj_integratePos(model, predicted[i], a["qvel"][i + 1], cfg["dt"])
        integration = (
            float(np.max(np.abs(predicted - a["qpos"][1:]))) if len(predicted) else 0
        )
        errors["integration"] = max(errors["integration"], integration)
        assert integration < 1e-10, key
        # All one-second chunks must concatenate exactly to the final trace.
        pieces = [np.load(p) for p in sorted((path / "chunks").glob("*.npz"))]
        for name in ["qpos", "qvel", "ctrl", "time"]:
            assert np.array_equal(np.concatenate([p[name] for p in pieces]), a[name]), (
                key,
                name,
            )
        for piece in pieces:
            piece.close()
        for index in np.linspace(0, len(a["time"]) - 1, 12).astype(int):
            data.qpos[:] = a["qpos"][index]
            data.qvel[:] = a["qvel"][index]
            data.ctrl[:] = a["contact_ctrl"][index]
            data.xfrc_applied[:] = 0
            data.xfrc_applied[ctl.root] = a["contact_external"][index]
            mujoco.mj_forward(model, data)
            errors["feet"] = max(
                errors["feet"],
                float(np.max(np.abs(data.site_xpos[ctl.sites] - a["feet"][index]))),
            )
            errors["com"] = max(
                errors["com"],
                float(np.max(np.abs(data.subtree_com[ctl.root] - a["com"][index]))),
            )
            measured = wrenches(model, data, ctl)
            errors["contact_rebuild"] = max(
                errors["contact_rebuild"],
                float(np.max(np.abs(measured - a["actual_wrench"][index]))),
            )
            J = ctl.jacobians(data)
            generalized = sum(J[i + 1].T @ measured[i] for i in [0, 1])
            # Compare the SAME foot-ground contacts. Strong disturbances can
            # create self-contact, which is real physics but absent from this QP.
            assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
            efc = np.zeros(data.nefc)
            floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
            foot_bodies = set(model.site_bodyid[ctl.sites])
            for contact_id, contact in enumerate(data.contact):
                pair = [contact.geom1, contact.geom2]
                other = pair[1] if pair[0] == floor else pair[0]
                if floor in pair and model.geom_bodyid[other] in foot_bodies:
                    address = contact.efc_address
                    if address >= 0:
                        efc[address : address + contact.dim] = data.efc_force[
                            address : address + contact.dim
                        ]
                else:
                    force = np.zeros(6)
                    mujoco.mj_contactForce(model, data, contact_id, force)
                    if abs(force[0]) > 1:
                        additional_contacts.append(
                            dict(
                                case=key,
                                time=float(a["time"][index]),
                                bodies=[
                                    mujoco.mj_id2name(
                                        model,
                                        mujoco.mjtObj.mjOBJ_BODY,
                                        model.geom_bodyid[g],
                                    )
                                    for g in pair
                                ],
                                normal_force=float(force[0]),
                            )
                        )
            contact_generalized = np.zeros(model.nv)
            mujoco.mj_mulJacTVec(model, data, contact_generalized, efc)
            errors["virtual_work"] = max(
                errors["virtual_work"],
                float(np.max(np.abs(generalized - contact_generalized))),
            )
            # Only joint motors apply actuation. Contact forces are solved by physics.
            assert np.max(np.abs(data.qfrc_actuator[:6])) < 1e-12
            np.testing.assert_allclose(
                data.qfrc_actuator[ctl.dofs], data.ctrl, atol=1e-8
            )
        derived = evaluate(a, config, float(model.body_mass.sum()))
        assert (
            derived["landings"] == row["landings"]
            and derived["criteria_passed"] == row["criteria_passed"]
        )
        if row["success"]:
            assert (
                derived["criteria_passed"]
                and abs(row["duration"] - cfg["duration"]) < cfg["dt"] * 1.1
            )
        landings += sum(derived["landings"])
        single_support += sum(derived["single_support_seconds"])
        cases += 1
    assert (
        errors["feet"] < 1e-10
        and errors["com"] < 1e-10
        and errors["virtual_work"] < 1e-6
    ), errors
    assert errors["contact_rebuild"] < 0.1, errors
    result = dict(
        cases_verified=cases,
        verified_load_confirmations=landings,
        measured_single_support_seconds=single_support,
        max_errors=errors,
        source_and_assets_unchanged=True,
        all_chunks_match=True,
        additional_contacts=additional_contacts,
    )
    (R / output).write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
