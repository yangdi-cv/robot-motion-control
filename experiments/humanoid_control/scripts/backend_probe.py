"""Compare backends on identical recorded-state QP matrices, without executing actions."""

import json
import time
import numpy as np
import mujoco
from model import R, initialize
from controller import WholeBody
from run import Config, Reference
from qp import solve_qp


def main():
    manifest = json.loads((R / "manifest.json").read_text())
    key = next(
        k
        for k, c in manifest["configs"].items()
        if c["scenario"] == "walk"
        and c["backend"] == "clarabel"
        and c["walk_steps"] == 4
        and c["step_distance"] == 0.06
        and c["hz"] == 100
        and not c["push_x"]
        and not c["push_y"]
    )
    cfg = Config(**manifest["configs"][key])
    trace = np.load(R / manifest["root"] / key / "trace.npz")
    model = mujoco.MjModel.from_xml_path(str(R / "models/scene.xml"))
    data = initialize(model)
    controller = WholeBody(model, data, cfg.hz, cfg.friction)
    reference = Reference(data, controller, cfg)
    rows = []
    for stamp in [0.83, 0.97, 1.65, 3.0, 5.76, 8.2, 10.0]:
        i = np.searchsorted(trace["time"], stamp)
        data.qpos[:] = trace["qpos"][i]
        data.qvel[:] = trace["qvel"][i]
        data.ctrl[:] = trace["contact_ctrl"][i]
        data.xfrc_applied[:] = 0
        data.xfrc_applied[controller.root] = trace["contact_external"][i]
        controller.solve(*reference.at(float(trace["time"][i])))
        H, g, A, l, u = controller.last_problem
        for backend in ["clarabel", "osqp"]:
            started = time.perf_counter()
            try:
                sol = solve_qp(H, g, A, l, u, backend)
                x = sol.x
                row = dict(
                    time=stamp,
                    backend=backend,
                    status=sol.info.status,
                    raw_constraint_violation=float(
                        max(np.max(l - A @ x), np.max(A @ x - u))
                    ),
                    objective=float(0.5 * x @ H @ x + g @ x),
                    iterations=sol.info.iter,
                )
            except RuntimeError as exc:
                row = dict(time=stamp, backend=backend, error=str(exc))
            row["seconds"] = time.perf_counter() - started
            rows.append(row)
    (R / "backend_probe.json").write_text(
        json.dumps(
            dict(case=key, scope="same QP inputs, no physical actions", rows=rows),
            indent=2,
        )
    )
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
