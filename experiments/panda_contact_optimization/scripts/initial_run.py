"""Panda contact control in Isaac Sim 5.1; all forces come from PhysX."""

import argparse, json, csv
from pathlib import Path
import numpy as np

p = argparse.ArgumentParser()
p.add_argument("--out", type=Path, required=True)
p.add_argument("--mode", choices=["position", "admittance"], default="admittance")
p.add_argument("--duration", type=float, default=20)
p.add_argument("--speed", type=float, default=0.005)
p.add_argument("--mass", type=float, default=1)
p.add_argument("--damping", type=float, default=40)
p.add_argument("--record", action="store_true")
p.add_argument("--inner", choices=["compensated", "pid"], default="compensated")
p.add_argument("--execution-case", default=None)
p.add_argument("--offset-limit", type=float, default=0.08)
p.add_argument("--contact-gate", action="store_true")
p.add_argument("--force-ramp", type=float, default=0.0)
p.add_argument("--freeze-on-contact", action="store_true")
p.add_argument("--force-feedback", choices=["cycle", "last", "ema"], default="cycle")
p.add_argument("--filter-alpha", type=float, default=0.2)
p.add_argument("--control-dt", type=float, choices=[0.005, 0.01, 0.02], default=0.01)
p.add_argument(
    "--ik-seed", choices=["measured", "previous", "differential"], default="measured"
)
p.add_argument("--ik-tolerance", type=float, default=None)
p.add_argument("--ik-step-limit", type=float, default=0.05)
p.add_argument("--plane-height-shift", type=float, default=0.0)
p.add_argument("--gravity", action="store_true")
p.add_argument("--gravity-compensation", action="store_true")
p.add_argument("--geometry", choices=["probe", "gripper"], default="probe")
p.add_argument(
    "--stiffness",
    type=float,
    default=0.0,
    help="0 is rigid PhysX contact; positive compliant stiffness N/m",
)
p.add_argument("--contact-damping", type=float, default=100.0)
p.add_argument("--friction", type=float, default=0.5)
p.add_argument(
    "--approach-profile", choices=["constant", "slow", "brake"], default="constant"
)
p.add_argument("--approach-limit", type=float, default=0.055)
p.add_argument("--brake-distance", type=float, default=0.005)
p.add_argument("--contact-threshold", type=float, default=0.5)
p.add_argument("--force-velocity-limit", type=float, default=0.03)
p.add_argument("--solver-iterations", type=int, default=16)
a = p.parse_args()
if a.force_ramp > 0 and not a.contact_gate:
    p.error("--force-ramp requires --contact-gate")
if a.execution_case and a.control_dt != 0.01:
    p.error("execution-case requires --control-dt .01")
a.out.mkdir(parents=True, exist_ok=True)
(a.out / "COMPLETE").unlink(missing_ok=True)
from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 960, "height": 720})
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, RigidPrim
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdGeom, UsdPhysics, UsdLux, Gf, PhysxSchema, UsdShade
import omni.usd
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.robot_motion.motion_generation")
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
    interface_config_loader,
)

w = World(stage_units_in_meters=1.0, physics_dt=0.001, rendering_dt=0.02)
asset = get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
add_reference_to_stage(asset, "/World/Panda")
r = w.scene.add(SingleArticulation("/World/Panda", name="panda"))
w.reset()
r.enable_gravity() if a.gravity else r.disable_gravity()
q0 = np.array([0, -0.785398, 0, -2.356194, 0, 1.570796, 0.785398, 0.04, 0.04])
r.set_joint_positions(q0)
r.set_joint_velocities(np.zeros(9))
from isaacsim.core.utils.types import ArticulationAction

r.get_articulation_controller().apply_action(ArticulationAction(joint_positions=q0))
w.step(render=False)
stage = omni.usd.get_context().get_stage()
light = UsdLux.DomeLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(700)
for prim in list(stage.Traverse()):
    if str(prim.GetPath()).startswith("/World/Panda") and prim.IsInstance():
        prim.SetInstanceable(False)
for prim in stage.Traverse():
    if str(prim.GetPath()).startswith("/World/Panda") and prim.HasAPI(
        UsdPhysics.CollisionAPI
    ):
        UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(
            a.geometry == "gripper"
            and any(
                part in str(prim.GetPath())
                for part in [
                    "/panda_hand/",
                    "/panda_leftfinger/",
                    "/panda_rightfinger/",
                ]
            )
        )
probe = UsdGeom.Sphere.Define(stage, "/World/Panda/panda_hand/contact_probe")
probe.CreateRadiusAttr(0.025)
probe.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.25, 0.65)])
UsdPhysics.CollisionAPI.Apply(probe.GetPrim()).CreateCollisionEnabledAttr(
    a.geometry == "probe"
)
if a.geometry == "gripper":
    UsdGeom.Imageable(probe).MakeInvisible()
for prim in stage.Traverse():
    if a.geometry == "probe" and any(
        (
            part in str(prim.GetPath())
            for part in [
                "/panda_hand/geometry",
                "/panda_leftfinger/geometry",
                "/panda_rightfinger/geometry",
            ]
        )
    ):
        UsdGeom.Imageable(prim).MakeInvisible()
from isaacsim.core.prims import SingleXFormPrim

hand = SingleXFormPrim("/World/Panda/panda_hand")
pos = hand.get_world_pose()[0]
# Determine actual active collision bottom, including fingers, rather than assuming hand origin.
cache = UsdGeom.BBoxCache(0, ["default", "render", "proxy"])
bottoms = []
for prim in stage.Traverse():
    if (
        str(prim.GetPath()).startswith("/World/Panda")
        and prim.HasAPI(UsdPhysics.CollisionAPI)
        and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
    ):
        box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
        if not box.IsEmpty():
            bottoms.append(float(box.GetMin()[2]))
contact_bottom = min(bottoms)
plane = w.scene.add(
    FixedCuboid(
        "/World/Plane",
        name="plane",
        position=np.array(
            [
                pos[0],
                pos[1],
                contact_bottom
                - (0.3 if a.execution_case else 0.040)
                - 0.01
                + a.plane_height_shift,
            ]
        ),
        scale=np.array([0.22, 0.22, 0.02]),
        color=np.array([0.3, 0.5, 0.7]),
    )
)
UsdGeom.Cube(plane.prim).CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
material = UsdShade.Material.Define(stage, "/World/ContactMaterial")
mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
mat.CreateStaticFrictionAttr(a.friction)
mat.CreateDynamicFrictionAttr(a.friction)
mat.CreateRestitutionAttr(0.0)
pm = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
if a.stiffness > 0:
    pm.CreateCompliantContactStiffnessAttr(a.stiffness)
    pm.CreateCompliantContactDampingAttr(a.contact_damping)
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        pa = PhysxSchema.PhysxArticulationAPI.Apply(prim)
        pa.CreateSolverPositionIterationCountAttr(a.solver_iterations)
        pa.CreateSolverVelocityIterationCountAttr(4)
view = RigidPrim(
    "/World/Panda/panda_(hand|leftfinger|rightfinger)",
    name="hand",
    track_contact_forces=True,
    contact_filter_prim_paths_expr=["/World/Plane"],
)
w.scene.add(view)
w.reset()
r.enable_gravity() if a.gravity else r.disable_gravity()
r.set_joint_positions(q0)
r.set_joint_velocities(np.zeros(9))
ctrl = r.get_articulation_controller()
ctrl.set_gains(kps=np.zeros(9), kds=np.zeros(9))
w.step(render=False)
stage.GetRootLayer().Export(str(a.out / "scene.usda"))
print(
    "BOUNDS",
    [
        (
            str(x.GetPath()),
            str(
                UsdGeom.BBoxCache(0, ["default"])
                .ComputeWorldBound(x)
                .ComputeAlignedBox()
            ),
        )
        for x in [probe.GetPrim(), plane.prim]
    ],
    flush=True,
)
lula = LulaKinematicsSolver(
    **interface_config_loader.load_supported_lula_kinematics_solver_config("Franka")
)
ik = ArticulationKinematicsSolver(r, lula, "panda_hand")
if a.ik_tolerance is not None:
    lula.set_default_position_tolerance(a.ik_tolerance)
rv = r._articulation_view
idx = rv.body_names.index("panda_hand") - 1
meta = {
    "plane_height_shift_m": a.plane_height_shift,
    "differential_damping": 0.01,
    "differential_joint_step_limit_rad": 0.003,
    "ik_step_limit_rad": a.ik_step_limit,
    "ik_seed": a.ik_seed,
    "ik_position_tolerance_m": lula.get_default_position_tolerance(),
    "ik_orientation_tolerance_rad": lula.get_default_orientation_tolerance(),
    "asset": asset,
    "bodies": rv.body_names,
    "hand_initial": pos.tolist(),
    "mode": a.mode,
    "physics_dt": 0.001,
    "control_dt": a.control_dt,
    "freeze_on_contact": a.freeze_on_contact,
    "force_feedback": a.force_feedback,
    "filter_alpha": a.filter_alpha,
    "M": a.mass,
    "B": a.damping,
    "approach_speed_m_s": a.speed,
    "offset_limit_m": a.offset_limit,
    "velocity_limit_m_s": 0.03,
    "contact_gate": a.contact_gate,
    "force_ramp_s": a.force_ramp,
    "controller": "mass-matrix compensated PID; identical inner loop in both modes",
    "force_target_N": 5,
    "gravity": "disabled equally in both comparisons",
    "probe_radius_m": 0.025,
    "collision_scope": "probe only; original Panda collisions disabled for isolated contact experiment",
}
(a.out / "metadata.json").write_text(json.dumps(meta, indent=2))
print("SCENE_READY", a.mode, a.inner, flush=True)
qref = q0.copy()
integ = np.zeros(9)
kp = np.array([100] * 9)
kd = np.array([20] * 9)
ki = np.array([5] * 7 + [0, 0])
if a.inner == "pid":
    kp = np.array([200, 200, 150, 150, 10, 8, 1, 20, 20], dtype=float)
    kd = np.array([10, 10, 8, 8, 0.3, 0.2, 0.02, 0.1, 0.1], dtype=float)
    ki = np.array([5, 5, 5, 5, 0.2, 0.2, 0.05, 0, 0])
meta["optimization"] = vars(a).copy()
meta["optimization"]["out"] = str(a.out)
meta["gravity"] = "enabled" if a.gravity else "disabled"
meta["collision_scope"] = a.geometry
meta["contact_bottom_initial_m"] = contact_bottom
meta["inner"] = a.inner
meta["controller"] = (
    "joint PID without dynamic compensation"
    if a.inner == "pid"
    else "mass-matrix compensated PID"
)
meta["gains"] = {"kp": kp.tolist(), "kd": kd.tolist(), "ki": ki.tolist()}
(a.out / "metadata.json").write_text(json.dumps(meta, indent=2))
torque = np.zeros(9)
offset = 0.0
vel = 0.0
force = 0.0
target = pos.copy()
f = (a.out / "trace.csv").open("w")
wr = csv.writer(f)
wr.writerow(
    ["t", "fz", "z", "z_ref", "offset"]
    + [f"q{i}" for i in range(7)]
    + [f"ref{i}" for i in range(7)]
    + ["x", "y", "x_ref", "y_ref", "control_force"]
)
camera = None
if a.record:
    from isaacsim.sensors.camera import Camera
    from isaacsim.core.utils.viewports import set_camera_view

    camera = Camera("/World/Camera", resolution=(960, 720))
    camera.initialize()
    camera.set_focal_length(1.8)
    set_camera_view(
        eye=[1.2, 1.2, 1.05], target=[0.25, 0, 0.45], camera_prim_path="/World/Camera"
    )
    (a.out / "frames").mkdir(exist_ok=True)
for _ in range(5):
    w.render()
stream = None
stream_log = None
if a.execution_case:
    from action_stream import ActionStream, CASES

    if a.execution_case not in CASES:
        raise ValueError(a.execution_case)
    stream = ActionStream(a.execution_case, q0)
    stream_log = (a.out / "execution.csv").open("w")
    stream_writer = csv.writer(stream_log)
    stream_writer.writerow(
        ["t", "age_s", "timeout"]
        + [f"oracle{i}" for i in range(7)]
        + [f"command{i}" for i in range(7)]
        + [f"actual{i}" for i in range(7)]
        + [f"effort{i}" for i in range(7)]
    )
    meta["execution_case"] = a.execution_case
    meta["source"] = "synthetic quintic action chunks; not VLA inference"
    meta["packet"] = {
        "generation_period_s": 0.2,
        "delay_s": 0.12,
        "horizon_steps": 40,
        "action_dt_s": 0.01,
        "timeout_s": 0.5,
        "outage_s": [2, 3],
        "amplitude_scale": 3,
    }
    (a.out / "metadata.json").write_text(json.dumps(meta, indent=2))
force_window = []
first_contact_time = None
frozen_z = None
nominal_descent = 0.0
filtered_force = 0.0
control_stride = round(a.control_dt / 0.001)
rv_dummy_hand_index = view.prim_paths.index("/World/Panda/panda_hand")
start_sim_time = w.current_time
ff = (a.out / "physics_force.csv").open("w")
fw = csv.writer(ff)
fw.writerow(["t", "fz_N"])
try:
    for step in range(int(a.duration / 0.001)):
        t = w.current_time - start_sim_time
        if abs(t - step * 0.001) > 1e-05:
            raise RuntimeError(f"Unexpected physics time: {t}, step {step}")
        if step % control_stride == 0:
            q = r.get_joint_positions()
            dq = r.get_joint_velocities()
            actual = view.get_world_poses()[0][rv_dummy_hand_index]
            cycle_mean = float(np.mean(force_window)) if force_window else 0.0
            filtered_force += a.filter_alpha * (cycle_mean - filtered_force)
            force = (
                cycle_mean
                if a.force_feedback == "cycle"
                else (
                    filtered_force
                    if a.force_feedback == "ema"
                    else (float(force_window[-1]) if force_window else 0.0)
                )
            )
            force_window = []
            if step < 5000:
                u = t / 5
                s = 10 * u**3 - 15 * u**4 + 6 * u**5
                qref = q0.copy()
                qref[:3] += np.array([0.06, 0.04, -0.04]) * s
                target = actual.copy()
            else:
                if step == 5000:
                    anchor = actual.copy()
                    anchor_quat = view.get_world_poses()[1][rv_dummy_hand_index]
                target = anchor.copy()
                approach_speed = a.speed
                if a.approach_profile == "slow":
                    approach_speed = 0.001
                elif a.approach_profile == "brake":
                    # Nominal gap estimate; actual contact still determined only by measured force.
                    surface = float(plane.get_world_pose()[0][2]) + 0.01
                    lowest = actual[2] + contact_bottom - pos[2]
                    gap = lowest - surface
                    approach_speed = min(
                        a.speed,
                        max(0.0003, a.speed * np.clip(gap / a.brake_distance, 0, 1)),
                    )
                nominal_descent = min(
                    nominal_descent + approach_speed * a.control_dt, a.approach_limit
                )
                target[2] -= nominal_descent
                if first_contact_time is None and cycle_mean > a.contact_threshold:
                    first_contact_time = t
                    frozen_z = actual[2] if a.freeze_on_contact else target[2]
                if a.freeze_on_contact and frozen_z is not None:
                    target[2] = frozen_z
                if a.mode == "admittance" and (
                    not a.contact_gate or first_contact_time is not None
                ):
                    u = (
                        1.0
                        if a.force_ramp <= 0
                        else np.clip((t - first_contact_time) / a.force_ramp, 0, 1)
                    )
                    force_target = 5.0 * u * u * (3.0 - 2.0 * u)
                    vel = np.clip(
                        vel
                        + a.control_dt
                        * (force_target - force - a.damping * vel)
                        / a.mass,
                        -a.force_velocity_limit,
                        a.force_velocity_limit,
                    )
                    offset = np.clip(
                        offset + a.control_dt * vel, -a.offset_limit, a.offset_limit
                    )
                    target[2] -= offset
                if a.ik_seed == "differential":
                    from isaacsim.core.utils.rotations import quat_to_rot_matrix

                    reference_position, reference_rotation = (
                        lula.compute_forward_kinematics("panda_hand", qref[:7])
                    )
                    rotation_error = (
                        quat_to_rot_matrix(anchor_quat) @ reference_rotation.T
                    )
                    angular_error = 0.5 * np.array(
                        [
                            rotation_error[2, 1] - rotation_error[1, 2],
                            rotation_error[0, 2] - rotation_error[2, 0],
                            rotation_error[1, 0] - rotation_error[0, 1],
                        ]
                    )
                    error = np.concatenate([target - reference_position, angular_error])
                    jac = rv.get_jacobians()[0, idx, :6, :7]
                    increment = jac.T @ np.linalg.solve(
                        jac @ jac.T + 0.01**2 * np.eye(6), error
                    )
                    lower, upper = lula.get_cspace_position_limits()
                    solution = np.clip(
                        qref[:7] + np.clip(increment, -0.003, 0.003), lower, upper
                    )
                    success = True
                elif a.ik_seed == "previous":
                    solution, success = lula.compute_inverse_kinematics(
                        "panda_hand", target, anchor_quat, warm_start=qref[:7]
                    )
                else:
                    action, success = ik.compute_inverse_kinematics(
                        target_position=target, target_orientation=anchor_quat
                    )
                    solution = action.joint_positions
                if not success:
                    raise RuntimeError("IK failed")
                if not np.all(np.isfinite(solution)):
                    (a.out / "FAILED.json").write_text(
                        json.dumps({"stage": "IK_nonfinite", "t": t})
                    )
                    raise RuntimeError("Nonfinite IK output")
                jump = float(np.max(np.abs(solution - qref[:7])))
                if jump > a.ik_step_limit:
                    (a.out / "FAILED.json").write_text(
                        json.dumps(
                            {
                                "stage": "IK_jump_guard",
                                "t": t,
                                "max_jump_rad": jump,
                                "limit_rad": a.ik_step_limit,
                            }
                        )
                    )
                    raise RuntimeError(
                        f"IK reference jump {jump} rad exceeds {a.ik_step_limit}"
                    )
                qref = q0.copy()
                qref[:7] = solution
            if stream is not None:
                qref, oracle, age, timed_out = stream.update(t, q)
            err = qref - q
            integ = np.clip(integ + err * a.control_dt, -0.2, 0.2)
            control = kp * err - kd * dq + ki * integ
            effort = (
                control
                if a.inner == "pid"
                else rv.get_mass_matrices()[0] @ control
                + rv.get_coriolis_and_centrifugal_forces()[0]
            )
            if a.gravity_compensation:
                effort = effort + rv.get_generalized_gravity_forces()[0]
            torque = np.clip(
                effort,
                -np.array([87] * 4 + [12] * 3 + [5, 5]),
                np.array([87] * 4 + [12] * 3 + [5, 5]),
            )
            if stream_log is not None:
                stream_writer.writerow(
                    [
                        t,
                        age,
                        int(timed_out),
                        *oracle[:7],
                        *qref[:7],
                        *q[:7],
                        *torque[:7],
                    ]
                )
            wr.writerow(
                [
                    t,
                    cycle_mean,
                    actual[2],
                    target[2],
                    offset,
                    *q[:7],
                    *qref[:7],
                    actual[0],
                    actual[1],
                    target[0],
                    target[1],
                    force,
                ]
            )
        r.set_joint_efforts(torque)
        w.step(render=False)
        if camera is not None and step % 100 == 0:
            w.render()
        if camera is not None and step % 100 == 0:
            from PIL import Image

            rgba = camera.get_rgba()
            if rgba is not None and rgba.size:
                Image.fromarray(rgba.astype(np.uint8)).save(
                    a.out / "frames" / f"{step // 100:05d}.png"
                )
        measured = float(np.sum(view.get_contact_force_matrix(dt=0.001)[:, 0, 2]))
        force_window.append(measured)
        fw.writerow([(step + 1) * 0.001, measured])
    if stream_log is not None:
        stream_log.close()
        (a.out / "stream_summary.json").write_text(
            json.dumps(
                {
                    "generated": stream.generated,
                    "received": stream.received,
                    "timeout_events": stream.timeout_events,
                }
            )
        )
    f.close()
    ff.flush()
    (a.out / "time_audit.json").write_text(
        json.dumps(
            {
                "initial_sim_time": start_sim_time,
                "final_sim_time": w.current_time,
                "expected_duration": a.duration,
                "measured_duration": w.current_time - start_sim_time,
            }
        )
    )
    (a.out / "COMPLETE").write_text("Simulation finished normally\n")
    print("DONE", flush=True)
finally:
    f.close()
    ff.close()
    if stream_log is not None:
        stream_log.close()
    app.close()
