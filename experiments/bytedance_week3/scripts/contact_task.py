"""Torque-actuated Panda probing a surface; measured contacts, no mocap welds."""

from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import mujoco

T = Path(__file__).resolve().parents[1]
Q0 = np.array([0.0, -0.5, 0.0, -2.1, 0.0, 1.6, 0.7854, 0.04, 0.04])


def model(stiffness=8000):
    tree = E.parse(T / "assets/panda/panda_collision.xml")
    r = tree.getroot()
    r.find("compiler").set("meshdir", str((T / "assets/panda/assets").resolve()))
    for g in r.findall(".//worldbody//geom"):
        g.set("contype", "0")
        g.set("conaffinity", "0")
    hand = r.find(".//body[@name='hand']")
    assert hand is not None
    E.SubElement(
        hand,
        "geom",
        name="probe",
        type="sphere",
        size=".015",
        pos="0 0 .16",
        rgba=".2 .5 .9 1",
        contype="1",
        conaffinity="1",
        mass="0",
        solref=f"-{stiffness} -{2*np.sqrt(stiffness)}",
        solimp=".95 .99 .001",
    )
    E.SubElement(hand, "site", name="tip", pos="0 0 .16", size=".006")
    world = r.find("worldbody")
    E.SubElement(
        world,
        "geom",
        name="surface",
        type="plane",
        size="1 1 .02",
        pos="0 0 0",
        rgba=".6 .65 .7 1",
        solref=f"-{stiffness} -{2*np.sqrt(stiffness)}",
        solimp=".95 .99 .001",
    )
    actuator = r.find("actuator")
    r.remove(actuator)
    actuator = E.SubElement(r, "actuator")
    for i in range(7):
        E.SubElement(
            actuator,
            "motor",
            name=f"torque{i+1}",
            joint=f"joint{i+1}",
            ctrlrange=("-87 87" if i < 4 else "-12 12"),
        )
    for j in ["finger_joint1", "finger_joint2"]:
        E.SubElement(actuator, "position", joint=j, kp="200", ctrlrange="0 .04")
    if r.find("keyframe") is not None:
        r.remove(r.find("keyframe"))
    m = mujoco.MjModel.from_xml_string(E.tostring(r).decode())
    d = mujoco.MjData(m)
    d.qpos[:] = Q0
    mujoco.mj_forward(m, d)
    site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tip")
    pos = d.site_xpos[site].copy()
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "surface")
    r.find("worldbody/geom[@name='surface']").set("pos", f"0 0 {pos[2]-.065}")
    m = mujoco.MjModel.from_xml_string(E.tostring(r).decode())
    d = mujoco.MjData(m)
    d.qpos[:] = Q0
    mujoco.mj_forward(m, d)
    return m, d, site, pos


def force(m, d):
    out = 0.0
    wrench = np.zeros(6)
    for i, c in enumerate(d.contact):
        names = [
            mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(g))
            for g in [c.geom1, c.geom2]
        ]
        if "probe" in names and "surface" in names:
            mujoco.mj_contactForce(m, d, i, wrench)
            out += abs(wrench[0])
    return out


def ik(m, shadow, site, q, xyz, rotation):
    shadow.qpos[:] = q
    for _ in range(12):
        mujoco.mj_forward(m, shadow)
        err = xyz - shadow.site_xpos[site]
        rot = shadow.site_xmat[site].reshape(3, 3)
        re = 0.5 * sum(np.cross(rot[:, i], rotation[:, i]) for i in range(3))
        error = np.r_[err, re]
        if np.linalg.norm(error) < 1e-5:
            break
        jp = np.zeros((3, m.nv))
        jr = np.zeros_like(jp)
        mujoco.mj_jacSite(m, shadow, jp, jr, site)
        J = np.vstack([jp[:, :7], jr[:, :7]])
        dq = J.T @ np.linalg.solve(J @ J.T + np.eye(6) * 0.001, error)
        shadow.qpos[:7] += np.clip(dq, -0.05, 0.05)
        shadow.qpos[:7] = np.clip(
            shadow.qpos[:7], m.jnt_range[:7, 0] + 0.01, m.jnt_range[:7, 1] - 0.01
        )
    return shadow.qpos[:7].copy()


def run(
    mode="admittance",
    stiffness=8000,
    rate=100,
    delay_ms=0,
    noise_std=0,
    seed=0,
    duration=8,
    video=None,
    K=0.0,
    approach_s=2.0,
    disable_contacts=False,
):
    m, d, site, initial = model(stiffness)
    shadow = mujoco.MjData(m)
    rotation = d.site_xmat[site].reshape(3, 3).copy()
    qref = Q0[:7].copy()
    offset = 0.0
    v = 0.0
    h = 1 / rate
    next_ctrl = 0.0
    history = []
    rows = []
    rng = np.random.default_rng(seed)
    M = 2.0
    B = 100.0
    caps = np.array([87] * 4 + [12] * 3)
    kp = np.array([700, 700, 700, 700, 400, 300, 200])
    kd = 2 * np.sqrt(kp) * 1.5
    seen = False
    first_contact = None
    if disable_contacts:
        m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    renderer = mujoco.Renderer(m, 480, 640) if video else None
    frames = []
    for step in range(round(duration / 0.001)):
        t = step * 0.001
        F = force(m, d)
        history.append(F)
        if F > 0.1 and first_contact is None:
            first_contact = t
        if t >= next_ctrl - 1e-9:
            u = np.clip((t - 0.5) / approach_s, 0, 1)
            blend = 10 * u**3 - 15 * u**4 + 6 * u**5
            base = initial.copy()
            base[2] -= 0.065 * blend
            measured = history[max(0, len(history) - 1 - round(delay_ms))] + rng.normal(
                0, noise_std
            )
            if F > 0.3:
                seen = True
            desired = (
                10.0 * min(1, max(0, (t - first_contact) / 0.5))
                if first_contact is not None
                else 0.0
            )
            if mode.startswith("admittance") and seen:
                # Positive surface reaction increases the commanded height.
                acc = (measured - desired - B * v - K * offset) / M
                v = np.clip(v + h * acc, -0.04, 0.04)
                offset = np.clip(offset + h * v, -0.025, 0.05)
            target = base.copy()
            target[2] += offset
            qref = ik(m, shadow, site, d.qpos.copy(), target, rotation)
            next_ctrl += h
        tau = kp * (qref - d.qpos[:7]) - kd * d.qvel[:7] + d.qfrc_bias[:7]
        sat = np.any(abs(tau) > caps)
        d.ctrl[:7] = np.clip(tau, -caps, caps)
        d.ctrl[7:] = 0.04
        mujoco.mj_step(m, d)
        rows.append(
            [
                t,
                F,
                desired,
                d.site_xpos[site, 2],
                base[2],
                offset,
                v,
                float(sat),
                *d.qpos[:7],
                *d.qvel[:7],
                *qref,
            ]
        )
        if renderer and step % 40 == 0:
            cam = mujoco.MjvCamera()
            cam.lookat[:] = [0.4, 0, 0.25]
            cam.distance = 1.5
            cam.azimuth = 135
            cam.elevation = -25
            option = mujoco.MjvOption()
            option.geomgroup[:] = 1
            renderer.update_scene(d, cam, scene_option=option)
            frames.append(renderer.render().copy())
    if renderer:
        renderer._mjr_context.free()
        renderer._gl_context.free()
    a = np.array(rows)
    steady = a[:, 0] >= 5
    contact = a[:, 1] > 0.1
    summary = dict(
        mode=mode,
        stiffness=stiffness,
        rate=rate,
        delay_ms=delay_ms,
        noise_std=noise_std,
        seed=seed,
        K=K,
        approach_s=approach_s,
        peak_force_N=float(a[:, 1].max()),
        steady_mean_force_N=float(a[steady, 1].mean()),
        steady_force_rmse_N=float(np.sqrt(np.mean((a[steady, 1] - 10) ** 2))),
        steady_force_std_N=float(a[steady, 1].std()),
        contact_fraction=float(contact.mean()),
        max_offset_m=float(np.max(abs(a[:, 5]))),
        torque_saturation_fraction=float(a[:, 7].mean()),
        first_contact_s=first_contact,
        finite=bool(np.isfinite(a).all()),
    )
    if video:
        import av

        container = av.open(str(video), "w")
        stream = container.add_stream("libx264", rate=25)
        stream.width = 640
        stream.height = 480
        stream.pix_fmt = "yuv420p"
        for frame in frames:
            for packet in stream.encode(
                av.VideoFrame.from_ndarray(frame, format="rgb24")
            ):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)
        container.close()
    return a, summary
