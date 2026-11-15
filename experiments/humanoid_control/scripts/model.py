"""Derived torque-driven G1; original pinned Menagerie files remain untouched."""

from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

R = Path(__file__).resolve().parents[1]
SOURCE = R / "third_party/menagerie/unitree_g1"


def build():
    destination = R / "models"
    destination.mkdir(exist_ok=True)
    root = ET.parse(SOURCE / "g1.xml").getroot()
    root.find("compiler").set("meshdir", "../third_party/menagerie/unitree_g1/assets")
    option = root.find("option")
    option.set("timestep", ".001")
    option.set("iterations", "100")
    option.set("tolerance", "1e-10")
    option.set("gravity", "0 0 -9.81")
    option.set("cone", "elliptic")
    actuators = root.find("actuator")
    for servo in list(actuators):
        bounds = servo.attrib["forcerange"]
        motor = ET.Element(
            "motor",
            {
                "name": servo.attrib["name"],
                "joint": servo.attrib["joint"],
                "gear": "1",
                "ctrllimited": "true",
                "ctrlrange": bounds,
                "forcelimited": "true",
                "forcerange": bounds,
            },
        )
        actuators.remove(servo)
        actuators.append(motor)
    for side in ["left", "right"]:
        foot = root.find(f".//body[@name='{side}_ankle_roll_link']")
        ET.SubElement(
            foot,
            "site",
            {
                "name": side + "_sole",
                "pos": ".035 0 -.035",
                "size": ".008",
                "rgba": "1 .2 .1 1",
            },
        )
    # Remove only servo control values from the initial keyframe, not its posture.
    for key in root.findall(".//key"):
        key.attrib.pop("ctrl", None)
    ET.ElementTree(root).write(destination / "g1_torque.xml")
    scene = ET.parse(SOURCE / "scene.xml").getroot()
    scene.find("include").set("file", "g1_torque.xml")
    visual = scene.find("visual")
    ET.SubElement(visual, "global", {}) if visual.find("global") is None else None
    visual.find("global").set("offwidth", "1280")
    visual.find("global").set("offheight", "960")
    ET.ElementTree(scene).write(destination / "scene.xml")
    return destination / "scene.xml"


def initialize(model):
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(model, data, key)
    data.ctrl[:] = 0
    for side in ["left", "right"]:
        for suffix, angle in [
            ("hip_pitch", -0.2),
            ("knee", 0.4),
            ("ankle_pitch", -0.2),
        ]:
            joint = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, side + "_" + suffix + "_joint"
            )
            data.qpos[model.jnt_qposadr[joint]] = angle
    mujoco.mj_forward(model, data)
    sites = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, s + "_sole")
        for s in ["left", "right"]
    ]
    data.qpos[2] -= np.mean(data.site_xpos[sites, 2]) + 0.0001
    mujoco.mj_forward(model, data)
    return data


if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path(str(build()))
    data = initialize(model)
    print(
        "nq/nv/nu",
        model.nq,
        model.nv,
        model.nu,
        "mass",
        model.body_mass.sum(),
        "base",
        data.qpos[:7],
    )
    print("com", data.subtree_com[1])
    for side in ["left", "right"]:
        site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, side + "_sole")
        print(side, data.site_xpos[site], data.site_xmat[site].reshape(3, 3))
    print(
        "contacts",
        data.ncon,
        "actuator gain",
        model.actuator_gainprm[:3, :3],
        "bias",
        model.actuator_biasprm[:3, :3],
    )
