"""Isaac Sim 5.1 standalone Panda scene with a genuine ROS2 OmniGraph bridge."""

import argparse
import json
import time
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--config", type=Path, required=True)
parser.add_argument("--out", type=Path, required=True)
parser.add_argument("--gui", action="store_true")
args = parser.parse_args()
config = json.loads(args.config.read_text())
args.out.mkdir(parents=True, exist_ok=True)
from isaacsim import SimulationApp

app = SimulationApp(
    {
        "headless": not args.gui,
        "width": 1280,
        "height": 720,
        "renderer": "RaytracedLighting",
    }
)
import numpy as np
import omni.graph.core as og
import omni.usd
from pxr import UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

enable_extension("isaacsim.ros2.bridge")
for _ in range(15):
    app.update()
world = World(
    stage_units_in_meters=1.0,
    physics_dt=config["physics_dt_s"],
    rendering_dt=config["physics_dt_s"],
)
world.scene.add_default_ground_plane()
asset = get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
add_reference_to_stage(asset, "/World/Panda")
robot = world.scene.add(SingleArticulation(prim_path="/World/Panda", name="panda"))
world.reset()
expected = config["joint_names"]
actual = robot.dof_names
if set(expected) != set(actual):
    raise ValueError(f"Unexpected joint names: {actual}")
positions = np.array(
    [config["initial_positions_rad"][expected.index(name)] for name in actual]
)
robot.set_joint_positions(positions)
robot.set_joint_velocities(np.zeros(len(actual)))
set_camera_view(
    eye=[1.6, 1.6, 1.2], target=[0.0, 0.0, 0.45], camera_prim_path="/OmniverseKit_Persp"
)
# Original graph topology based on NVIDIA's 5.1 joint-control tutorial; task-specific isolated topics.
nodes = [
    ("Pulse", "omni.graph.action.OnImpulseEvent"),
    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
    ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("State", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ("Command", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
    ("Drive", "isaacsim.core.nodes.IsaacArticulationController"),
    ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
]
connections = []
for name in ("State", "Command", "Drive", "Clock"):
    connections.append(("Pulse.outputs:execOut", name + ".inputs:execIn"))
for name in ("State", "Command", "Clock"):
    connections.append(("Context.outputs:context", name + ".inputs:context"))
connections += [
    ("Time.outputs:simulationTime", "State.inputs:timeStamp"),
    ("Time.outputs:simulationTime", "Clock.inputs:timeStamp"),
    ("Command.outputs:jointNames", "Drive.inputs:jointNames"),
    ("Command.outputs:positionCommand", "Drive.inputs:positionCommand"),
    ("Command.outputs:velocityCommand", "Drive.inputs:velocityCommand"),
]
values = [
    ("Context.inputs:domain_id", config["ros_domain_id"]),
    ("Context.inputs:useDomainIDEnvVar", False),
    ("State.inputs:targetPrim", "/World/Panda"),
    ("State.inputs:topicName", config["state_topic"]),
    ("Command.inputs:topicName", config["command_topic"]),
    ("Drive.inputs:robotPath", "/World/Panda"),
    ("Clock.inputs:topicName", config["clock_topic"]),
]
og.Controller.edit(
    {"graph_path": "/PandaTaskBridge", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: nodes,
        og.Controller.Keys.CONNECT: connections,
        og.Controller.Keys.SET_VALUES: values,
    },
)
from isaacsim.core.utils.types import ArticulationAction

controller = robot.get_articulation_controller()
default_kp, default_kd = controller.get_gains()
(args.out / "default_gains.json").write_text(
    json.dumps(
        {"kp": default_kp.tolist(), "kd": default_kd.tolist(), "dof_names": actual},
        indent=2,
    )
)
active_phase = -1
stage = omni.usd.get_context().get_stage()
joints = []
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.RevoluteJoint):
        joint = UsdPhysics.RevoluteJoint(prim)
        joints.append(
            {
                "name": prim.GetName(),
                "path": str(prim.GetPath()),
                "lower_deg": joint.GetLowerLimitAttr().Get(),
                "upper_deg": joint.GetUpperLimitAttr().Get(),
            }
        )
metadata = {
    "asset": asset,
    "dof_names": actual,
    "initial_positions_rad": positions.tolist(),
    "joints": joints,
    "end_effector_path": "/World/Panda/panda_hand",
    "physics_dt_s": config["physics_dt_s"],
    "gui": args.gui,
    "graph_path": "/PandaTaskBridge",
    "ros_domain_id": config["ros_domain_id"],
}
(args.out / "scene_metadata.json").write_text(json.dumps(metadata, indent=2))
stage.GetRootLayer().Export(str(args.out / "panda_scene.usda"))
start = time.monotonic()
next_tick = start
last_done = None
try:
    with (args.out / "simulation_timing.csv").open("w") as timing:
        timing.write("step,sim_time_s,host_monotonic_ns,step_wall_s\n")
        step = 0
        while app.is_running():
            tick = time.monotonic()
            schedule = args.out / "schedule.json"
            if schedule.exists():
                schedule_start = json.loads(schedule.read_text())["start_sim_time"]
                phase = int(
                    max(0, world.current_time - schedule_start)
                    / config["phase_duration_s"]
                )
                if phase < len(config["stages"]) and phase != active_phase:
                    condition = config["stages"][phase]
                    kp = default_kp.copy()
                    kd = default_kd.copy()
                    kp[:7] *= condition["kp_scale"]
                    kd[:7] *= condition["kd_scale"]
                    controller.set_gains(kps=kp, kds=kd)
                    base_start = np.array(config["initial_positions_rad"])
                    base_end = base_start + (
                        np.array(config["target_positions_rad"]) - base_start
                    ) * condition.get("amplitude_scale", 1.0)
                    init = (
                        base_start if condition["direction"] == "forward" else base_end
                    )
                    init = np.array([init[expected.index(name)] for name in actual])
                    robot.set_joint_positions(init)
                    robot.set_joint_velocities(np.zeros(len(actual)))
                    controller.apply_action(
                        ArticulationAction(
                            joint_positions=init, joint_velocities=np.zeros(len(actual))
                        )
                    )
                    with (args.out / "gain_changes.jsonl").open("a") as f:
                        f.write(
                            json.dumps(
                                {
                                    "phase": phase,
                                    "sim_time": world.current_time,
                                    "kp": kp.tolist(),
                                    "kd": kd.tolist(),
                                    **condition,
                                }
                            )
                            + "\n"
                        )
                    active_phase = phase
            world.step(render=config.get("render_simulation", True))
            og.Controller.set(
                og.Controller.attribute("/PandaTaskBridge/Pulse.state:enableImpulse"),
                True,
            )
            og.Controller.evaluate_sync("/PandaTaskBridge")
            timing.write(
                f"{step},{world.current_time},{time.monotonic_ns()},{time.monotonic()-tick}\n"
            )
            if step == 100:
                (args.out / "sim_ready").touch()
                print("SIM_READY", flush=True)
            if (args.out / "controller_done.json").exists():
                if last_done is None:
                    last_done = time.monotonic()
                if time.monotonic() - last_done > 2:
                    break
            if time.monotonic() - start > 700:
                raise TimeoutError("Controller completion not observed")
            step += 1
            if config.get("absolute_pacing", False):
                next_tick += config["physics_dt_s"]
                remaining = next_tick - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                elif -remaining > 5 * config["physics_dt_s"]:
                    next_tick = time.monotonic()
            else:
                time.sleep(max(0, config["physics_dt_s"] - (time.monotonic() - tick)))
finally:
    app.close()
