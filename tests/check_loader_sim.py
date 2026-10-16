#!/usr/bin/env python3
"""Isolated DART smoke test; strips cameras only from a temporary world copy."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import uuid
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--loaded", action="store_true",
                        help="Place bale_1 across the fork for a supported-load test")
    parser.add_argument("--grip", action="store_true", help="Test contact-only clamping, lift, transport and release")
    args = parser.parse_args()
    if args.loaded and args.grip: parser.error("Choose --loaded or --grip")
    root = Path(__file__).resolve().parents[1]
    # Use a unique partition so the test cannot command the user's running tractor.
    os.environ["GZ_PARTITION"] = "loader_test_" + uuid.uuid4().hex
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(root / "build")
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    tree = ET.parse(root / "worlds/simple_tractor.sdf")
    world = tree.getroot().find("world")
    for plugin in list(world.findall("plugin")):
        if plugin.get("name") == "gz::sim::systems::Sensors":
            world.remove(plugin)
    for link in world.findall("model/link"):
        for sensor in list(link.findall("sensor")):
            link.remove(sensor)
    if args.loaded:
        # Fork is an explicit test fixture, not the production world's default.
        model = world.find("model[@name='simple_tractor']")
        carrier = model.find("link[@name='attachment_carrier']")
        index = list(model).index(carrier)
        model.remove(carrier)
        for name in ["left", "right"]:
            for tag, attr, value in [("link", "name", f"grip_{name}"), ("joint", "name", f"grip_{name}_joint")]:
                element = model.find(f"{tag}[@{attr}='{value}']")
                if element is not None: model.remove(element)
            hydraulics = model.find("plugin[@name='bale_sim::LoaderHydraulics']")
            element = hydraulics.find(f"grip_{name}")
            if element is not None: hydraulics.remove(element)
            publisher = model.find("plugin[@name='gz::sim::systems::JointStatePublisher']")
            for joint in list(publisher.findall('joint_name')):
                if joint.text == f'grip_{name}_joint': publisher.remove(joint)
        fixture = ET.parse(root / "tests/fixtures/square_bale_fork.sdf")
        fork = fixture.getroot().find("model/link")
        model.insert(index, fork)
        fx, _, fz, *_ = map(float, fork.findtext("pose").split())
        # Across the fork: long bale axis is Y, bottom rests on the tine tops.
        x = fx + 0.69
        z = fz + 0.04 + 0.45
        world.find("model[@name='bale_1']/pose").text = f"{x} 0 {z} 0 0 {math.pi/2}"

    if args.grip:
        carrier = world.find("model[@name='simple_tractor']/link[@name='attachment_carrier']")
        fx = float(carrier.findtext("pose").split()[0])
        world.find("model[@name='bale_1']/pose").text = f"{fx+1.5} 0 0.45 0 0 0"
    node = Node()
    lock = threading.Lock()
    states, positions, samples = {}, {}, []

    def receive(which, msg):
        with lock:
            states[which] = list(msg.data)
            samples.append((which, list(msg.data)))

    def receive_poses(msg):
        with lock:
            for pose in msg.pose:
                if pose.name in {"attachment_carrier", "bale_1"}:
                    positions[pose.name] = (pose.position.x, pose.position.z)

    channels = ["lift", "tilt"] + (["grip_left", "grip_right"] if args.grip else [])
    for which in channels:
        node.subscribe(Double_V, f"/tractor/loader/{which}/state",
                       lambda msg, name=which: receive(name, msg))
    node.subscribe(Pose_V, "/world/tractor_world/pose/info", receive_poses)
    lift = node.advertise("/tractor/loader/lift/valve", Double)
    tilt = node.advertise("/tractor/loader/tilt/valve", Double)

    grip = node.advertise("/tractor/loader/grip/valve", Double)
    from gz.msgs10.twist_pb2 import Twist
    drive = node.advertise("/model/simple_tractor/cmd_vel", Twist)

    def snapshot():
        with lock:
            return {k: list(v) for k, v in states.items()}, dict(positions)

    def phase(duration, up=0.0, roll=0.0, send=True, squeeze=0.0, drive_speed=0.0):
        initial, _ = snapshot()
        sim_start = initial["lift"][6]
        deadline = time.monotonic() + max(15, duration * 5)
        while time.monotonic() < deadline:
            if send:
                lift.publish(Double(data=up))
                tilt.publish(Double(data=roll))
                grip.publish(Double(data=squeeze))
                msg = Twist(); msg.linear.x = drive_speed
                drive.publish(msg)
            time.sleep(0.05)
            state, _ = snapshot()
            if state["lift"][6] - sim_start >= duration:
                return snapshot()
        raise AssertionError("Simulation stopped progressing")

    with tempfile.TemporaryDirectory(prefix="bale-loader-test-") as tmp:
        path = Path(tmp)
        tree.write(path / "world.sdf", encoding="unicode")
        with (path / "server.log").open("w") as log:
            server = subprocess.Popen(
                ["gz", "sim", "-s", "-r", "--physics-engine",
                 "gz-physics-dartsim-plugin", str(path / "world.sdf")],
                stdout=log, stderr=log, start_new_session=True)
            try:
                deadline = time.monotonic() + 15
                while len(snapshot()[0]) < len(channels) and time.monotonic() < deadline:
                    time.sleep(0.1)
                assert len(snapshot()[0]) == len(channels), "Hydraulic plugin produced no telemetry"
                start, pos0 = phase(2)
                if args.grip:
                    closed, closed_pos = phase(5, squeeze=1)
                    print("Clamped:", closed, closed_pos, flush=True)
                    for jaw in ["grip_left", "grip_right"]:
                        assert .28 < closed[jaw][0] < .43, "Jaws missed the bale"
                        assert closed[jaw][3] > 6500, "Insufficient normal grip force"
                    phase(1)  # Neutral must retain clamp force.
                raised, pos1 = phase(3, up=1)
                print("Lift:", start["lift"][0], "->", raised["lift"][0], flush=True)
                print("Poses:", pos0, "->", pos1, flush=True)
                assert raised["lift"][0] > start["lift"][0] + 0.25, "Boom did not rise"
                held, _ = phase(2)
                print("Neutral hold:", held["lift"], flush=True)
                assert abs(held["lift"][0]-raised["lift"][0]) < 0.12, "Neutral did not hold"
                if args.grip:
                    assert pos1['bale_1'][1] > pos0['bale_1'][1] + .4, "Bale slipped during lift"
                    carried, carried_pos = phase(2, drive_speed=.3)
                    assert carried_pos['bale_1'][0] > pos1['bale_1'][0] + .2, "Bale did not travel with tractor"
                    assert carried_pos['bale_1'][1] > pos0['bale_1'][1] + .4, "Bale dropped during transport"
                    opened, released = phase(5, squeeze=-1)
                    print("Released:", released, flush=True)
                    for jaw in ['grip_left','grip_right']:
                        assert opened[jaw][0] < .05, "Grip did not reopen"
                    assert .35 < released['bale_1'][1] < .6, "Released bale did not return to ground"
                    print('PASS: contact-only grip, lift, hold, transport and release of 525 kg bale', flush=True)
                rolled, _ = phase(1.5, roll=1)
                assert rolled["tilt"][0] > held["tilt"][0] + 0.15, "Tilt did not roll back"
                dumped, _ = phase(1.5, roll=-1)
                assert dumped["tilt"][0] < rolled["tilt"][0] - 0.15, "Tilt did not dump"
                lowered, _ = phase(1.5, up=-1)
                assert lowered["lift"][0] < held["lift"][0] - 0.12, "Boom did not lower"
                # Stop publishing while lift is commanded: timeout must neutralize.
                phase(1, up=1)
                expired, _ = phase(2, send=False)
                assert expired["lift"][5] < 0.02, "Stale valve command did not expire"
                if args.loaded:
                    assert pos1["bale_1"][1] > pos0["bale_1"][1] + 0.2, \
                        "Supported bale did not rise with the fork"
                with lock:
                    for which, state in samples:
                        assert all(math.isfinite(v) for v in state), "Nonfinite telemetry"
                        assert 0 <= state[4] <= (100.0001 if which.startswith("grip_") else 200.0001), "Pressure exceeded limit"
                        low, high = (0, .70) if which.startswith("grip_") else (0, 1.44) if which == "lift" else (-.65, .85)
                        assert low-.02 <= state[0] <= high+.02, "Joint escaped limits"
                print("PASS: lift/lower, tilt/dump, neutral hold, timeout, pressure and joint bounds",
                      "with contact-gripped 525 kg bale" if args.grip else
                      "with supported 525 kg bale" if args.loaded else "unloaded", flush=True)
            except BaseException:
                print((path / "server.log").read_text()[-4000:], flush=True)
                raise
            finally:
                os.killpg(server.pid, signal.SIGINT)
                try:
                    server.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait()
                print("Gazebo shutdown status:", server.returncode, flush=True)
                text = (path / "server.log").read_text()
                if "malloc" in text:
                    print("Existing Gazebo allocator error occurred during shutdown.", flush=True)


if __name__ == "__main__":
    main()
