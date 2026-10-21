"""Generate reproducible trials without modifying the user's normal world."""
import copy
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CAMERAS = {
    "front": {"pose": [2.05, 0, 2.25, 0, .20, 0], "hfov": math.pi / 3},
    "right": {"pose": [1.8, -1.65, 3.25, 0, .32, 0], "hfov": math.radians(90)},
    "left": {"pose": [1.8, 1.65, 3.25, 0, .32, 0], "hfov": math.radians(90)},
    # Elevated forward view clears most of the boom and fork occlusion during
    # high placement while retaining a view of the bale top and stack surface.
    "mast": {"pose": [-.35, 0, 4.55, 0, .52, 0], "hfov": math.radians(82)},
}


def camera_config(width=320, height=240):
    result = copy.deepcopy(CAMERAS)
    for name, cfg in result.items():
        f = width / (2 * math.tan(cfg["hfov"] / 2))
        cfg.update(width=width, height=height, fx=f, fy=f, cx=width / 2,
                   cy=height / 2, near=.1, far=30, rate_hz=10,
                   frame=name + "_camera", depth_convention="axial_forward_m",
                   topic="/tractor/camera/" + name)
    return result


def scenario(seed=0, split="train", **overrides):
    rng = np.random.default_rng(seed)
    cfg = dict(seed=seed, split=split, target=[float(rng.uniform(10, 13)),
               float(rng.uniform(-1.5, 1.5)), float(rng.uniform(-.35, .35))],
               ground_color=[.28, .34, .13], bale_color=[.68, .53, .25],
               light=float(rng.uniform(.55, .95)), clutter=True,
               loader_lift=0.0, cameras=["front", "right"])
    if seed % 3 == 1:
        cfg["ground_color"] = [.43, .35, .19]  # dry soil: reduced color contrast
        cfg["bale_color"] = [.57, .46, .24]
    if seed % 3 == 2:
        cfg["ground_color"] = [.39, .38, .23]
        cfg["bale_color"] = [.52, .46, .28]
    cfg.update(overrides)
    return cfg


def make_world(directory, cfg):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(ROOT / "worlds/simple_tractor.sdf")
    world = tree.getroot().find("world")
    tractor = world.find("model[@name='simple_tractor']")
    ET.SubElement(world, "plugin", filename="gz-sim-imu-system", name="gz::sim::systems::Imu")
    # Absolute library path eliminates dependency on the launching shell.
    tractor.find("plugin[@name='bale_sim::LoaderHydraulics']").set(
        "filename", str(ROOT / "build/libBaleLoaderHydraulics.so"))
    chassis = tractor.find("link[@name='chassis']")
    for sensor in list(chassis.findall("sensor")):
        chassis.remove(sensor)
    imu = ET.SubElement(chassis, "sensor", name="tractor_imu", type="imu")
    ET.SubElement(imu, "always_on").text = "true"
    ET.SubElement(imu, "update_rate").text = "100"
    ET.SubElement(imu, "topic").text = "/tractor/imu"
    ET.SubElement(imu, "imu")
    cameras = camera_config()
    for name in cfg["cameras"]:
        c = cameras[name]
        for kind in ("rgb", "depth"):
            sensor = ET.SubElement(chassis, "sensor", name=name + "_" + kind,
                                   type="camera" if kind == "rgb" else "depth_camera")
            pose = list(c["pose"])
            pose[2] -= 1.05  # sensor is attached to the chassis link
            ET.SubElement(sensor, "pose").text = " ".join(map(str, pose))
            ET.SubElement(sensor, "always_on").text = "true"
            ET.SubElement(sensor, "update_rate").text = str(c["rate_hz"])
            ET.SubElement(sensor, "topic").text = c["topic"] + "/" + kind
            camera = ET.SubElement(sensor, "camera")
            ET.SubElement(camera, "horizontal_fov").text = str(c["hfov"])
            img = ET.SubElement(camera, "image")
            ET.SubElement(img, "width").text = str(c["width"])
            ET.SubElement(img, "height").text = str(c["height"])
            ET.SubElement(img, "format").text = "R8G8B8"
            clip = ET.SubElement(camera, "clip")
            ET.SubElement(clip, "near").text = ".1"
            ET.SubElement(clip, "far").text = "30"
    x, y, yaw = cfg["target"]
    world.find("model[@name='bale_1']/pose").text = f"{x} {y} .45 0 0 {yaw}"
    for name, pose in [("bale_2", "12 6 .45 0 0 .5"),
                       ("bale_3", "11 -6 .45 0 0 -.5")]:
        world.find(f"model[@name='{name}']/pose").text = pose
    def color(path, rgb):
        material = world.find(path)
        for tag in ("ambient", "diffuse"):
            material.find(tag).text = " ".join(map(str, rgb)) + " 1"
    color("model[@name='ground']/link/visual/material", cfg["ground_color"])
    for name in ("bale_1", "bale_2", "bale_3"):
        color(f"model[@name='{name}']/link/visual/material", cfg["bale_color"])
    world.find("light/diffuse").text = " ".join([str(cfg["light"])] * 3) + " 1"
    if cfg.get("clutter"):
        # Visual-only patches add cheap ground variation without changing contact.
        rng = np.random.default_rng(cfg["seed"] + 10000)
        ground_link = world.find("model[@name='ground']/link")
        for i in range(24):
            visual = ET.SubElement(ground_link, "visual", name=f"soil_patch_{i}")
            px, py = rng.uniform(-13, 13, 2)
            ET.SubElement(visual, "pose").text = f"{px} {py} .052 0 0 {rng.uniform(-3, 3)}"
            box = ET.SubElement(ET.SubElement(visual, "geometry"), "box")
            ET.SubElement(box, "size").text = f"{rng.uniform(.3, 2)} {rng.uniform(.2, 1)} .002"
            mat = ET.SubElement(visual, "material")
            rgb = np.clip(np.array(cfg["ground_color"]) * rng.uniform(.75, 1.25), 0, 1)
            for tag in ("ambient", "diffuse"):
                ET.SubElement(mat, tag).text = " ".join(map(str, rgb)) + " 1"
    path = directory / "world.sdf"
    tree.write(path, encoding="unicode")
    sources = sorted((ROOT / "bale_autonomy").glob("*.py")) + [
        ROOT / "scripts/bale_autonomy.py",ROOT / "worlds/simple_tractor.sdf",
        ROOT / "requirements-autonomy.txt"]
    fingerprints = {}
    for source in sources:
        relative = source.relative_to(ROOT)
        data = source.read_bytes()
        fingerprints[str(relative)] = hashlib.sha256(data).hexdigest()
        target = directory / "source" / relative
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes(data)
    metadata = dict(schema_version=1, source_sha256=fingerprints, scenario=cfg, cameras={
        name: cameras[name] for name in cfg["cameras"]},
        frames={"model": "tractor reference used by Gazebo Ackermann odometry",
                "camera": "+X forward, +Y left, +Z up; poses relative to model",
                "tractor_level": "model origin and heading, roll/pitch removed; frame of position_model detections",
                "world": "initial tractor model pose; meters, radians"},
        bale_size_m=[2.4, 1.2, .9], source_world="worlds/simple_tractor.sdf",
        localization="wheel-speed integration with ideal simulated IMU heading; initial model pose zero",
        imu=dict(topic="/tractor/imu", rate_hz=100, ideal=True, frame="chassis"))
    metadata["software"] = dict(python=sys.version.split()[0],
        packages={name:version(name) for name in ("numpy","scipy","opencv-python-headless")})
    metadata["hydraulic_plugin_sha256"] = hashlib.sha256(
        (ROOT / "build/libBaleLoaderHydraulics.so").read_bytes()).hexdigest()
    (directory / "metadata.json").write_text(json.dumps(metadata, indent=2))
    return path, metadata
