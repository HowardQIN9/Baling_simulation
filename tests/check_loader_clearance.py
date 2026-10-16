#!/usr/bin/env python3
"""Conservative wheel/loader box-envelope check over sampled joint ranges.

Each tire cylinder is enclosed in an oriented box. Separation of these boxes
therefore guarantees separation from the actual tire at each sampled pose.
This checks geometry independently of Gazebo's disabled model self-collision.
"""
from itertools import product
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def transform(text):
    x, y, z, roll, pitch, yaw = map(float, (text or "0 0 0 0 0 0").split())
    c, s = math.cos(roll), math.sin(roll)
    rx = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    t = np.eye(4)
    t[:3, :3] = rz(yaw) @ ry(pitch) @ rx
    t[:3, 3] = [x, y, z]
    return t


def box(collision, link_pose):
    t = link_pose @ transform(collision.findtext("pose"))
    size = collision.findtext("geometry/box/size")
    if size is not None:
        half = np.array(list(map(float, size.split()))) / 2
    else:
        radius = float(collision.findtext("geometry/cylinder/radius"))
        half = np.array([radius, radius,
                         float(collision.findtext("geometry/cylinder/length"))/2])
    return t[:3, 3], t[:3, :3], half


def separation(a, b):
    ca, ra, ha = a
    cb, rb, hb = b
    delta = cb - ca
    axes = [ra[:, i] for i in range(3)] + [rb[:, i] for i in range(3)]
    axes += [np.cross(ra[:, i], rb[:, j]) for i, j in product(range(3), repeat=2)]
    largest = -math.inf
    for axis in axes:
        length = np.linalg.norm(axis)
        if length < 1e-8:
            continue
        axis = axis / length
        gap = abs(delta @ axis) - np.abs(ra.T @ axis) @ ha - np.abs(rb.T @ axis) @ hb
        largest = max(largest, gap)
    return largest


def main():
    root = Path(__file__).resolve().parents[1]
    model = ET.parse(root / "worlds/simple_tractor.sdf").getroot().find(
        "world/model[@name='simple_tractor']")
    links = {l.get("name"): l for l in model.findall("link")}
    boom0 = transform(links["loader_boom"].findtext("pose"))
    carrier0 = transform(links["attachment_carrier"].findtext("pose"))
    carrier_relative = np.linalg.inv(boom0) @ carrier0
    wheels = []
    for name in ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]:
        link = links[name]
        initial = transform(link.findtext("pose"))
        for steering in (np.linspace(-.6, .6, 25) if name.startswith("front") else [0]):
            t = initial.copy()
            t[:3, :3] = rz(steering) @ initial[:3, :3]
            wheels.append((name, steering, box(link.find("collision"), t)))
    smallest = math.inf
    worst = None
    checked = 0
    for lift in np.linspace(0, 1.44, 73):
        boom = boom0.copy()
        boom[:3, :3] = ry(-lift) @ boom0[:3, :3]
        loader = []
        for name, t in [("loader_mount", transform(links["loader_mount"].findtext("pose"))),
                        ("loader_boom", boom)]:
            loader += [(name+"/"+c.get("name"), box(c, t)) for c in links[name].findall("collision")]
        for tilt in np.linspace(-.65, .85, 16):
            t = boom @ carrier_relative
            t[:3, :3] = ry(-tilt) @ t[:3, :3]
            loader += [(f"carrier@{tilt:.2f}/"+c.get("name"), box(c, t))
                       for c in links["attachment_carrier"].findall("collision")]
            for side, sign in [("left", 1), ("right", -1)]:
                jaw = links.get(f"grip_{side}")
                if jaw is None:
                    continue
                for travel in [0, .35, .70]:
                    local = transform(jaw.findtext("pose"))
                    local[1, 3] -= sign * travel
                    loader += [(f"{side} jaw@{tilt:.2f},{travel:.2f}/"+c.get("name"),
                                box(c, t @ local)) for c in jaw.findall("collision")]
        for loader_name, a in loader:
            for wheel_name, steering, b in wheels:
                # AABB rejection is also a valid separating-axis lower bound.
                gap = np.abs(a[0]-b[0])-np.abs(a[1])@a[2]-np.abs(b[1])@b[2]
                sep = float(max(gap))
                if sep < .05:
                    sep = separation(a, b)
                checked += 1
                if sep < smallest:
                    smallest = sep
                    worst = (loader_name, wheel_name, float(lift), float(steering))
                assert sep > .02, f"Wheel/loader clearance below 2 cm: {sep}, {worst}"
    print(f"PASS: {checked} conservative wheel-envelope pairs; "
          f"minimum separating-axis gap {smallest:.3f} m; limiting pose {worst}")


if __name__ == "__main__":
    main()
