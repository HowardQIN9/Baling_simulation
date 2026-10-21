"""Known-size square-bale RGB-D baseline; no simulator labels enter estimation.

This is a geometric baseline for upright bales on level ground, not a trained
real-world detector. A visible top face is required to resolve box orientation.
"""
import math

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def wrap_axis(angle):
    """A rectangular bale's horizontal orientation is ambiguous modulo pi."""
    return (angle + math.pi / 2) % math.pi - math.pi / 2


def yaw_of(pose):
    q = pose["quaternion_xyzw"]
    return math.atan2(2*(q[3]*q[2] + q[0]*q[1]), 1 - 2*(q[1]**2 + q[2]**2))


def cloud(depth, camera, attitude=None):
    v, u = np.indices(depth.shape)
    with np.errstate(invalid="ignore"):
        points = np.stack([depth, -(u-camera["cx"]) * depth / camera["fx"],
                           -(v-camera["cy"]) * depth / camera["fy"]], axis=-1)
    rot = Rotation.from_euler("xyz", camera["pose"][3:]).as_matrix()
    with np.errstate(invalid="ignore"):
        points = points @ rot.T + camera["pose"][:3]
        if attitude is not None:
            # Level the body frame before ground/height filtering. Yaw is applied
            # separately by to_world; roll/pitch are essential during steering.
            full = Rotation.from_quat(attitude)
            yaw = yaw_of(dict(quaternion_xyzw=attitude))
            leveling = Rotation.from_euler("z", -yaw).as_matrix() @ full.as_matrix()
            points = points @ leveling.T
        return points


def detect(rgb, depth, camera, attitude=None, minimum_forward=5.25):
    points = cloud(depth, camera, attitude)
    valid = np.isfinite(depth) & (depth > camera["near"]) & (depth < camera["far"])
    # Self-mask covers the lowered loader/grip. Raised parts above the height gate
    # also disappear; remaining intrusions must pass the known-size top-face test.
    valid &= (points[..., 0] > minimum_forward) & (points[..., 2] > .15) & (points[..., 2] < 1.12)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    # Broad straw-color cue; geometry carries separation from similarly colored soil.
    valid &= (hsv[..., 0] >= 8) & (hsv[..., 0] <= 48) & (hsv[..., 1] > 25)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(valid.astype(np.uint8), 8)
    result = []
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] < 18:
            continue
        mask = labels == label
        pts = points[mask]
        top = pts[(pts[:, 2] > .83) & (pts[:, 2] < .97), :2]
        if len(top) < 8:
            continue
        rect = cv2.minAreaRect(top.astype(np.float32))
        corners = cv2.boxPoints(rect).astype(float)
        edges = np.roll(corners, -1, axis=0) - corners
        lengths = np.linalg.norm(edges, axis=1)
        edge = edges[np.argmax(lengths)]
        long, short = float(max(lengths)), float(min(lengths))
        # Do not report a precise center for a visibly truncated top face.
        if not (2.05 < long < 2.65 and .95 < short < 1.4):
            continue
        yaw = wrap_axis(math.atan2(edge[1], edge[0]))
        center = np.array(rect[0], dtype=float)
        # Allow for finite pixel sampling of edges; inflation is uncertainty, not
        # a correction based on simulator ground truth.
        sigma = max(.04, .5 * max(2.4-long, 1.2-short))
        yaw_sigma = max(math.radians(2), math.atan2(2*sigma, 2.4))
        x, y, w, h, area = stats[label].tolist()
        result.append(dict(position_model=[float(center[0]), float(center[1]), .45],
            position_frame="tractor_level",
            yaw_model=yaw, position_std_m=sigma, yaw_std_rad=yaw_sigma,
            observed_size_m=[long, short], top_points=len(top), pixels=area,
            bbox=[x, y, w, h], method="rgb_depth_top_rectangle"))
    return sorted(result, key=lambda d: d["position_model"][0] +
                  1.5 * abs(d["position_model"][1]))


def to_world(detection, odometry):
    yaw = yaw_of(odometry)
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, _ = detection["position_model"]
    px, py = odometry["position"][:2]
    return dict(detection, position_world=[px+c*x-s*y, py+s*x+c*y, .45],
                yaw_world=wrap_axis(detection["yaw_model"] + yaw))


class TargetTracker:
    """Conservative static-target association and exponentially weighted fusion.

    Variance floors prevent same-camera repeated frames from claiming unlimited
    independent information. Values are heuristic, not calibrated covariances.
    """
    def __init__(self, initial_region="forward"):
        self.initial_region = initial_region
        self.target = None
        self.history = []
        self.rejects = 0

    def update(self, candidates, stamp, camera):
        candidates = [c for c in candidates if c["position_std_m"] <= .18]
        if self.target is None:
            if self.initial_region == "left":
                candidates = sorted([c for c in candidates if c["position_world"][1] > 3],
                                    key=lambda c:-c["position_world"][1])
            elif self.initial_region == "right":
                candidates = sorted([c for c in candidates if c["position_world"][1] < -3],
                                    key=lambda c:c["position_world"][1])
            else:
                candidates = [c for c in candidates if abs(c["position_model"][1]) < 4]
        if not candidates:
            return None
        if self.target is None:
            chosen = candidates[0]
        else:
            chosen = min(candidates, key=lambda c: np.linalg.norm(
                np.array(c["position_world"][:2]) - self.target["position_world"][:2]))
            innovation = np.linalg.norm(np.array(chosen["position_world"][:2]) -
                                        self.target["position_world"][:2])
            if innovation > 1.0 or abs(wrap_axis(chosen["yaw_world"] -
                                               self.target["yaw_world"])) > .4:
                self.rejects += 1
                return None
        if self.target is None:
            self.target = dict(chosen, last_seen=stamp, observations=1, cameras=[camera])
        else:
            weight = .25
            old = self.target
            position = (1-weight)*np.array(old["position_world"]) + weight*np.array(chosen["position_world"])
            yaw = wrap_axis(old["yaw_world"] + weight*wrap_axis(chosen["yaw_world"]-old["yaw_world"]))
            self.target = dict(chosen, position_world=position.tolist(), yaw_world=yaw,
                position_std_m=max(.04, (1-weight)*old["position_std_m"] + weight*chosen["position_std_m"]),
                last_seen=stamp, observations=old["observations"]+1,
                cameras=sorted(set(old["cameras"] + [camera])))
        self.history.append(dict(self.target))
        return dict(self.target)


def annotate(rgb, detections, target=None):
    image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    for d in detections:
        x, y, w, h = d["bbox"]
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 1)
        text = f"{d['position_model'][0]:.2f}m yaw {math.degrees(d['yaw_model']):.1f}"
        cv2.putText(image, text, (max(0, x-20), max(12, y-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .32, (0, 255, 0), 1)
    return image
