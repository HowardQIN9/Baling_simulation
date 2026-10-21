"""Small SE(2) motion-primitive planner and bounded Ackermann path follower.

The collision footprint includes the lowered, open grip. Observed bales are
obstacles. A rectangular field boundary is an explicit known-map assumption.
"""
import heapq
import math
import time

import numpy as np
from scipy.interpolate import CubicHermiteSpline
from .localization import REAR_AXLE_OFFSET

WHEELBASE = 2.642
MAX_CURVATURE = .19  # reserve steering authority for tracking corrections
CONTROL_MAX_CURVATURE = math.tan(.56) / WHEELBASE
FRONT, REAR, HALF_WIDTH = 5.2, -2.3, 1.55
GOAL_OFFSET = 7.15


def wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def rectangle(x, y, yaw, front, rear, half_width):
    local = np.array([[front, half_width], [rear, half_width],
                      [rear, -half_width], [front, -half_width]])
    c, s = math.cos(yaw), math.sin(yaw)
    return local @ np.array([[c, s], [-s, c]]) + [x, y]


def polygon_gap(a, b):
    """Largest separating-axis gap; <=0 means overlap (not Euclidean distance)."""
    edges = np.vstack([a[1]-a[0], a[2]-a[1], b[1]-b[0], b[2]-b[1]])
    axes = np.stack([-edges[:, 1], edges[:, 0]], axis=1)
    axes /= np.linalg.norm(axes, axis=1)[:, None]
    pa, pb = a @ axes.T, b @ axes.T
    return float(np.max(np.maximum(pb.min(axis=0)-pa.max(axis=0),
                                   pa.min(axis=0)-pb.max(axis=0))))


def footprint(pose, front_extent=FRONT):
    return rectangle(*pose, front_extent, REAR, HALF_WIDTH)


def obstacle_polygons(detections, margin=.15):
    return [rectangle(*d["position_world"][:2], d["yaw_world"],
                       1.2+margin, -1.2-margin, .6+margin) for d in detections]


def valid_pose(pose, obstacles, front_extent=FRONT):
    body = footprint(pose,front_extent)
    if np.any(np.abs(body) > 14.5):
        return False
    return all(polygon_gap(body, obstacle) > 0 for obstacle in obstacles)


def pregrasp(target, vehicle):
    bx, by = target["position_world"][:2]
    yaw = target["yaw_world"]
    # Select the nearer of the two equivalent long-axis approach ends.
    options = []
    for heading in (yaw, wrap(yaw+math.pi)):
        goal = (bx-GOAL_OFFSET*math.cos(heading), by-GOAL_OFFSET*math.sin(heading), heading)
        options.append(goal)
    return min(options, key=lambda p: math.hypot(p[0]-vehicle[0], p[1]-vehicle[1]))


def integrate(pose, distance, curvature):
    x, y, yaw = pose
    x -= REAR_AXLE_OFFSET*math.cos(yaw)
    y -= REAR_AXLE_OFFSET*math.sin(yaw)
    dyaw = distance * curvature
    if abs(curvature) < 1e-9:
        return (x+(distance+REAR_AXLE_OFFSET)*math.cos(yaw),
                y+(distance+REAR_AXLE_OFFSET)*math.sin(yaw), yaw)
    return (x+(math.sin(yaw+dyaw)-math.sin(yaw))/curvature+REAR_AXLE_OFFSET*math.cos(yaw+dyaw),
            y+(-math.cos(yaw+dyaw)+math.cos(yaw))/curvature+REAR_AXLE_OFFSET*math.sin(yaw+dyaw),
            wrap(yaw+dyaw))


def visible(pose, target, cameras):
    corners = rectangle(*target["position_world"][:2],target["yaw_world"],1.2,-1.2,.6)
    c,s = math.cos(pose[2]),math.sin(pose[2])
    body = (corners-pose[:2]) @ np.array([[c,-s],[s,c]])
    for camera in cameras.values():
        delta = body-np.array(camera["pose"][:2])
        angles = np.arctan2(delta[:,1],delta[:,0])-camera["pose"][5]
        if np.all(delta[:,0] > .2) and np.all(np.abs(angles) < camera["hfov"]/2-.08):
            return True
    return False


def connector(start, goal, direction, obstacles, pose_check=None):
    rear = [np.array(p[:2])-REAR_AXLE_OFFSET*np.array([math.cos(p[2]),math.sin(p[2])])
            for p in (start,goal)]
    distance = float(np.linalg.norm(rear[1]-rear[0]))
    if distance < .05:
        return None
    for scale in (1.0, 1.5, 2.0):
        length = distance*scale
        tangents = [[direction*length*math.cos(p[2]), direction*length*math.sin(p[2])]
                    for p in (start, goal)]
        spline = CubicHermiteSpline([0, 1], rear, tangents)
        t = np.linspace(0, 1, max(20, int(distance/.1)))
        points, d1, d2 = spline(t), spline(t, 1), spline(t, 2)
        speed = np.linalg.norm(d1, axis=1)
        if speed.min() < .1:
            continue
        curvature = (d1[:,0]*d2[:,1]-d1[:,1]*d2[:,0])/speed**3
        if np.max(np.abs(curvature)) > MAX_CURVATURE:
            continue
        yaw = np.arctan2(d1[:,1]*direction, d1[:,0]*direction)
        points += REAR_AXLE_OFFSET*np.stack([np.cos(yaw),np.sin(yaw)],axis=1)
        path = [(float(p[0]), float(p[1]), float(a), direction) for p,a in zip(points,yaw)]
        if all(valid_pose(p[:3], obstacles) and (pose_check is None or pose_check(p[:3])) for p in path):
            return path
    return None


def plan(start, goal, obstacles, timeout=15, target=None, cameras=None, front_extent=FRONT):
    def is_valid(p):
        return valid_pose(p, obstacles,front_extent) and (target is None or visible(p,target,cameras))
    if not is_valid(start) or not is_valid(goal):
        raise ValueError("Start or pre-grasp footprint is in collision / outside field")
    direct = connector(start, goal, 1, obstacles, is_valid)
    if direct:
        return direct, dict(expanded=0, method="curvature_checked_hermite")
    def key(p, direction):
        return (round(p[0]/.35), round(p[1]/.35), round(wrap(p[2])/.10), direction)
    def heuristic(p):
        return math.hypot(p[0]-goal[0], p[1]-goal[1]) + 1.5*abs(wrap(p[2]-goal[2]))
    # Nodes retain continuous poses; discretization is only a search index.
    nodes = [(tuple(start), -1, 1, 0.0)]
    queue = [(heuristic(start), 0)]
    costs = {key(start, 1): 0.0}
    deadline = time.monotonic()+timeout
    expanded = 0
    while queue and time.monotonic() < deadline:
        _, idx = heapq.heappop(queue)
        pose, parent, previous_direction, cost = nodes[idx]
        if cost > costs.get(key(pose, previous_direction), math.inf)+1e-6:
            continue
        expanded += 1
        if expanded % 6 == 0 and math.hypot(pose[0]-goal[0], pose[1]-goal[1]) < 7:
            tail = connector(pose, goal, previous_direction, obstacles, is_valid)
            if tail:
                chain = []
                while idx >= 0:
                    p, idx, direction, _ = nodes[idx]
                    chain.append((*p, direction))
                chain.reverse()
                return chain + tail[1:], dict(expanded=expanded, method="hybrid_astar_hermite")
        for direction in (1, -1):
            for curvature in (-MAX_CURVATURE, 0, MAX_CURVATURE):
                distance = direction*.6
                samples = [integrate(pose, distance*f, curvature) for f in (.33,.67,1)]
                if not all(is_valid(p) for p in samples):
                    continue
                p = samples[-1]
                new_cost = cost + .6*(1 if direction>0 else 1.8) + (
                    2.0 if direction != previous_direction else 0) + .1*abs(curvature)/MAX_CURVATURE
                k = key(p, direction)
                if new_cost >= costs.get(k, math.inf):
                    continue
                costs[k] = new_cost
                nodes.append((p, idx, direction, new_cost))
                heapq.heappush(queue, (new_cost+heuristic(p), len(nodes)-1))
    raise RuntimeError(f"No feasible approach within {timeout}s ({expanded} expansions)")


class Follower:
    def __init__(self, path, final_position_tolerance=.45,
                 heading_tolerance=math.radians(8), lookahead=1.6):
        self.path = np.asarray(path, dtype=float)
        self.final_position_tolerance = final_position_tolerance
        self.heading_tolerance = heading_tolerance
        self.lookahead = lookahead
        # A direction change labels the new edge, so share the cusp pose.
        self.sections = []
        begin = 0
        for i in range(1, len(path)):
            if self.path[i,3] != self.path[i-1,3]:
                if i-begin > 1:
                    self.sections.append((begin, i-1, self.path[i-1,3]))
                begin = i-1
        self.sections.append((begin, len(path)-1, self.path[-1,3]))
        self.section = 0
        self.progress = 0

    def command(self, pose, measured_speed):
        begin, end, direction = self.sections[self.section]
        segment = self.path[begin:end+1]
        position = np.array(pose[:2])-REAR_AXLE_OFFSET*np.array([math.cos(pose[2]),math.sin(pose[2])])
        rear_points = segment[:,:2]-REAR_AXLE_OFFSET*np.stack([
            np.cos(segment[:,2]),np.sin(segment[:,2])],axis=1)
        distances = np.linalg.norm(rear_points-position, axis=1)
        nearest = int(np.argmin(distances))
        remaining = float(np.linalg.norm(rear_points[-1]-position))
        heading_error = abs(wrap(pose[2]-segment[-1,2]))
        final = self.section == len(self.sections)-1
        # At a final point, position alone is insufficient: a long bale in
        # front of the tractor turns a small heading error into a large drop
        # error. Keep following slowly until both pose components converge.
        terminal_axis = np.array([math.cos(segment[-1,2]),math.sin(segment[-1,2])])
        terminal_offset = position-rear_points[-1]
        passed_end = direction*float(terminal_offset@terminal_axis) > 0
        cross_track = abs(float(terminal_offset[0]*terminal_axis[1]-
                                terminal_offset[1]*terminal_axis[0]))
        if (final and heading_error < self.heading_tolerance and
                (remaining < self.final_position_tolerance or
                 (passed_end and cross_track < 2*self.final_position_tolerance))):
            return 0., 0., True
        if not final and remaining < .28:
            if not final and abs(measured_speed) < .08:
                self.section += 1
            return 0., 0., False
        target_idx = nearest
        arc = 0.
        while target_idx < len(segment)-1 and arc < self.lookahead:
            arc += np.linalg.norm(rear_points[target_idx+1]-rear_points[target_idx])
            target_idx += 1
        delta = rear_points[target_idx]-position
        lateral = -math.sin(pose[2])*delta[0] + math.cos(pose[2])*delta[1]
        curvature = np.clip(2*lateral/max(float(delta@delta), .16), -CONTROL_MAX_CURVATURE, CONTROL_MAX_CURVATURE)
        cruise = .32 if abs(curvature)>.12 else .45
        speed = direction*min(cruise, max(.10, .5*remaining))
        return float(speed), float(speed*curvature), False
