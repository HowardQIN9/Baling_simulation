"""Conventional contact-only pickup, low carry, ground placement and withdrawal.

Control reads RGB-D, estimated vehicle state and hydraulic/jaw feedback only.
No simulator attachment joint, teleportation or simulator object pose is used.
"""
import json
import math

import cv2
import numpy as np

from .perception import cloud, wrap_axis
from .planning import Follower, connector, integrate, obstacle_polygons, plan, valid_pose, wrap

CARRY_LIFT = .25
LOADED_FRONT = 6.2


def carrier_position(lift):
    return np.array([.4+2.8*math.cos(-.5+lift),0.,1.55+2.8*math.sin(-.5+lift)])


def carrier_rotation(lift,tilt):
    angle = -lift-tilt
    c,s = math.cos(angle),math.sin(angle)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])


def valve_for_speed(axis,speed):
    # Invert the existing equivalent cylinder's flow/speed relation.
    bore,rod,arm,flow = (.07,.04,.32,60) if axis=="lift" else (.05,.03,.18,45)
    area = math.pi*(bore*bore-(rod*rod if speed<0 else 0))/4
    valve = speed*2*area*arm/(flow/60000)
    return float(np.clip(valve,-.7,.7)) if abs(valve)>.05 else 0.


def payload_observation(rgb, depth, camera, attitude, maximum_forward=6.6):
    """Find a straw-colored cuboid near the grip, including above ground level."""
    xyz = cloud(depth,camera,attitude)
    hsv = cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    mask = np.isfinite(depth) & (depth>.1) & (xyz[...,0]>3.0) & (xyz[...,0]<maximum_forward)
    mask &= (np.abs(xyz[...,1])<.95) & (xyz[...,2]>.14) & (xyz[...,2]<3.4)
    mask &= (hsv[...,0]>=8) & (hsv[...,0]<=48) & (hsv[...,1]>25)
    count,labels,stats,_ = cv2.connectedComponentsWithStats(mask.astype(np.uint8),8)
    candidates = []
    for label in range(1,count):
        if stats[label,cv2.CC_STAT_AREA] < 30:
            continue
        points = xyz[labels==label]
        top_height = float(np.percentile(points[:,2],98))
        top = points[points[:,2]>top_height-.14,:2]
        if len(top)<12:
            continue
        rect = cv2.minAreaRect(top.astype(np.float32))
        corners = cv2.boxPoints(rect).astype(float)
        edges = np.roll(corners,-1,axis=0)-corners
        lengths = np.linalg.norm(edges,axis=1)
        if not (1.95<max(lengths)<2.7 and .85<min(lengths)<1.45):
            continue
        edge = edges[np.argmax(lengths)]
        candidates.append(dict(position_model=[float(rect[0][0]),float(rect[0][1]),top_height-.45],
            position_frame="tractor_level",
            yaw_model=wrap_axis(math.atan2(edge[1],edge[0])),top_height_m=top_height,
            observed_size_m=[float(max(lengths)),float(min(lengths))]))
    return min(candidates,key=lambda p:abs(p["position_model"][0]-4.5)) if candidates else None


def jaws_loaded(loader):
    """Closure away from end stops plus both jaws pushing; not proof of pickup."""
    for name in ("grip_left","grip_right"):
        state = loader.get(name)
        if not state or not (.15<state[0]<.58 and abs(state[1])<.025 and state[3]>6500):
            return False
    return True


class PickPlace:
    def __init__(self,target,observed,drop_pose,directory,visual=None,stop_after_pickup=False):
        self.target = dict(target)
        self.drop = np.array(drop_pose,dtype=float)
        self.directory,self.visual = directory,visual
        others = list(observed)
        if others:
            # The tracker estimate can drift away from its first map detection
            # as the tractor moves. Remove the nearest source landmark by
            # identity-at-acquisition instead of relying on a fixed distance.
            selected = min(range(len(others)),key=lambda i: np.linalg.norm(
                np.array(others[i]["position_world"][:2])-
                np.array(target["position_world"][:2])))
            others.pop(selected)
        self.obstacles = obstacle_polygons(others)
        self.phase = "fine_approach"
        self.started = None
        self.phase_started = None
        self.stable_since = None
        self.follower = None
        self.payload = None
        self.payload_stamp = -1e9
        self.grip_reference = None
        self.lift_start_height = None
        self.retreat_start = None
        self.goal = None
        self.events = []
        self.stop_after_pickup = stop_after_pickup
        self.finished = False
        self.pickup_verified = False
        self.plans = 0
        self.travel_speed_limit = .25

    def observe(self, observation, stamp):
        if observation is not None:
            self.payload = dict(observation)
            self.payload_stamp = stamp

    def load_evidence(self,now,loader):
        visible = self.payload is not None and now-self.payload_stamp<.65
        contact = jaws_loaded(loader)
        residual = None
        if visible and self.grip_reference is not None:
            q,tilt = loader["lift"][0],loader["tilt"][0]
            expected = carrier_position(q)+carrier_rotation(q,tilt) @ self.grip_reference
            residual = float(np.linalg.norm(np.array(self.payload["position_model"])-expected))
        carrying = bool(self.pickup_verified and contact and visible and residual is not None
                        and residual<.45 and self.payload["position_model"][2]>.65)
        return dict(jaw_contact=contact,payload_visible=visible,
                    relative_position_error_m=residual,carrying_verified=carrying)

    def transition(self,phase,now,details=None):
        print(f"Pick/place: {self.phase} -> {phase}",flush=True)
        self.events.append(dict(stamp=now,phase=phase,details=details))
        self.phase,self.phase_started = phase,now
        self.stable_since = None
        (self.directory/"pick_place_events.json").write_text(json.dumps(self.events,indent=2))

    def path(self,pose,goal,front=5.2):
        points,info = plan(pose,goal,self.obstacles,timeout=25,front_extent=front)
        self.follower = Follower(points)
        self.goal = goal
        self.plans += 1
        (self.directory/f"manipulation_plan_{self.plans}.json").write_text(json.dumps(dict(
            phase=self.phase,start=pose,goal=goal,path=points,planner=info,front_extent_m=front),indent=2))
        if self.visual:
            self.visual.set_plan(points,goal)

    def follow(self,pose,speed,limit,front=5.2):
        velocity,yaw_rate,done = self.follower.command(pose,speed)
        if abs(velocity)>limit:
            yaw_rate *= limit/abs(velocity)
            velocity = math.copysign(limit,velocity)
        future = integrate(pose,velocity*.8,yaw_rate/velocity if velocity else 0)
        if not valid_pose(future,self.obstacles,front_extent=front):
            raise RuntimeError("Pick/place predicted obstacle collision")
        return velocity,yaw_rate,done

    def step(self,now,pose,odom,loader,ground_target):
        if self.started is None:
            self.started = self.phase_started = now
        q = loader["lift"][0]
        qdot = loader["lift"][1]
        tilt_q = loader["tilt"][0]
        tilt_dot = loader["tilt"][1]
        command = dict(speed=0.,yaw_rate=0.,lift=0.,tilt=0.,grip=0.)
        # Keep the attachment level while the boom changes angle.
        tilt_speed = np.clip(-qdot+1.2*(-q-tilt_q)-.35*(qdot+tilt_dot),-.16,.16)
        command["tilt"] = valve_for_speed("tilt",tilt_speed)
        elapsed = now-self.phase_started
        fresh = self.payload is not None and now-self.payload_stamp<.65
        if self.phase == "fine_approach":
            if ground_target is None or now-ground_target["last_seen"]>1:
                raise RuntimeError("Bale observation lost during grip alignment")
            if self.follower is None:
                yaw = self.target["yaw_world"]
                offset = carrier_position(0)[0]+1.5
                x,y = self.target["position_world"][:2]
                goal = (x-offset*math.cos(yaw),y-offset*math.sin(yaw),yaw)
                self.path(pose,goal)
            command["lift"] = float(np.clip(-4*q,-.5,.5)) if q>.015 else 0.
            v,w,done = self.follow(pose,odom["linear"][0],.16)
            command.update(speed=v,yaw_rate=w)
            if done:
                if not fresh:
                    raise RuntimeError("No visible bale between the open grip jaws")
                self.lift_start_height = self.payload["position_model"][2]
                self.transition("clamp",now)
            if elapsed>50:
                raise TimeoutError("Grip alignment timeout")
        elif self.phase == "clamp":
            command["grip"] = .9
            if jaws_loaded(loader):
                self.stable_since = now if self.stable_since is None else self.stable_since
                if now-self.stable_since>.6 and fresh:
                    self.grip_reference = carrier_rotation(q,tilt_q).T @ (
                        np.array(self.payload["position_model"])-carrier_position(q))
                    self.grip_yaw = self.payload["yaw_model"]
                    command["grip"] = 0.
                    self.transition("lift_verify",now,dict(jaws=loader,payload=self.payload))
            else:
                self.stable_since = None
            if elapsed>9:
                raise RuntimeError("Grip did not establish pressure against both sides of the bale")
        elif self.phase == "lift_verify":
            error = CARRY_LIFT-q
            lift_speed = 1.5*error-.65*qdot
            command["lift"] = valve_for_speed("lift",np.clip(lift_speed,-.07,.07))
            if abs(error)<.035 and abs(q+tilt_q)<.05 and abs(qdot+tilt_dot)<.045 and fresh and self.payload["position_model"][2]>self.lift_start_height+.3:
                self.stable_since = now if self.stable_since is None else self.stable_since
                if now-self.stable_since>1.2:
                    self.pickup_verified = True
                    # Recalibrate the held-load offset from its raised, less
                    # occluded camera view before judging transport slip.
                    self.grip_reference = carrier_rotation(q,tilt_q).T @ (
                        np.array(self.payload["position_model"])-carrier_position(q))
                    self.grip_yaw = self.payload["yaw_model"]
                    self.retreat_start = np.array(pose[:2])
                    self.transition("done" if self.stop_after_pickup else "retreat",now,
                                    dict(payload=self.payload,lift_angle=q))
            else:
                self.stable_since = None
            if elapsed>25:
                raise RuntimeError("Lift did not produce a verified elevated load")
        elif self.phase in ("retreat","transport"):
            error = CARRY_LIFT-q
            command["lift"] = valve_for_speed("lift",np.clip(1.2*error,-.04,.04))
            if not fresh:
                if now-self.payload_stamp>6:
                    raise RuntimeError("Lost camera verification of carried bale")
                return command
            expected = carrier_position(q)+carrier_rotation(q,tilt_q) @ self.grip_reference
            slip = np.linalg.norm(np.array(self.payload["position_model"])-expected)
            if slip>.65 or self.payload["position_model"][2]<self.lift_start_height+.2:
                raise RuntimeError(f"Carried bale slipped or fell (relative error {slip:.2f}m)")
            if self.phase == "retreat":
                command["speed"] = -.20
                if np.linalg.norm(np.array(pose[:2])-self.retreat_start)>1.6:
                    command["speed"] = 0.
                    self.transition("transport_plan",now)
            else:
                v,w,done = self.follow(pose,odom["linear"][0],self.travel_speed_limit,LOADED_FRONT)
                command.update(speed=v,yaw_rate=w)
                if done:
                    self.transition("ground_insert",now)
                if elapsed>160:
                    raise TimeoutError("Loaded transport timeout")
        elif self.phase == "ground_insert":
            if not fresh:
                if now-self.payload_stamp>6:
                    raise RuntimeError("Lost camera verification during ground placement alignment")
                return command
            payload = np.asarray(self.payload["position_model"])
            heading = wrap(self.drop[2]-self.payload["yaw_model"])
            c,s = math.cos(heading),math.sin(heading)
            goal = (self.drop[0]-c*payload[0]+s*payload[1],
                    self.drop[1]-s*payload[0]-c*payload[1],heading)
            dx,dy = goal[0]-pose[0],goal[1]-pose[1]
            cy,sy = math.cos(pose[2]),math.sin(pose[2])
            longitudinal = cy*dx+sy*dy
            lateral = -sy*dx+cy*dy
            heading_error = wrap(heading-pose[2])
            current_world = np.array([pose[0]+cy*payload[0]-sy*payload[1],
                                      pose[1]+sy*payload[0]+cy*payload[1]])
            if np.linalg.norm(current_world-self.drop[:2])<.18 and abs(heading_error)<.035:
                self.transition("lower",now)
            else:
                direction = (1. if longitudinal>=0 else -1.) if abs(longitudinal)>.06 else 1.
                command["speed"] = direction*min(.08,max(.06,.6*abs(longitudinal)))
                lookahead = max(.25,math.hypot(longitudinal,lateral))
                alpha = math.atan2(direction*lateral,abs(longitudinal)+.12)
                curvature = np.clip(2*math.sin(alpha)/lookahead+1.2*heading_error,-.20,.20)
                command["yaw_rate"] = float(command["speed"]*curvature)
            if elapsed>40:
                raise TimeoutError("Ground placement alignment timeout")
        elif self.phase == "transport_plan":
            heading = wrap(self.drop[2]-self.grip_yaw)
            floor_offset = carrier_position(0)+self.grip_reference
            c,s = math.cos(heading),math.sin(heading)
            goal = (self.drop[0]-c*floor_offset[0]+s*floor_offset[1],
                    self.drop[1]-s*floor_offset[0]-c*floor_offset[1],heading)
            self.path(pose,goal,LOADED_FRONT)
            self.transition("transport",now)
        elif self.phase == "lower":
            command["lift"] = valve_for_speed("lift",max(-.055,-1.5*q)) if q>.008 else 0.
            if fresh and self.payload["position_model"][2]<.54 and abs(q+tilt_q)<.03:
                command["lift"] = 0.
                self.stable_since = now if self.stable_since is None else self.stable_since
                if now-self.stable_since>.7:
                    self.transition("release",now,dict(payload=self.payload))
            if elapsed>14:
                raise RuntimeError("Could not verify ground support before release")
        elif self.phase == "release":
            command["grip"] = -.8
            if all(loader[axis][0]<.035 for axis in ("grip_left","grip_right")):
                self.stable_since = now if self.stable_since is None else self.stable_since
                if now-self.stable_since>.6:
                    self.retreat_start = np.array(pose[:2])
                    self.transition("withdraw",now)
            if elapsed>10:
                raise RuntimeError("Grip failed to open for release")
        elif self.phase == "withdraw":
            command["speed"] = -.18
            if np.linalg.norm(np.array(pose[:2])-self.retreat_start)>1.5:
                command["speed"] = 0.
                self.transition("verify_placement",now)
        elif self.phase == "verify_placement":
            if elapsed>1.:
                if not fresh or abs(self.payload["position_model"][2]-.45)>.15:
                    raise RuntimeError("Released bale is not visibly supported on the ground")
                c,s = math.cos(pose[2]),math.sin(pose[2])
                x,y = self.payload["position_model"][:2]
                placed = np.array([pose[0]+c*x-s*y,pose[1]+s*x+c*y])
                if np.linalg.norm(placed-self.drop[:2])>.6:
                    raise RuntimeError("Released bale is outside the estimated drop zone")
                self.transition("done",now,dict(payload=self.payload))
        elif self.phase == "done":
            self.finished = True
        return command
