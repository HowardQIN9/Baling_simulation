"""Three-layer mission helpers; only camera geometry and measured joints enter control."""
import json
import math

import cv2
import numpy as np

from .manipulation import (CARRY_LIFT, PickPlace, carrier_position, carrier_rotation,
                           valve_for_speed, LOADED_FRONT, jaws_loaded)
from .perception import cloud, yaw_of, wrap_axis
from .planning import Follower, obstacle_polygons, plan, valid_pose, wrap

BALE_HEIGHT = .9
STACK_ORDER = (1, 2, 3)


def world_payload(payload, pose):
    c,s = math.cos(pose[2]),math.sin(pose[2])
    x,y,z = payload["position_model"]
    return np.array([pose[0]+c*x-s*y,pose[1]+s*x+c*y,z])


def placement_angles(height, reference):
    """Respect existing boom and tilt stops; solve for the measured load center."""
    def center(q):
        tilt = -min(q,.65)
        return carrier_position(q)+carrier_rotation(q,tilt) @ reference
    if height > center(.78)[2]:
        raise ValueError("Requested stacking height exceeds the verified upright reach")
    low,high = 0.,.78
    for _ in range(40):
        mid = (low+high)/2
        if center(mid)[2]<height:
            low = mid
        else:
            high = mid
    q = (low+high)/2
    return q,-min(q,.65),center(q)


def support_surface(rgb,depth,camera,odom,drop,level):
    """Fit the visible top of the existing stack near its estimated map position."""
    xyz = cloud(depth,camera,odom["quaternion_xyzw"])
    yaw = yaw_of(odom)
    c,s = math.cos(yaw),math.sin(yaw)
    wx = odom["position"][0]+c*xyz[...,0]-s*xyz[...,1]
    wy = odom["position"][1]+s*xyz[...,0]+c*xyz[...,1]
    hsv = cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    mask = np.isfinite(depth) & (depth>.1)
    mask &= ((wx-drop[0])**2+(wy-drop[1])**2<1.7**2)
    mask &= (np.abs(xyz[...,2]-level*BALE_HEIGHT)<.16)
    mask &= (hsv[...,0]>=8) & (hsv[...,0]<=48) & (hsv[...,1]>25)
    if np.count_nonzero(mask)<20:
        return None
    heights = xyz[...,2][mask]
    top = float(np.percentile(heights,95))
    mask &= xyz[...,2]>top-.07
    points = np.stack([wx[mask],wy[mask]],axis=1).astype(np.float32)
    if len(points)<15:
        return None
    rect = cv2.minAreaRect(points)
    corners = cv2.boxPoints(rect)
    edges = np.roll(corners,-1,axis=0)-corners
    lengths = np.linalg.norm(edges,axis=1)
    if not (2.05<max(lengths)<2.65 and .95<min(lengths)<1.4):
        return None
    edge = edges[np.argmax(lengths)]
    return dict(position_world=[*map(float,rect[0]),top],
                yaw_world=wrap_axis(math.atan2(edge[1],edge[0])),top_height_m=top)


class StackPickPlace(PickPlace):
    """Low carry to standoff, observe support, raise, insert, lower and withdraw."""
    def __init__(self,*args,level=0,**kwargs):
        super().__init__(*args,**kwargs)
        self.level = level
        # Keep the known ground-place cycle at its validated speed; stacked
        # bales use a faster field transit and slower final insertion.
        self.travel_speed_limit = .32 if level else .25
        self.height = .45+BALE_HEIGHT*level
        self.surface = None
        self.surface_stamp = -1e9
        self.placed = None
        self.stack_polygons = obstacle_polygons([dict(position_world=self.drop[:2],yaw_world=self.drop[2])]) if level else []
        # During transit the stack is a full-height obstacle, like all other bales.
        self.obstacles = [p for p in self.obstacles if np.linalg.norm(p.mean(axis=0)-self.drop[:2])>1]
        self.travel_obstacles = self.obstacles+self.stack_polygons
        self.raise_q = None
        self.last_payload_stamp = None
        self.ground_insert_started = False

    def observe(self,observation,stamp):
        if observation is None:
            return
        if self.payload is not None and self.last_payload_stamp is not None:
            dt = max(.001,stamp-self.last_payload_stamp)
            jump = np.linalg.norm(np.array(observation["position_model"])-self.payload["position_model"])
            # Multiple cameras can yield intermittent partial-box fits. Reject
            # physically impossible frame-to-frame jumps; persistent disagreement
            # still triggers the stale-load stop.
            if jump > .20 + .12*dt:
                return
        self.payload = dict(observation)
        self.payload_stamp = stamp
        self.last_payload_stamp = stamp

    def observe_surface(self,surface,stamp):
        if surface is not None:
            if self.surface is None or stamp-self.surface_stamp>2:
                self.surface = dict(surface)
            else:
                weight=.25
                old=self.surface
                position=(1-weight)*np.array(old["position_world"])+weight*np.array(surface["position_world"])
                yaw=wrap_axis(old["yaw_world"]+weight*wrap_axis(surface["yaw_world"]-old["yaw_world"]))
                self.surface=dict(surface,position_world=position.tolist(),yaw_world=yaw,
                                  top_height_m=(1-weight)*old["top_height_m"]+weight*surface["top_height_m"])
            self.surface_stamp = stamp

    def stack_route(self,pose,goal):
        """Plan a loaded leg to the west-side waypoint; replan from there after stopping."""
        # Keep the long loaded front implement inside the west field boundary
        # while the tractor reverses toward this staging point.
        waypoint = (-11.,-6.,0.)
        first,info_a = plan(pose,waypoint,self.travel_obstacles,timeout=25,
                            front_extent=LOADED_FRONT)
        self.follower = Follower(first,final_position_tolerance=.15)
        self.insertion_staging_goal = goal
        self.plans += 1
        (self.directory/f"manipulation_plan_{self.plans}.json").write_text(json.dumps(dict(
            phase="loaded_stack_waypoint",start=pose,waypoint=waypoint,goal=goal,
            path=first,planner=info_a,front_extent_m=LOADED_FRONT),indent=2))
        if self.visual:
            self.visual.set_plan(first,waypoint)

    def check_carried(self,now,loader):
        q,tilt=loader["lift"][0],loader["tilt"][0]
        fresh = self.payload is not None and now-self.payload_stamp<.65
        if not fresh:
            # The open hydraulic frame hides the bale for part of the route.
            # Maintain a short dead-reckoned carry only while both loaded jaws
            # and the raised lift encoder still confirm the grasp.
            if now-self.payload_stamp>25:
                raise RuntimeError("Camera view did not recover during loaded transport")
            if jaws_loaded(loader) and q>CARRY_LIFT-.12:
                return True
            return False
        expected=carrier_position(q)+carrier_rotation(q,tilt)@self.grip_reference
        slip=np.linalg.norm(np.array(self.payload["position_model"])-expected)
        if slip>.65 or self.payload["position_model"][2]<self.lift_start_height+.2:
            raise RuntimeError(f"Carried bale slipped or fell (relative error {slip:.2f}m)")
        return True

    def step(self,now,pose,odom,loader,ground_target):
        q,qd = loader["lift"][:2]
        tilt,td = loader["tilt"][:2]
        elapsed = now-(self.phase_started if self.phase_started is not None else now)
        fresh = self.payload is not None and now-self.payload_stamp<.65
        command = dict(speed=0.,yaw_rate=0.,lift=0.,tilt=0.,grip=0.)
        desired_tilt = -min(q,.65)
        feedforward = -qd if q<.65 else 0.
        command["tilt"] = valve_for_speed("tilt",np.clip(
            feedforward+1.2*(desired_tilt-tilt)-.35*(qd+td),-.16,.16))
        if self.level and self.phase=="transport_plan":
            self.place_q,_,offset = placement_angles(self.height,self.grip_reference)
            heading = wrap(self.drop[2]-self.grip_yaw)
            c,s = math.cos(heading),math.sin(heading)
            self.insertion_goal = (self.drop[0]-c*offset[0]+s*offset[1],
                                   self.drop[1]-s*offset[0]-c*offset[1],heading)
            goal = (self.insertion_goal[0]-3.4*c,self.insertion_goal[1]-3.4*s,heading)
            self.stack_route(pose,goal)
            self.transition("transport",now)
            return command
        if self.level and self.phase in ("transport","transport_replan","transport_to_stack"):
            if self.phase=="transport_replan":
                original=self.obstacles
                self.obstacles=self.travel_obstacles
                try:
                    self.path(pose,self.insertion_staging_goal,LOADED_FRONT)
                finally:
                    self.obstacles=original
                self.transition("transport_to_stack",now)
                return command
            if not self.check_carried(now,loader):
                return command
            error=CARRY_LIFT-q
            command["lift"]=valve_for_speed("lift",np.clip(1.5*error,-.07,.07))
            remaining=float(np.linalg.norm(np.array(pose[:2])-self.follower.path[-1,:2]))
            cruise=.32 if self.phase=="transport" else .22
            limit=min(cruise,.12+.15*min(1.,remaining/2.))
            v,w,done=self.follow(pose,odom["linear"][0],limit,LOADED_FRONT)
            command.update(speed=v,yaw_rate=w)
            if done:
                if self.phase=="transport":
                    self.transition("transport_replan",now)
                    command.update(speed=0.,yaw_rate=0.)
                else:
                    self.transition("lower",now)
            if elapsed>180:
                raise TimeoutError("Loaded stack transport timeout")
            return command
        if self.level and self.phase=="lower":
            # Base controller has reached the standoff with the load still low.
            if self.raise_q is None:
                if self.surface is None or now-self.surface_stamp>2:
                    if elapsed>5:
                        raise RuntimeError("No fresh camera verification of stack support surface")
                    return command
                self.drop[:2] = self.surface["position_world"][:2]
                self.drop[2] = self.surface["yaw_world"]
                self.height = self.surface["top_height_m"]+.45
                self.raise_q,_,_ = placement_angles(self.height+.20,self.grip_reference)
                self.place_q,_,_ = placement_angles(self.height,self.grip_reference)
                self.transition("raise_for_stack",now,dict(support=self.surface,center_height=self.height))
        if self.phase in ("raise_for_stack","insert","stack_lower"):
            # Once the load is raised, the side cameras see only a clipped top
            # face. Fuse their visibility check with the calibrated jaw-to-bale
            # transform and hydraulic encoders for the load center.
            model_payload = dict(self.payload) if self.payload is not None else None
            if model_payload is not None and self.grip_reference is not None:
                model_payload["position_model"] = (
                    carrier_position(q)+carrier_rotation(q,tilt)@self.grip_reference).tolist()
                model_payload["yaw_model"] = self.grip_yaw
            if not fresh:
                if self.phase=="raise_for_stack":
                    command["lift"] = valve_for_speed("lift",np.clip(
                        1.2*(self.raise_q-q)-.35*qd,-.035,.045))
                    if now-self.payload_stamp>8 and elapsed>35:
                        raise RuntimeError("Camera view did not recover after raising the bale")
                    return command
                if self.phase=="insert":
                    # The raised load can disappear behind the boom during
                    # insertion. Continue from the calibrated jaw transform
                    # while both jaws still confirm contact; release remains
                    # blocked on a fresh post-release RGB-D observation.
                    command["lift"] = valve_for_speed("lift",np.clip(
                        1.2*(self.raise_q-q)-.35*qd,-.025,.025))
                    if jaws_loaded(loader):
                        pass
                    elif now-self.payload_stamp>8:
                        raise RuntimeError("Lost jaw contact during camera-blind stack insertion")
                    else:
                        return command
                else:
                    if now-self.payload_stamp>6:
                        raise RuntimeError("Lost RGB-D observation during precision insertion")
                    return command
            if self.phase=="raise_for_stack":
                if self.payload["position_model"][2]<self.lift_start_height+.2:
                    raise RuntimeError("Load height dropped while raising over the stack")
            if self.phase=="raise_for_stack":
                command["lift"] = valve_for_speed("lift",np.clip(1.2*(self.raise_q-q),-.035,.045))
                if abs(q-self.raise_q)<.03 and abs(qd)<.06 and abs(tilt-desired_tilt)<.07:
                    self.stable_since = now if self.stable_since is None else self.stable_since
                    if now-self.stable_since>.6:
                        # A fresh relative load observation corrects accumulated odometry error.
                        payload = world_payload(self.payload,pose)
                        heading = wrap(self.drop[2]-self.payload["yaw_model"])
                        self.transition("insert",now,dict(goal_world=self.drop[:2].tolist(),
                                                           heading=heading,load_world=payload.tolist()))
                else:
                    self.stable_since = None
                if elapsed>30:
                    raise RuntimeError("Could not reach and stabilize stack clearance height")
            elif self.phase=="insert":
                command["lift"] = valve_for_speed("lift",np.clip(1.2*(self.raise_q-q),-.03,.03))
                payload = world_payload(model_payload,pose)
                heading = wrap(self.drop[2]-model_payload["yaw_model"])
                c,s = math.cos(heading),math.sin(heading)
                px,py = model_payload["position_model"][:2]
                goal = (self.drop[0]-c*px+s*py,
                        self.drop[1]-s*px-c*py,heading)
                dx,dy = goal[0]-pose[0],goal[1]-pose[1]
                cy,sy = math.cos(pose[2]),math.sin(pose[2])
                longitudinal = cy*dx+sy*dy
                lateral = -sy*dx+cy*dy
                heading_error = wrap(heading-pose[2])
                if np.linalg.norm(payload[:2]-self.drop[:2])<.30 and abs(heading_error)<.06:
                    command.update(speed=0.,yaw_rate=0.)
                    self.transition("stack_lower",now)
                else:
                    direction = (1. if longitudinal>=0 else -1.) if abs(longitudinal)>.06 else 1.
                    command["speed"] = direction*min(.10,max(.06,.6*abs(longitudinal)))
                    lookahead = max(.25,math.hypot(longitudinal,lateral))
                    alpha = math.atan2(direction*lateral,abs(longitudinal)+.12)
                    curvature = np.clip(2*math.sin(alpha)/lookahead+1.2*heading_error,-.20,.20)
                    command["yaw_rate"] = float(command["speed"]*curvature)
                if elapsed>65:
                    raise RuntimeError("Stack insertion timeout")
            else:
                lift_error = self.place_q-q
                lift_speed = np.clip(1.1*lift_error,-.025,.025)
                command["lift"] = valve_for_speed("lift",lift_speed)
                if abs(lift_error)>.012 and command["lift"]==0.:
                    command["lift"] = float(np.sign(lift_error)*.06)
                measured = model_payload["position_model"][2]
                if abs(measured-self.height)<.10 and abs(qd)<.025 and abs(tilt-desired_tilt)<.035:
                    self.stable_since = now if self.stable_since is None else self.stable_since
                    if now-self.stable_since>1.2:
                        command["lift"] = 0.
                        self.transition("release",now,dict(payload=self.payload,support_height=self.height-.45))
                else:
                    self.stable_since = None
                if elapsed>20:
                    raise RuntimeError("Could not verify bale height above stack before opening")
            return command
        if self.phase=="withdraw":
            command["speed"] = -.18
            # Leave the open pads elevated until completely clear of the stack.
            if np.linalg.norm(np.array(pose[:2])-self.retreat_start)>3.4:
                command["speed"] = 0.
                self.transition("verify_placement",now)
            return command
        if self.phase=="verify_placement":
            if elapsed>2.:
                if not fresh or abs(self.payload["position_model"][2]-self.height)>.15:
                    raise RuntimeError("Released bale is not visibly supported at requested layer")
                self.placed = world_payload(self.payload,pose)
                tolerance = .40 if self.level else .65
                if np.linalg.norm(self.placed[:2]-self.drop[:2])>tolerance:
                    raise RuntimeError("Released bale is outside stack alignment tolerance")
                self.transition("done",now,dict(payload=self.payload,placed=self.placed.tolist()))
            return command
        # Keep the stack in all transit collision checks, remove it only for raised insertion.
        original = self.obstacles
        self.obstacles = self.travel_obstacles
        try:
            base = super().step(now,pose,odom,loader,ground_target)
        finally:
            self.obstacles = original
        if self.level:
            base["tilt"] = command["tilt"]
        return base
