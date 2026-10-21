"""Bounded native Transport queues and isolated Gazebo process lifecycle."""
from collections import deque
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

import numpy as np
from gz.msgs10.double_pb2 import Double
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.image_pb2 import Image, RGB_INT8, R_FLOAT32
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node

from .scene import ROOT
from .localization import PlanarEstimator


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9


def pose_dict(pose):
    p, q = pose.position, pose.orientation
    return dict(position=[p.x, p.y, p.z], quaternion_xyzw=[q.x, q.y, q.z, q.w])


def decode_image(msg, kind):
    # Respect row stride; refuse unexpected encodings instead of reinterpreting bytes.
    if kind == "rgb":
        if msg.pixel_format_type != RGB_INT8:
            raise ValueError(f"Expected RGB_INT8, got {msg.pixel_format_type}")
        rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        return rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
    if msg.step < msg.width * 4 or len(msg.data) != msg.height * msg.step:
        raise ValueError("Unexpected depth image stride")
    if msg.pixel_format_type != R_FLOAT32:
        raise ValueError(f"Expected R_FLOAT32, got {msg.pixel_format_type}")
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    return rows[:, :msg.width * 4].copy().view("<f4").reshape(msg.height, msg.width)


class Session:
    def __init__(self, world_path, metadata, wall_timeout=180):
        os.environ["GZ_PARTITION"] = "bale_autonomy_" + uuid.uuid4().hex
        self.partition = os.environ["GZ_PARTITION"]
        self.metadata = metadata
        self.lock = threading.Lock()
        self.images = {name: {kind: deque(maxlen=16) for kind in ("rgb", "depth")}
                       for name in metadata["cameras"]}
        self.poses = deque(maxlen=200)
        self.odometry = deque(maxlen=200)
        self.raw_odometry = deque(maxlen=200)
        self.imu = deque(maxlen=300)
        self.estimator = PlanarEstimator()
        self.sensor_error = None
        self.loader = {}
        self.commands = deque(maxlen=300)
        self.callbacks = []
        self.node = Node()
        for name, camera in metadata["cameras"].items():
            for kind in ("rgb", "depth"):
                def receive(msg, name=name, kind=kind):
                    with self.lock:
                        self.images[name][kind].append((stamp(msg), msg))
                self.callbacks.append(receive)
                self.node.subscribe(Image, camera["topic"] + "/" + kind, receive)
        def receive_poses(msg):
            data = {p.name: pose_dict(p) for p in msg.pose
                    if p.name in ("simple_tractor", "bale_1", "bale_2", "bale_3")}
            with self.lock:
                self.poses.append((stamp(msg), data))
        def receive_odom(msg):
            data = pose_dict(msg.pose)
            data.update(linear=[msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
                        angular=[msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
            with self.lock:
                t = stamp(msg)
                self.raw_odometry.append((t, data))
                if self.imu:
                    ti, imu = min(self.imu, key=lambda item: abs(item[0]-t))
                    if abs(ti-t) < .06 and (not self.odometry or t > self.odometry[-1][0]):
                        try:
                            self.odometry.append((t, self.estimator.update(t, data, imu)))
                        except ValueError as exc:
                            self.sensor_error = str(exc)
        def receive_imu(msg):
            q = msg.orientation
            imu = dict(quaternion_xyzw=[q.x,q.y,q.z,q.w],
                       angular_velocity=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],
                       linear_acceleration=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
            with self.lock:
                self.imu.append((stamp(msg), imu))
        self.callbacks += [receive_poses, receive_odom, receive_imu]
        self.node.subscribe(Pose_V, "/world/tractor_world/pose/info", receive_poses)
        self.node.subscribe(Odometry, "/model/simple_tractor/odometry", receive_odom)
        self.node.subscribe(IMU, "/tractor/imu", receive_imu)
        for axis in ("lift", "tilt", "grip_left", "grip_right"):
            def receive_axis(msg, axis=axis):
                with self.lock:
                    self.loader[axis] = list(msg.data)
            self.callbacks.append(receive_axis)
            self.node.subscribe(Double_V, f"/tractor/loader/{axis}/state", receive_axis)
        self.drive = self.node.advertise("/model/simple_tractor/cmd_vel", Twist)
        self.lift = self.node.advertise("/tractor/loader/lift/valve", Double)
        self.tilt = self.node.advertise("/tractor/loader/tilt/valve", Double)
        self.grip = self.node.advertise("/tractor/loader/grip/valve", Double)
        self.log_path = Path(world_path).parent / "server.log"
        self.log = self.log_path.open("w")
        env = dict(os.environ)
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(ROOT / "build")
        # NVIDIA's device EGL platform is accessible without modifying /dev/dri groups.
        vendor = Path("/usr/share/glvnd/egl_vendor.d/10_nvidia.json")
        if vendor.exists():
            env.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", str(vendor))
        self.server = subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering",
                                        str(world_path)], stdout=self.log, stderr=self.log,
                                       env=env, start_new_session=True)
        self.environment = env
        self.gui = None
        self.gui_log = None
        self.deadline = time.monotonic() + wall_timeout
        self.closed = False
        self.last_saved = {name: -1 for name in metadata["cameras"]}

    def open_gui(self):
        self.gui_log = (self.log_path.parent / "gui.log").open("w")
        config = ROOT / "worlds/autonomy_gui.config"
        if self.metadata["scenario"].get("task") in ("pick_place","stack"):
            text = config.read_text().replace("6 -14 15 0 0.78 1.57","3 -18 23 0 0.85 1.57")
            config = self.log_path.parent / "gui.config"
            config.write_text(text)
        self.gui = subprocess.Popen(["gz","sim","-g","--gui-config",
            str(config)],env=self.environment,
            stdout=self.gui_log,stderr=self.gui_log,start_new_session=True)
        print(f"Gazebo GUI connecting to partition {self.partition}",flush=True)

    def command(self, speed=0.0, yaw_rate=0.0, lift=0.0, tilt=0.0, grip=0.0):
        with self.lock:
            t = self.odometry[-1][0] if self.odometry else 0.
            self.commands.append((t, dict(speed_m_s=float(speed), yaw_rate_rad_s=float(yaw_rate),
                                          lift_valve=float(lift), tilt_valve=float(tilt), grip_valve=float(grip))))
        msg = Twist()
        msg.linear.x, msg.angular.z = float(speed), float(yaw_rate)
        self.drive.publish(msg)
        self.lift.publish(Double(data=float(lift)))
        self.tilt.publish(Double(data=float(tilt)))
        self.grip.publish(Double(data=float(grip)))

    def state(self):
        with self.lock:
            return dict(poses=self.poses[-1] if self.poses else None,
                        odometry=self.odometry[-1] if self.odometry else None,
                        imu=self.imu[-1] if self.imu else None,
                        loader=dict(self.loader))

    def pairs(self):
        """Nearest RGB/depth within 5ms; state within 60ms, with explicit offsets."""
        result = []
        with self.lock:
            for name, streams in self.images.items():
                if not streams["rgb"] or not streams["depth"]:
                    continue
                # Process oldest unseen pair; queues cap producer/consumer latency.
                for t, rgb in streams["rgb"]:
                    if t <= self.last_saved[name]:
                        continue
                    td, depth = min(streams["depth"], key=lambda p: abs(p[0] - t))
                    if abs(td - t) > .005 or not self.odometry:
                        continue
                    tp, poses = min(self.poses, key=lambda p: abs(p[0] - t)) if self.poses else (t,{})
                    to, odom = min(self.odometry, key=lambda p: abs(p[0] - t))
                    tr, raw = min(self.raw_odometry, key=lambda p: abs(p[0]-t))
                    ti, imu = min(self.imu, key=lambda p: abs(p[0]-t))
                    if abs(tp-t) > .06 or abs(to-t) > .06:
                        continue
                    self.last_saved[name] = t
                    result.append(dict(camera=name, stamp=t, rgb_message=rgb,
                        depth_message=depth, depth_offset_s=td-t, pose_offset_s=tp-t,
                        odom_offset_s=to-t, imu_offset_s=ti-t, ground_truth=poses,
                        odometry=odom, raw_wheel_odometry=raw, imu=imu,
                        loader=dict(self.loader), command=next((command for tc,command in
                            reversed(self.commands) if tc <= t), None)))
                    break
        return result

    def check(self):
        if self.gui is not None and self.gui.poll() is not None:
            raise RuntimeError("Gazebo GUI closed or failed; inspect " +
                               str(self.log_path.parent / "gui.log"))
        if self.sensor_error:
            raise RuntimeError(self.sensor_error)
        if self.server.poll() is not None:
            raise RuntimeError(f"Gazebo exited ({self.server.returncode}):\n" +
                               self.log_path.read_text()[-2500:])
        if time.monotonic() > self.deadline:
            raise TimeoutError("Trial wall-time budget exceeded; inspect server.log")

    def close(self):
        if self.closed:
            return
        self.closed = True
        for _ in range(3):
            self.command()
            time.sleep(.03)
        if self.gui is not None and self.gui.poll() is None:
            try:
                os.killpg(self.gui.pid,signal.SIGINT)
                self.gui.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.gui.pid,signal.SIGKILL)
                self.gui.wait(timeout=3)
            except ProcessLookupError:
                pass
        if self.gui_log is not None:
            self.gui_log.close()
        if self.server.poll() is None:
            try:
                os.killpg(self.server.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.server.wait(timeout=4)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.server.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.server.wait(timeout=3)
        self.log.close()
        return self.server.returncode
