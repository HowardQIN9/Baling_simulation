"""Analytic geometry and failure-path checks independent of Gazebo labels."""
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from scipy.spatial.transform import Rotation
from gz.msgs10.image_pb2 import Image, RGB_INT8, R_FLOAT32

from bale_autonomy.perception import (TargetTracker, detect, to_world,
                                      wrap_axis)
from bale_autonomy.planning import (MAX_CURVATURE, connector, integrate, plan,
                                    polygon_gap, rectangle, valid_pose)
from bale_autonomy.scene import camera_config, make_world, scenario
from bale_autonomy.transport import decode_image
from bale_autonomy.localization import PlanarEstimator
from bale_autonomy.recording import Recorder
from bale_autonomy.manipulation import payload_observation, jaws_loaded, carrier_position, carrier_rotation, PickPlace, LOADED_FRONT
from bale_autonomy.stacking import placement_angles, support_surface, StackPickPlace


def analytic_image(camera, center, yaw, body_rpy=None):
    """Render a known box and plane via ray/slab intersection (no vision code)."""
    v, u = np.indices((camera["height"], camera["width"]))
    rays = np.stack([np.ones_like(u), -(u-camera["cx"])/camera["fx"],
                     -(v-camera["cy"])/camera["fy"]], axis=-1)
    rot = Rotation.from_euler("xyz", camera["pose"][3:]).as_matrix()
    rays = rays @ rot.T
    origin = np.array(camera["pose"][:3])
    if body_rpy is not None:
        body_rotation = Rotation.from_euler("xyz",body_rpy).as_matrix()
        rays = rays @ body_rotation.T
        origin = origin @ body_rotation.T
    box_rot = Rotation.from_euler("z", yaw).as_matrix()
    local_origin = (origin-center) @ box_rot
    local_rays = rays @ box_rot
    with np.errstate(divide="ignore", invalid="ignore"):
        a = (-np.array([1.2,.6,.45])-local_origin)/local_rays
        b = (np.array([1.2,.6,.45])-local_origin)/local_rays
        entry = np.minimum(a,b).max(axis=-1)
        exit = np.maximum(a,b).min(axis=-1)
        ground = -origin[2]/rays[...,2]
    depth = np.where(ground>0, ground, np.inf)
    hit = (entry>0) & (exit>=entry) & (entry<depth)
    depth[hit] = entry[hit]
    rgb = np.empty((*u.shape,3), np.uint8)
    rgb[:] = [105,95,60]  # ground deliberately similar to bale
    rgb[hit] = [140,115,60]
    return rgb, depth.astype(np.float32)


class PerceptionTests(unittest.TestCase):
    def test_select_left_bale_without_switching_to_center_bale(self):
        tracker = TargetTracker(initial_region="left")
        center = dict(position_model=[12,0,.45],position_world=[12,0,.45],yaw_world=0.,
                      position_std_m=.05)
        left = dict(center,position_model=[12,6,.45],position_world=[12,6,.45],yaw_world=.5)
        selected = tracker.update([center,left],1,"left")
        self.assertEqual(selected["position_world"],[12,6,.45])
        self.assertIsNone(tracker.update([center],2,"front"))
        self.assertEqual(tracker.target["position_world"],[12,6,.45])

    def test_roll_pitch_compensation(self):
        camera = camera_config()["right"]
        center = np.array([11.,1.,.45])
        attitude = Rotation.from_euler("xyz",[.07,-.025,.25]).as_quat().tolist()
        rgb,depth = analytic_image(camera,center,.1,[.07,-.025,.25])
        detections = detect(rgb,depth,camera,attitude)
        self.assertTrue(detections)
        world = to_world(detections[0],dict(position=[0,0,0],quaternion_xyzw=attitude))
        self.assertLess(np.linalg.norm(np.array(world["position_world"])-center),.15)
        self.assertLess(abs(wrap_axis(world["yaw_world"]-.1)),math.radians(3))

    def test_tracked_bale_can_move_outside_initial_forward_gate(self):
        tracker = TargetTracker()
        d = dict(position_model=[12,0,.45],position_world=[12,0,.45],yaw_world=0.,
                 position_std_m=.05)
        tracker.update([d],1,"front")
        d = dict(d,position_model=[10,-6,.45])
        self.assertIsNotNone(tracker.update([d],2,"right"))
        self.assertEqual(tracker.target["last_seen"],2)

    def test_box_pose_from_two_viewpoints_and_similar_ground(self):
        for camera in camera_config().values():
            for yaw in (-.5, 0, .5):
                with self.subTest(camera=camera["frame"], yaw=yaw):
                    center = np.array([11., .6, .45])
                    rgb, depth = analytic_image(camera, center, yaw)
                    found = detect(rgb, depth, camera)
                    self.assertTrue(found)
                    self.assertLess(np.linalg.norm(np.array(found[0]["position_model"])-center), .15)
                    self.assertLess(abs(wrap_axis(found[0]["yaw_model"]-yaw)), math.radians(3))

    def test_invalid_depth_cannot_create_target(self):
        c = camera_config()["front"]
        rgb = np.full((240,320,3), [140,115,60], np.uint8)
        self.assertEqual(detect(rgb, np.full((240,320), np.nan), c), [])

    def test_world_transform_and_outlier_gate(self):
        d = dict(position_model=[10.,0.,.45], yaw_model=.1,
                 position_std_m=.05, yaw_std_rad=.04)
        odom = dict(position=[2,3,0], quaternion_xyzw=[0,0,math.sqrt(.5),math.sqrt(.5)])
        candidate = to_world(d, odom)
        np.testing.assert_allclose(candidate["position_world"], [2,13,.45], atol=1e-6)
        tracker = TargetTracker()
        tracker.update([candidate], 1, "front")
        outlier = dict(candidate, position_world=[20,30,.45])
        self.assertIsNone(tracker.update([outlier], 2, "right"))
        self.assertEqual(tracker.target["last_seen"], 1)

    def test_row_stride_and_encoding_validation(self):
        rgb = Image(width=2,height=1,step=8,pixel_format_type=RGB_INT8,
                    data=bytes([1,2,3,4,5,6,0,0]))
        self.assertEqual(decode_image(rgb,"rgb").tolist(), [[[1,2,3],[4,5,6]]])
        depth = Image(width=2,height=1,step=12,pixel_format_type=R_FLOAT32,
                      data=np.array([1,2,99],"<f4").tobytes())
        np.testing.assert_array_equal(decode_image(depth,"depth"), [[1,2]])
        with self.assertRaises(ValueError):
            decode_image(rgb,"depth")


class PlanningTests(unittest.TestCase):
    def test_model_origin_offset_during_turn(self):
        from bale_autonomy.localization import REAR_AXLE_OFFSET
        estimator = PlanarEstimator()
        raw = dict(position=[0,0,0],linear=[5,0,0],angular=[0,0,1],quaternion_xyzw=[0,0,0,1])
        for t in np.linspace(0,1,101):
            imu = dict(quaternion_xyzw=Rotation.from_euler("z",t).as_quat().tolist())
            estimate = estimator.update(float(t),raw,imu)
        expected = [5*math.sin(1)+REAR_AXLE_OFFSET*(math.cos(1)-1),
                    5*(1-math.cos(1))+REAR_AXLE_OFFSET*math.sin(1),0]
        np.testing.assert_allclose(estimate["position"],expected,atol=1e-4)

    def test_wheel_imu_estimator_uses_measured_heading(self):
        estimator = PlanarEstimator()
        raw = dict(position=[99,99,0],linear=[1,0,0],angular=[0,0,0],quaternion_xyzw=[0,0,0,1])
        imu = dict(quaternion_xyzw=[0,0,math.sqrt(.5),math.sqrt(.5)])
        for t in np.linspace(0,1,11):
            estimate = estimator.update(float(t),raw,imu)
        np.testing.assert_allclose(estimate["position"],[0,1,0],atol=1e-9)
        with self.assertRaises(ValueError):
            estimator.update(1,raw,imu)

    def test_curvature_bounded_connector_and_collision(self):
        path = connector((0,0,0), (6,.3,.08), 1, [])
        self.assertIsNotNone(path)
        self.assertTrue(all(valid_pose(p[:3],[]) for p in path))
        obstacle = rectangle(6,0,0,1,-1,1)
        self.assertIsNone(connector((0,0,0), (6,.3,.08), 1, [obstacle]))

    def test_infeasible_goal_is_rejected(self):
        with self.assertRaises(ValueError):
            plan((0,0,0), (20,0,0), [])

    def test_ackermann_arc_and_reverse(self):
        p = integrate((0,0,0), 2, MAX_CURVATURE)
        self.assertAlmostEqual(p[2], 2*MAX_CURVATURE)
        back = integrate(p, -2, MAX_CURVATURE)
        np.testing.assert_allclose(back, [0,0,0], atol=1e-9)
        a = rectangle(0,0,0,1,-1,1)
        self.assertGreater(polygon_gap(a, rectangle(3,0,0,1,-1,1)), .9)
        self.assertLess(polygon_gap(a, rectangle(.5,0,.2,1,-1,1)), 0)


class RecordingTests(unittest.TestCase):
    def test_generated_world_has_replayable_calibration_and_sources(self):
        import hashlib
        import xml.etree.ElementTree as ET
        with tempfile.TemporaryDirectory() as tmp:
            path, metadata = make_world(tmp,scenario(7))
            world = ET.parse(path).getroot().find("world")
            sensors = world.findall("model[@name='simple_tractor']/link[@name='chassis']/sensor")
            self.assertEqual(len(sensors),5)  # two RGB/depth pairs plus IMU
            for relative,digest in metadata["source_sha256"].items():
                snapshot = Path(tmp)/"source"/relative
                self.assertEqual(hashlib.sha256(snapshot.read_bytes()).hexdigest(),digest)
            self.assertEqual(metadata["cameras"]["front"]["width"],320)

    def test_storage_guard_preserves_existing_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            existing = Path(tmp)/"existing.txt"
            existing.write_text("keep this data")
            recorder = Recorder(tmp,tmp,budget_gb=0)
            try:
                with self.assertRaisesRegex(RuntimeError,"storage budget"):
                    recorder.save(dict(camera="front",stamp=1.))
            finally:
                recorder.close()
            self.assertEqual(existing.read_text(),"keep this data")


class ManipulationTests(unittest.TestCase):
    def test_tool_rotation_is_accounted_for_in_load_position(self):
        reference = np.array([1.5,0,.25])
        q,tilt = .25,-.40
        measurement = carrier_position(q)+carrier_rotation(q,tilt) @ reference
        restored = carrier_rotation(q,tilt).T @ (measurement-carrier_position(q))
        np.testing.assert_allclose(restored,reference,atol=1e-9)

    def test_loaded_collision_check_includes_payload_overhang(self):
        obstacle = rectangle(6.8,0,0,1,-1,.8)
        self.assertTrue(valid_pose((0,0,0),[obstacle]))
        self.assertFalse(valid_pose((0,0,0),[obstacle],front_extent=LOADED_FRONT))

    def test_empty_clamp_times_out_instead_of_lifting(self):
        target = dict(position_world=[8.5,0,.45],yaw_world=0.)
        loader = dict(lift=[0,0,0,0],tilt=[0,0,0,0],
                      grip_left=[.7,0,0,12500],grip_right=[.7,0,0,12500])
        with tempfile.TemporaryDirectory() as tmp:
            cycle = PickPlace(target,[],[0,-6,0],Path(tmp))
            cycle.phase = "clamp"
            cycle.step(0,(0,0,0),dict(linear=[0,0,0]),loader,None)
            with self.assertRaisesRegex(RuntimeError,"both sides"):
                cycle.step(10,(0,0,0),dict(linear=[0,0,0]),loader,None)
            self.assertFalse(cycle.pickup_verified)

    def test_empty_jaw_end_stop_is_not_a_loaded_grip(self):
        state = dict(grip_left=[.35,0,0,10000],grip_right=[.35,0,0,10000])
        self.assertTrue(jaws_loaded(state))
        state["grip_right"] = [.7,0,0,12500]
        self.assertFalse(jaws_loaded(state))
        state["grip_right"] = [.35,0,0,20]
        self.assertFalse(jaws_loaded(state))

    def test_camera_detects_ground_and_elevated_load(self):
        camera = camera_config()["right"]
        for height in (.45,1.1):
            with self.subTest(height=height):
                center = np.array([4.5,0,height])
                rgb,depth = analytic_image(camera,center,0)
                estimate = payload_observation(rgb,depth,camera,[0,0,0,1])
                self.assertIsNotNone(estimate)
                self.assertLess(np.linalg.norm(np.array(estimate["position_model"])-center),.15)

    def test_carry_kinematics_raise_and_extend_the_grip(self):
        initial,raised = carrier_position(0),carrier_position(.25)
        self.assertGreater(raised[2]-initial[2],.6)
        self.assertGreater(raised[0],initial[0])

    def test_three_layer_reach_respects_existing_tilt_stop(self):
        reference = np.array([1.5,0,.25])
        for height in (1.35,2.25,2.45):
            q,tilt,center = placement_angles(height,reference)
            self.assertTrue(0<q<.78)
            self.assertGreaterEqual(tilt,-.65)
            self.assertAlmostEqual(center[2],height,places=8)
        with self.assertRaises(ValueError):
            placement_angles(3.5,reference)

    def test_side_cameras_observe_third_layer_when_front_view_is_below_it(self):
        for name in ("left","right"):
            camera = camera_config()[name]
            center = np.array([4.5,0,2.45])
            rgb,depth = analytic_image(camera,center,0)
            estimate = payload_observation(rgb,depth,camera,[0,0,0,1])
            self.assertIsNotNone(estimate)
            self.assertLess(np.linalg.norm(np.array(estimate["position_model"])-center),.1)

    def test_support_surface_is_measured_at_both_stack_heights(self):
        camera = camera_config()["right"]
        odom = dict(position=[0,0,0],quaternion_xyzw=[0,0,0,1])
        for level in (1,2):
            rgb,depth = analytic_image(camera,np.array([8.,0,.45+.9*(level-1)]),0)
            surface = support_surface(rgb,depth,camera,odom,[8,0,0],level)
            self.assertIsNotNone(surface)
            self.assertAlmostEqual(surface["top_height_m"],level*.9,places=5)
            self.assertIsNone(support_surface(rgb,depth,camera,odom,[2,8,0],level))

    def test_elevated_insertion_waits_for_visible_support(self):
        target = dict(position_world=[12,6,.45],yaw_world=.5)
        loader = dict(lift=[.25,0],tilt=[-.25,0])
        with tempfile.TemporaryDirectory() as tmp:
            cycle = StackPickPlace(target,[],[0,-6,0],Path(tmp),level=1)
            cycle.phase = "lower"
            cycle.phase_started = 0
            command = cycle.step(1,(-8,-6,0),dict(linear=[0,0,0]),loader,None)
            self.assertEqual(command["speed"],0)
            self.assertIsNone(cycle.raise_q)
            with self.assertRaisesRegex(RuntimeError,"support surface"):
                cycle.step(6,(-8,-6,0),dict(linear=[0,0,0]),loader,None)

    def test_stacking_payload_filter_rejects_one_frame_component_jump(self):
        target = dict(position_world=[12,6,.45],yaw_world=.5)
        with tempfile.TemporaryDirectory() as tmp:
            cycle = StackPickPlace(target,[],[0,-6,0],Path(tmp),level=1)
            first = dict(position_model=[4.5,0,1.],yaw_model=0.)
            cycle.observe(first,1.)
            cycle.observe(dict(position_model=[4.,.01,1.02],yaw_model=0.),1.1)
            np.testing.assert_allclose(cycle.payload["position_model"],first["position_model"])
            self.assertEqual(cycle.payload_stamp,1.)
            cycle.observe(dict(position_model=[4.55,0,1.02],yaw_model=0.),1.2)
            self.assertEqual(cycle.payload_stamp,1.2)


if __name__ == "__main__":
    unittest.main()
