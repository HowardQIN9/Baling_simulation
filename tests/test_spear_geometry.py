"""Independent geometric regressions for the spear experiment's measurements."""
import json
import copy
import math
from pathlib import Path
import sys
import unittest
import tempfile
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'spear_handling'))
from geometry import engagement, segment_overlap, transform, bale_attitude
from generate import validate_config, generate
from validate import tilt_tracking_command


class SpearGeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((Path(__file__).resolve().parents[1]/'spear_handling/config.json').read_text())
        self.bale = [2., 0., self.cfg['bale']['height']/2, 1., 0., 0., 0.]
        # Spear tip is 0.6m past near face; shaft height matches channel center.
        x = 2.-self.cfg['bale']['width']/2+.6-1.1
        self.carrier = [x, 0., .36-.15, 1., 0., 0., 0.]

    def test_all_four_shaft_depths_and_overlap(self):
        m = engagement(self.cfg, self.carrier, self.bale)
        self.assertEqual(len(m['shafts']), 4)
        self.assertTrue(m['all_channels_clear'])
        self.assertAlmostEqual(m['depth_m'], .6)
        self.assertAlmostEqual(m['min_overlap_m'], .6)

    def test_measurements_invariant_under_translated_rotated_world(self):
        # Regression for mixing model-relative carrier with world bale positions.
        yaw = .73
        q = [math.cos(yaw/2), 0., 0., math.sin(yaw/2)]
        world = [8., -4., .9, *q]
        carrier = [*transform(world, self.carrier[:3]), *q]
        bale = [*transform(world, self.bale[:3]), *q]
        m = engagement(self.cfg, carrier, bale)
        self.assertAlmostEqual(m['depth_m'], .6)
        self.assertAlmostEqual(m['min_overlap_m'], .6)
        self.assertTrue(m['all_channels_clear'])

    def test_lateral_misalignment_is_not_successful_insertion(self):
        self.carrier[1] += .12
        self.assertFalse(engagement(self.cfg, self.carrier, self.bale)['all_channels_clear'])

    def test_tilted_shaft_can_hit_channel_ceiling(self):
        angle = -.15
        self.carrier[3:] = [math.cos(angle/2), 0., math.sin(angle/2), 0.]
        self.assertFalse(engagement(self.cfg, self.carrier, self.bale)['all_channels_clear'])

    def test_negative_depth_before_entry(self):
        self.carrier[0] -= .8
        m = engagement(self.cfg, self.carrier, self.bale)
        self.assertAlmostEqual(m['depth_m'], -.2)
        self.assertEqual(m['min_overlap_m'], 0.)

    def test_finite_segment_and_misses(self):
        self.assertAlmostEqual(segment_overlap([-2, 0, 0], [2, 0, 0], [1, 1, 1]), 2.)
        self.assertEqual(segment_overlap([-2, 2, 0], [2, 2, 0], [1, 1, 1]), 0.)
        self.assertEqual(segment_overlap([-2, 0, 0], [-1.5, 0, 0], [1, 1, 1]), 0.)
        self.assertAlmostEqual(segment_overlap([0, 0, 0], [.5, 0, 0], [1, 1, 1]), .5)

    def test_invalid_configuration_is_rejected(self):
        for section, key, value in [('bale', 'mass', -1), ('bale', 'width', float('nan')),
                                    ('attachment', 'count', 1), ('attachment', 'count', 4.5),
                                    ('channel', 'width', .01), ('channel', 'height_from_bottom', .01),
                                    ('friction', 'bale_spear', -.1), ('resistance', 'enabled', 1),
                                    ('test', 'insertion_fraction', 1), ('physics', 'step', 0)]:
            with self.subTest(section=section, key=key, value=value):
                cfg = copy.deepcopy(self.cfg)
                cfg['interaction']['mode']='channels'
                cfg[section][key] = value
                with self.assertRaises(ValueError):
                    validate_config(cfg)

    def test_center_elevation_does_not_prove_whole_bale_clearance(self):
        pitch = math.radians(30)
        pose = [2., 0., .70, math.cos(pitch/2), 0., math.sin(pitch/2), 0.]
        evidence = bale_attitude(self.cfg, pose)
        self.assertAlmostEqual(evidence['tilt_deg'], 30.)
        self.assertLess(evidence['bottom_height_m'], .01)

    def test_material_entry_does_not_require_channels(self):
        self.carrier[1] += .12
        evidence=engagement(self.cfg,self.carrier,self.bale)
        self.assertTrue(evidence['all_material_clear'])
        self.assertFalse(evidence['all_channels_clear'])

    def test_solid_bale_filters_only_native_shaft_pair(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'world.sdf'
            generate(self.cfg,path)
            world=ET.parse(path).getroot().find('world')
            bale=world.find("model[@name='bale_1']/link")
            self.assertEqual(len(bale.findall('collision')),1)
            body=bale.find("collision[@name='bale_body']")
            self.assertIsNotNone(body)
            shaft=world.find("model[@name='simple_tractor']/link[@name='attachment_carrier']/collision[@name='spear_0_collision']")
            def masks(c):
                return tuple(int(c.findtext('surface/contact/'+key)) for key in ('category_bitmask','collide_bitmask'))
            bc,bm=masks(body); sc,sm=masks(shaft)
            self.assertEqual((bc&sm)|(sc&bm),0)
            self.assertNotEqual((bc&bm),0) # bale/bale contact retained
            self.assertNotEqual((sc&65535)|(65535&sm),0) # native ground retained

    def test_channel_baseline_still_generates_compound_body(self):
        cfg=json.loads((Path(__file__).resolve().parents[1]/'spear_handling/config_channels.json').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'world.sdf'
            generate(cfg,path)
            world=ET.parse(path).getroot().find('world')
            self.assertEqual(len(world.find("model[@name='bale_1']/link").findall('collision')),7)
            self.assertEqual(world.findtext("plugin[@name='spear_handling::SpearContact']/material"),'false')

    def test_invalid_material_parameters(self):
        for key,value in [('samples',3),('samples',32.5),('transverse_stiffness',-1),
                          ('bearing_limit_per_m',float('inf')),('cutting_force',True)]:
            cfg=copy.deepcopy(self.cfg)
            cfg['interaction'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):
                validate_config(cfg)

    def test_tilt_compensation_tracks_moving_boom_without_position_lag(self):
        # With zero position error and tilt already moving correctly, the valve
        # must continue the compensating motion rather than brake it to zero.
        self.assertLess(tilt_tracking_command(0.,-.1,.1),0.)
        self.assertGreater(tilt_tracking_command(0.,.1,-.1),0.)
        self.assertEqual(tilt_tracking_command(0.,0.,0.),0.)


if __name__ == '__main__':
    unittest.main()
