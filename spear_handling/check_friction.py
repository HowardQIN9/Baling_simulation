#!/usr/bin/env python3
"""Verify each friction override with sliding blocks, independently of pickup.

Three static support slabs exercise ground/bale/spear collision classification.
All SDF surface coefficients are 0.9; requested pair overrides are varied while
uniform gravity has horizontal/vertical ratio 0.3. Sliding acceleration should
be gx - mu*abs(gz). These fixtures test the backend, not material calibration.
"""
import copy
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as E

from generate import ROOT, el, box, inertia, boxpart, generate


def fixture(cfg, output, coefficients):
    # Reuse the exact plugin parameters generated for the spear experiment.
    generate(cfg, output)
    plugin = copy.deepcopy(E.parse(output).getroot().find("world/plugin[@name='spear_handling::SpearContact']"))
    for key in ('bale_ground', 'bale_spear', 'bale_bale'):
        plugin.find(key).text = str(coefficients[key])
    sdf = E.Element('sdf', version='1.10')
    world = el(sdf, 'world', name='friction_check')
    el(world, 'gravity', '2.943 0 -9.81')
    physics = el(world, 'physics', name='physics', type='ignored')
    el(physics, 'max_step_size', cfg['physics']['step'])
    el(physics, 'real_time_factor', 1)
    el(world, 'plugin', filename='gz-sim-physics-system', name='gz::sim::systems::Physics')
    world.append(plugin)
    b = cfg['bale']
    size = [b['width'], b['length'], b['height']]
    for index, (support_name, collision_name, y) in enumerate([
            ('ground', 'ground_collision', -6),
            ('bale_support', 'support_collision', 0),
            ('tool_support', 'spear_fixture', 6)], 1):
        support = el(world, 'model', name=support_name)
        el(support, 'static', 'true')
        el(support, 'pose', f'0 {y} -.05 0 0 0')
        link = el(support, 'link', name='support_link')
        collision = box(link, collision_name, [20, 4, .1], [0, 0, 0])
        surface = el(el(el(collision, 'surface'), 'friction'), 'ode')
        el(surface, 'mu', .9)
        el(surface, 'mu2', .9)
        model = el(world, 'model', name=f'bale_{index}')
        el(model, 'pose', f'0 {y} {b["height"]/2+.002} 0 0 0')
        link = el(model, 'link', name='bale_link')
        inertia(link, [boxpart(b['mass'], size, [0, 0, 0])])
        collision = box(link, 'probe_collision', size, [0, 0, 0])
        surface = el(el(el(collision, 'surface'), 'friction'), 'ode')
        el(surface, 'mu', .9)
        el(surface, 'mu2', .9)
    tree = E.ElementTree(sdf)
    E.indent(tree)
    tree.write(output, encoding='unicode')


def run_case(cfg, directory, coefficients):
    directory.mkdir(parents=True)
    world = directory/'world.sdf'
    fixture(cfg, world, coefficients)
    os.environ['GZ_PARTITION'] = 'friction_'+uuid.uuid4().hex
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.double_v_pb2 import Double_V
    node, lock = Node(), threading.Lock()
    poses, counts = [], []

    def receive(msg):
        t = msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        with lock:
            poses.append((t, {p.name: p.position.x for p in msg.pose}))

    def contacts(msg):
        with lock:
            counts.append(list(msg.data))

    node.subscribe(Pose_V, '/spear/world_poses', receive)
    node.subscribe(Double_V, '/spear/contact/state', contacts)
    result = dict(requested_mu=coefficients, passed=False, support_surface_mu=.9)
    with (directory/'server.log').open('w') as log:
        server = subprocess.Popen(['gz', 'sim', '-s', '-r', '--physics-engine',
                                   'gz-physics-dartsim-plugin', str(world)],
                                  stdout=log, stderr=log, start_new_session=True)
        try:
            deadline = time.monotonic()+25
            while True:
                with lock:
                    complete = poses and poses[-1][0] >= 1.2 and counts
                if complete:
                    break
                if server.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Friction fixture telemetry unavailable; inspect server.log')
                time.sleep(.02)
            with lock:
                samples, contact_samples = list(poses), list(counts)
            (directory/'poses.json').write_text(json.dumps(samples))
            points = [min(samples, key=lambda p: abs(p[0]-t)) for t in (.3, .6, .9)]
            pairs = []
            # Probe order differs from the telemetry's ground/spear/bale order.
            for index, pair, counter in [(1, 'bale_ground', 1), (2, 'bale_bale', 3), (3, 'bale_spear', 2)]:
                (t0, p0), (t1, p1), (t2, p2) = points
                name = f'bale_{index}'
                acceleration = 2*((p2[name]-p1[name])/(t2-t1)-(p1[name]-p0[name])/(t1-t0))/(t2-t0)
                mu = coefficients[pair]
                expected = max(0., 2.943-mu*9.81)
                displacement = points[-1][1][name]-points[0][1][name]
                count = max(c[counter] for c in contact_samples)
                passed = abs(acceleration-expected) < .15 and count > 0
                if mu > .3:
                    passed = passed and abs(displacement) < .01
                pairs.append(dict(pair=pair, requested_mu=mu, acceleration_m_s2=acceleration,
                                  expected_acceleration_m_s2=expected,
                                  inferred_sliding_mu=(2.943-acceleration)/9.81 if mu < .3 else None,
                                  displacement_m=displacement, callbacks=count, passed=passed))
            result.update(pairs=pairs, passed=all(p['passed'] for p in pairs))
        except Exception as exc:
            result['error'] = str(exc)
        finally:
            if server.poll() is None:
                os.killpg(server.pid, signal.SIGINT)
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait()
            result['server_exit'] = server.returncode
    (directory/'report.json').write_text(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-teardown', action='store_true',
                        help='Compare fixed-iteration shutdown with and without the custom plugin')
    args = parser.parse_args()
    cfg = json.loads(Path(__file__).with_name('config.json').read_text())
    root = ROOT/'runs'/('spear_friction_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    if args.check_teardown:
        root.mkdir(parents=True)
        results = []
        for custom in (False, True):
            tag = 'custom' if custom else 'native'
            path = root/(tag+'.sdf')
            fixture(cfg, path, dict(bale_ground=.1, bale_spear=.2, bale_bale=.6))
            if not custom:
                tree = E.parse(path)
                world = tree.getroot().find('world')
                world.remove(world.find("plugin[@name='spear_handling::SpearContact']"))
                tree.write(path, encoding='unicode')
            env = dict(os.environ, GZ_PARTITION='teardown_'+uuid.uuid4().hex)
            with (root/(tag+'.log')).open('w') as log:
                server = subprocess.Popen(['gz', 'sim', '-s', '-r', '--iterations', '1200',
                                           str(path)], stdout=log, stderr=log, env=env,
                                          start_new_session=True)
                forced = False
                try:
                    server.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    forced = True
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait()
            results.append(dict(custom_plugin=custom, forced_cleanup=forced, exit=server.returncode))
        summary = dict(run=str(root), teardown_comparison=results)
        (root/'report.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2), flush=True)
        return 0 if all(r['exit'] == 0 for r in results) else 1
    results = []
    values = [.1, .2, .6]
    for index in range(3):
        coefficients = {key: values[(j+index)%3] for j, key in
                        enumerate(('bale_ground', 'bale_spear', 'bale_bale'))}
        result = run_case(cfg, root/f'case_{index+1}', coefficients)
        results.append(result)
        print(json.dumps(result, indent=2), flush=True)
    summary = dict(run=str(root), passed=all(r['passed'] for r in results), cases=results)
    (root/'report.json').write_text(json.dumps(summary, indent=2))
    print(f'Artifacts: {root}', flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
