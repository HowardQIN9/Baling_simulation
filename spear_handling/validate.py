#!/usr/bin/env python3
"""Truth-guided spear physics experiments with explicit world-frame telemetry."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

from generate import generate, ROOT
from geometry import engagement, bale_attitude


def tilt_tracking_command(error, velocity, lift_velocity):
    """Track a moving level/tilt target through the existing cylinder model."""
    desired=-lift_velocity
    area=math.pi*(.05**2-(.03**2 if desired<0 else 0.))/4
    feedforward=desired*2*area*.18/(45/60000)
    return 2.*error+.6*(desired-velocity)+feedforward


class Telemetry:
    """Callbacks replace whole packets; control takes one locked snapshot."""
    def __init__(self):
        self.lock = threading.Lock()
        self.states = {}
        self.poses = {}
        self.pose_stamp = None
        self.received = 0.

    def state(self, name, msg):
        with self.lock:
            self.states[name] = list(msg.data)

    def pose(self, msg):
        poses = {v.name: [v.position.x, v.position.y, v.position.z,
                         v.orientation.w, v.orientation.x, v.orientation.y, v.orientation.z]
                 for v in msg.pose}
        with self.lock:
            self.poses = poses
            self.pose_stamp = msg.header.stamp.sec + msg.header.stamp.nsec*1e-9
            self.received = time.monotonic()

    def snapshot(self):
        with self.lock:
            return dict(self.states), dict(self.poses), self.pose_stamp, self.received


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['A', 'B', 'C', 'sequence', 'release'], default='A')
    parser.add_argument('--no-release-preload', action='store_true',
                        help='Compare withdrawal without the additional downward preload')
    parser.add_argument('--no-insertion-preload', action='store_true')
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('config.json'))
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--start-delay', type=float, default=0.,
                        help='Seconds to arrange the GUI before starting motion')
    parser.add_argument('--keep-open', action='store_true',
                        help='Keep the GUI open after recording the result')
    args = parser.parse_args()
    if not math.isfinite(args.start_delay) or args.start_delay < 0:
        parser.error('--start-delay must be finite and nonnegative')
    if args.keep_open and not args.gui:
        parser.error('--keep-open requires --gui')
    cfg = json.loads(args.config.read_text())
    material = cfg.get('interaction', {}).get('mode', 'channels') == 'material'
    if args.case == 'B':
        cfg['test']['insertion_fraction'] = .2
    if args.case == 'C':
        cfg['test']['bale_y'] = .12
    run = ROOT/'runs'/('spear_'+time.strftime('%Y%m%d_%H%M%S')+'_'+args.case+'_'+uuid.uuid4().hex[:6])
    run.mkdir(parents=True)
    (run/'config.json').write_text(json.dumps(cfg, indent=2))
    generate(cfg, run/'world.sdf')
    sources = [*Path(__file__).parent.glob('*.py'), *Path(__file__).parent.glob('src/*'),
               Path(__file__).parent/'CMakeLists.txt', ROOT/'worlds/simple_tractor.sdf',
               ROOT/'src/LoaderHydraulics.cc', ROOT/'src/HydraulicModel.hh']
    hashes = {}
    for source in sources:
        relative = source.resolve().relative_to(ROOT)
        data = source.read_bytes()
        saved = run/'source'/relative
        saved.parent.mkdir(parents=True, exist_ok=True)
        saved.write_bytes(data)
        hashes[str(relative)] = hashlib.sha256(data).hexdigest()
    for name in ('libSpearContact.so', 'libBaleLoaderHydraulics.so'):
        binary = ROOT/'spear_handling/build'/name
        hashes[str(binary.relative_to(ROOT))] = hashlib.sha256(binary.read_bytes()).hexdigest()
    (run/'source_sha256.json').write_text(json.dumps(hashes, indent=2))
    os.environ['GZ_PARTITION'] = 'spear_'+uuid.uuid4().hex
    from gz.transport13 import Node
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.twist_pb2 import Twist
    node, telemetry = Node(), Telemetry()
    callbacks = []  # Retain callbacks for the subscription lifetime.
    for name in ['lift', 'tilt', 'contact', 'material_bale_1', 'material_bale_2']:
        callback = lambda m, n=name: telemetry.state(n, m)
        callbacks.append(callback)
        topic = ('/spear/material/'+name.removeprefix('material_') if name.startswith('material_')
                 else '/spear/contact/state' if name == 'contact' else f'/tractor/loader/{name}/state')
        node.subscribe(Double_V, topic, callback)
    node.subscribe(Pose_V, '/spear/world_poses', telemetry.pose)
    pubs = {n: node.advertise(f'/tractor/loader/{n}/valve', Double) for n in ['lift', 'tilt']}
    drive = node.advertise('/model/simple_tractor/cmd_vel', Twist)
    cmd = ['gz', 'sim', '-r', '--physics-engine', 'gz-physics-dartsim-plugin', str(run/'world.sdf')]
    if not args.gui:
        cmd.insert(2, '-s')
    else:
        gui = run/'gui.config'
        gui.write_text((ROOT/'worlds/autonomy_gui.config').read_text().replace(
            '6 -14 15 0 0.78 1.57', '6 -12 8 0 0.48 1.57').replace(
            'Autonomous bale approach', 'Four-spear two-bale demonstration'))
        cmd[2:2] = ['--gui-config', str(gui)]
    server = None
    report = dict(schema_version=2, case=args.case, run=str(run), stages={}, passed=False,
                  interaction_mode='material' if material else 'channels',
                  insertion_preload=material and not args.no_insertion_preload,
                  release_preload=material and not args.no_release_preload,
                  pose_frame='world', partition=os.environ['GZ_PARTITION'],
                  pickup_limits=dict(min_lift_m=.25, min_bottom_height_m=.10,
                                     max_tilt_deg=8., max_tail_height_range_m=.03,
                                     min_overlap_m=.10, max_insertion_push_m=.15))
    states, poses, now = {}, {}, 0.
    entry_angle = cfg['test']['forward_tilt']
    carry_angle = cfg['test'].get('carry_angle', .015)
    target = cfg['bale']['width']*cfg['test']['insertion_fraction']
    report['target_depth_m'] = target
    entry_lift = cfg['test'].get('entry_lift', .03) if material else 0.

    def refresh():
        nonlocal states, poses, now
        states, poses, stamp, received = telemetry.snapshot()
        if server.poll() is not None:
            raise RuntimeError('Gazebo exited; inspect server.log')
        if stamp is None or time.monotonic()-received > 2:
            raise RuntimeError('World-pose telemetry stale for 2 wall seconds')
        if not all(n in poses for n in ['attachment_carrier', 'bale_1', 'bale_2', 'simple_tractor']):
            raise RuntimeError('Incomplete world-pose packet')
        if not all(n in states and len(states[n]) >= 7 for n in ['lift', 'tilt', 'contact']):
            raise RuntimeError('Incomplete actuator/contact telemetry')
        if any(abs(states[n][6 if n != 'contact' else 0]-stamp) > .15
               for n in ['lift', 'tilt', 'contact']):
            raise RuntimeError('Actuator/contact telemetry out of sync with world poses')
        now = stamp
        if material and any(n not in states or abs(states[n][0]-stamp)>.15
                            for n in ('material_bale_1','material_bale_2')):
            raise RuntimeError('Material force telemetry stale or missing')

    def geometry(bale='bale_1'):
        return engagement(cfg, poses['attachment_carrier'], poses[bale])

    def depth(bale='bale_1'):
        return geometry(bale)['depth_m']

    def force_z(bale='bale_1'):
        return states.get('material_'+bale, [0.]*13)[5]

    def insertion_clear(bale='bale_1'):
        return geometry(bale)['all_material_clear' if material else 'all_channels_clear']

    def clip(value, limit):
        return max(-limit, min(limit, value))

    def stop():
        drive.publish(Twist())
        for pub in pubs.values():
            pub.publish(Double(data=0.))

    with (run/'server.log').open('w') as log, (run/'trajectory.jsonl').open('w', buffering=1) as trace:
        def phase(name, duration, lift=0., angle=None, speed=0., until=None, required=False, check=None):
            report['last_phase'] = name
            refresh()
            start, deadline = now, time.monotonic()+max(30, duration*6)
            angle = entry_angle if angle is None else angle
            achieved = False
            samples = []
            selected = 'bale_2' if name in ('engage_lower','lower_preload','insert_lower','lower_stop','lift_both','forward','brake','reverse','final_hold') else 'bale_1'
            while True:
                refresh()
                if time.monotonic() > deadline:
                    raise TimeoutError(f'{name}: wall deadline exceeded')
                if check is not None and not check():
                    raise RuntimeError(f'{name}: continuous load/support check failed')
                if until and until():
                    achieved = True
                    break
                if now-start >= duration:
                    break
                if name in ('engage','insert','engage_lower','insert_lower') and math.dist(poses[selected][:2], insertion_origins[selected]) > .15:
                    raise RuntimeError('Insertion pushed the bale >0.15 m; lift blocked')
                if name == 'withdraw' and math.dist(release_origin,poses['bale_1'][:2])>.15:
                    report['withdrawal_bale_displacement_m']=math.dist(release_origin,poses['bale_1'][:2])
                    raise RuntimeError('Bale followed the withdrawing spears >0.15 m; withdrawal stopped')
                lift_goal=lift() if callable(lift) else lift
                for n, goal in [('lift', lift_goal), ('tilt', angle-states['lift'][0])]:
                    q, qdot = states[n][:2]
                    # Meter motion through the existing inner hydraulic PID.
                    # Damping suppresses outer-loop reversals at 20 Hz feedback.
                    kp, kd, limit = (4., 1., .45) if n == 'lift' else (2., .6, .25)
                    error = goal-q
                    u = clip(kp*error-kd*qdot, limit)
                    if n=='tilt':
                        u=clip(tilt_tracking_command(error,qdot,states['lift'][1]),limit)
                    if n=='lift' and name=='lift_both':
                        u=clip(u,.22)
                    if (abs(error) < .003 and abs(qdot) < .02 and
                            (n!='tilt' or abs(states['lift'][1])<.005)):
                        u = 0.
                    elif abs(u) < .06:
                        u = math.copysign(.06, u) if abs(u) > .01 else 0.
                    pubs[n].publish(Double(data=u))
                velocity = speed() if callable(speed) else speed
                msg = Twist()
                msg.linear.x = velocity
                drive.publish(msg)
                row = dict(phase=name, time=now, pose_frame='world', states=states,
                           poses=poses, target_bale=selected, engagement=geometry(selected), command_speed=velocity,
                           bale_attitude=bale_attitude(cfg, poses[selected]))
                trace.write(json.dumps(row)+'\n')
                samples.append(row)
                time.sleep(.025)
            # Never leave a latched driving command active across stages/errors.
            drive.publish(Twist())
            result = dict(time=now, poses=poses, states=states, depth=depth(selected),
                          target_bale=selected, engagement=geometry(selected), condition_reached=achieved)
            if samples:
                tail = [r for r in samples if r['time'] >= now-1.]
                result['tail_height_range_m'] = max(r['poses'][selected][2] for r in tail)-min(r['poses'][selected][2] for r in tail)
                result['tail_min_bottom_height_m'] = min(r['bale_attitude']['bottom_height_m'] for r in tail)
                result['tail_max_tilt_deg'] = max(r['bale_attitude']['tilt_deg'] for r in tail)
                result['tail_min_overlap_m'] = min(r['engagement']['min_overlap_m'] for r in tail)
                if material:
                    fz=[r['states']['material_'+selected][5] for r in samples]
                    result['spear_force_z_range_N'] = [min(fz),max(fz)]
                    result['final_spear_force_z_N'] = force_z(selected)
            report['stages'][name] = result
            print(name, json.dumps(dict(t=now, depth=depth(selected), z=poses[selected][2],
                                        contact=states['contact'][1:4])), flush=True)
            if required and not achieved:
                raise RuntimeError(f'{name}: required condition not reached; subsequent motion blocked')
            return result

        try:
            print(f'Artifacts: {run}', flush=True)
            server = subprocess.Popen(cmd, stdout=log, stderr=log, start_new_session=True)
            deadline = time.monotonic()+25
            while True:
                packet, objects, stamp, received = telemetry.snapshot()
                needed=['lift','tilt','contact']+(['material_bale_1','material_bale_2'] if material else [])
                if (all(n in packet for n in needed) and len(objects) == 4 and
                        drive.has_connections() and all(p.has_connections() for p in pubs.values())):
                    break
                if server.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('No complete Gazebo telemetry; inspect server.log')
                time.sleep(.05)
            if args.start_delay:
                print(f'Demo starts in {args.start_delay:g} seconds; arrange the Gazebo view.', flush=True)
                deadline = time.monotonic()+args.start_delay
                while time.monotonic() < deadline:
                    refresh()
                    stop()
                    time.sleep(.05)
            phase('settle', 3., lift=entry_lift)
            if abs(states['lift'][0]-entry_lift)>.008 or abs(states['lift'][0]+states['tilt'][0]-entry_angle)>.015:
                raise RuntimeError('Actuator positioning not established; drive motion blocked')
            settled = poses['bale_1'][2]
            insertion_origins = {'bale_1':list(poses['bale_1'][:2])}
            report['settled_height_m'] = settled
            if material and not args.no_insertion_preload and target>.25:
                phase('engage', 12., lift=entry_lift, speed=.12,
                      until=lambda: geometry()['min_overlap_m']>.15, required=True)
                phase('insertion_preload', 5., lift=cfg['test'].get('insertion_lift', .015),
                      until=lambda: force_z() < -cfg['test'].get('insertion_preload_N',600.), required=True)
                entry_lift=states['lift'][0]
            entry_x=poses['simple_tractor'][0]
            slope=math.tan(entry_angle)
            jacobian=2.8*(math.cos(entry_lift-.5)+slope*math.sin(entry_lift-.5))
            def insertion_boom():
                # Follow the embedded shaft direction while driving on level
                # ground. With downward-pointing shafts, horizontal travel
                # otherwise lifts their existing passage and unloads the bale.
                return max(0.,entry_lift+slope*(poses['simple_tractor'][0]-entry_x)/jacobian)
            phase('insert', 20., lift=insertion_boom if material and not args.no_insertion_preload else entry_lift,
                  speed=lambda: clip((target-depth())*.8, .15),
                  until=lambda: abs(target-depth()) < .012, required=True)
            if material: entry_lift=states['lift'][0]
            phase('stop', 1., lift=entry_lift)
            reached = abs(target-depth()) < .025 and geometry()['min_overlap_m'] >= target-.03
            report['insertion_reached'] = reached
            report['insertion_geometry_clear'] = insertion_clear()
            if not reached or not insertion_clear():
                raise RuntimeError('Insertion geometry failed; lift blocked')
            contact_before = states['contact'][2]
            phase('lift', 12., lift=.20, angle=carry_angle)
            held = phase('hold', 4., lift=.20, angle=carry_angle)
            raised = poses['bale_1'][2] > settled+.25
            contact = (held.get('spear_force_z_range_N',[0,0])[1]>100. if material
                       else states['contact'][2] > contact_before)
            retained = (held['tail_min_overlap_m'] > .1 and held['tail_height_range_m'] < .03
                        and held['tail_min_bottom_height_m'] > .10 and held['tail_max_tilt_deg'] < 8.)
            report.update(bale_raised=raised, spear_contact_during_lift=contact,
                          retained=retained, lift_height_m=poses['bale_1'][2]-settled)
            if not (raised and contact and retained):
                raise RuntimeError('Pickup/retention checks failed')
            if args.case in ['A', 'B', 'C']:
                report['passed'] = True
                if args.case != 'A':
                    report['interpretation'] = 'Same measured pickup checks at perturbed insertion/alignment; no required physical failure assumed.'
            else:
                phase('raise_for_stack', 15., lift=.56, angle=0.)
                phase('align_stack', 45., lift=.56, angle=0.,
                      speed=lambda: clip((poses['bale_2'][0]-poses['bale_1'][0])*.6, .2),
                      until=lambda: abs(poses['bale_2'][0]-poses['bale_1'][0]) < .025, required=True)
                phase('place', 12., lift=.365, angle=0.)
                if material and not args.no_release_preload:
                    phase('release_preload', 8., lift=.33, angle=0.,
                          until=lambda: force_z() < -cfg['test'].get('release_preload_N',300.), required=True)
                    withdraw_lift=states['lift'][0]
                elif material:
                    withdraw_lift=states['lift'][0]
                else:
                    phase('unload_spears', 8., lift=.345, angle=0.)
                    withdraw_lift=.345
                report['withdrawal_start_force_z_N']=force_z() if material else None
                release_origin=list(poses['bale_1'][:2])
                phase('withdraw', 18., lift=withdraw_lift, angle=0., speed=-.12,
                      until=lambda: depth() < -.12, required=True)
                phase('stack_settle', 4., lift=withdraw_lift, angle=0.)
                report['withdrawal_bale_displacement_m']=math.dist(release_origin,poses['bale_1'][:2])
                h = cfg['bale']['height']
                def stack_supported():
                    return (abs(poses['bale_1'][2]-poses['bale_2'][2]-h) < .08 and
                            math.dist(poses['bale_1'][:2], poses['bale_2'][:2]) < .15 and
                            all(bale_attitude(cfg,poses[n])['tilt_deg']<8 for n in ('bale_1','bale_2')))
                report['stacked'] = stack_supported()
                if not report['stacked']:
                    raise RuntimeError('Stack not aligned/supported after full withdrawal')
                if args.case == 'release':
                    report['passed'] = True
                else:
                    lower_lift=cfg['test'].get('entry_lift',.03) if material else 0.
                    phase('lower_for_second', 15., lift=lower_lift)
                    insertion_origins['bale_2']=list(poses['bale_2'][:2])
                    if material and not args.no_insertion_preload:
                        phase('engage_lower',12.,lift=lower_lift,speed=.1,
                              until=lambda: geometry('bale_2')['min_overlap_m']>.15,required=True)
                        phase('lower_preload',5.,lift=cfg['test'].get('insertion_lift',.015),
                              until=lambda: force_z('bale_2') < -cfg['test'].get('insertion_preload_N',600.),required=True)
                        lower_lift=states['lift'][0]
                    lower_x=poses['simple_tractor'][0]
                    lower_jacobian=2.8*(math.cos(lower_lift-.5)+slope*math.sin(lower_lift-.5))
                    def lower_boom():
                        return max(0.,lower_lift+slope*(poses['simple_tractor'][0]-lower_x)/lower_jacobian)
                    phase('insert_lower', 25., lift=lower_boom if material and not args.no_insertion_preload else lower_lift,
                          speed=lambda: clip((target-depth('bale_2'))*.8, .12),
                          until=lambda: abs(target-depth('bale_2')) < .012, required=True)
                    if not insertion_clear('bale_2'):
                        raise RuntimeError('Lower-bale insertion misaligned; lift blocked')
                    phase('lower_stop',1.,lift=states['lift'][0])
                    phase('lift_both', 15., lift=.20, angle=carry_angle,check=stack_supported)
                    def both_raised():
                        return (poses['bale_2'][2] > h/2+.25 and stack_supported() and
                                bale_attitude(cfg,poses['bale_2'])['bottom_height_m']>.10 and
                                geometry('bale_2')['min_overlap_m']>.1)
                    report['both_raised'] = both_raised()
                    if not report['both_raised']:
                        raise RuntimeError('Two-bale lift failed')
                    speed = cfg['test']['transport_speed']
                    phase('forward', 5., lift=.20, angle=carry_angle, speed=speed, check=both_raised)
                    phase('brake', 2., lift=.20, angle=carry_angle, check=both_raised)
                    phase('reverse', 5., lift=.20, angle=carry_angle, speed=-speed, check=both_raised)
                    phase('final_hold', 4., lift=.20, angle=carry_angle, check=both_raised)
                    report['passed'] = both_raised()
        except Exception as exc:
            report['error'] = f'{type(exc).__name__}: {exc}'
        finally:
            stop()
            final_states, final_poses, final_stamp, _ = telemetry.snapshot()
            report['final_measurement'] = dict(time=final_stamp, poses=final_poses, states=final_states)
            if args.keep_open and server is not None and server.poll() is None:
                report['awaiting_gui_close'] = True
                (run/'report.json').write_text(json.dumps(report, indent=2))
                print(f"Demo finished: passed={report['passed']}. Result saved. Close Gazebo when finished viewing.", flush=True)
                try:
                    while server.poll() is None:
                        stop()
                        time.sleep(.1)
                except KeyboardInterrupt:
                    pass
                report['awaiting_gui_close'] = False
            if server is not None and server.poll() is None:
                os.killpg(server.pid, signal.SIGINT)
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait()
            report['server_exit'] = server.returncode if server is not None else None
            (run/'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k not in ('stages','final_measurement')}, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
