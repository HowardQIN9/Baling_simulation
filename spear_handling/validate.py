#!/usr/bin/env python3
"""Physical experiments using simulator truth for measurement and feedback."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
from generate import generate, ROOT

def main():
    p=argparse.ArgumentParser(); p.add_argument('--case',choices=['A','B','C','sequence'],default='A'); p.add_argument('--config',type=Path,default=Path(__file__).with_name('config.json')); p.add_argument('--gui',action='store_true'); args=p.parse_args()
    cfg=json.loads(args.config.read_text())
    if args.case=='B': cfg['test']['insertion_fraction']=.2
    if args.case=='C': cfg['test']['bale_y']=.12
    run=ROOT/'runs'/('spear_'+time.strftime('%Y%m%d_%H%M%S')+'_'+args.case); run.mkdir(parents=True)
    (run/'config.json').write_text(json.dumps(cfg,indent=2)); generate(cfg,run/'world.sdf')
    os.environ['GZ_PARTITION']='spear_'+uuid.uuid4().hex
    from gz.transport13 import Node
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.twist_pb2 import Twist
    node=Node(); states={}; poses={}
    for name in ['lift','tilt']:
        node.subscribe(Double_V,f'/tractor/loader/{name}/state',lambda m,n=name:states.update({n:list(m.data)}))
    node.subscribe(Double_V,'/spear/contact/state',lambda m:states.update(contact=list(m.data)))
    def receive(m):
        for v in m.pose:
            if v.name in ['bale_1','bale_2','attachment_carrier','simple_tractor']:
                poses[v.name]=[v.position.x,v.position.y,v.position.z,v.orientation.w,v.orientation.x,v.orientation.y,v.orientation.z]
    node.subscribe(Pose_V,'/world/tractor_world/pose/info',receive)
    pubs={n:node.advertise(f'/tractor/loader/{n}/valve',Double) for n in ['lift','tilt']}
    drive=node.advertise('/model/simple_tractor/cmd_vel',Twist)
    log=(run/'server.log').open('w'); trace=(run/'trajectory.jsonl').open('w')
    cmd=['gz','sim','-r','--physics-engine','gz-physics-dartsim-plugin',str(run/'world.sdf')]
    if not args.gui: cmd.insert(2,'-s')
    server=subprocess.Popen(cmd,stdout=log,stderr=log,start_new_session=True)
    report={'case':args.case,'run':str(run),'stages':{},'passed':False}
    def sim(): return states.get('lift',[0]*7)[6]
    def clip(v,lim): return max(-lim,min(lim,v))
    def depth(bale='bale_1'):
        c=poses['attachment_carrier']; b=poses[bale]; w,x,y,z=c[3:]; dx=cfg['attachment']['root_x']+cfg['attachment']['spear_length']; dz=cfg['attachment']['root_z']
        tipx=c[0]+(1-2*(y*y+z*z))*dx+2*(x*z+w*y)*dz
        return tipx-(b[0]-cfg['bale']['width']/2)
    def phase(name,duration,lift=0,angle=-.03,speed=0,until=None):
        start=sim(); deadline=time.monotonic()+max(30,duration*6)
        while sim()-start<duration:
            if time.monotonic()>deadline or server.poll() is not None: raise RuntimeError('Server stopped or simulation timeout')
            for n,target in [('lift',lift),('tilt',angle-states['lift'][0])]:
                error=target-states[n][0]; u=clip(error*8,.65)
                if abs(error)<.003: u=0
                elif abs(u)<.09: u=math.copysign(.09,u)
                pubs[n].publish(Double(data=u))
            msg=Twist(); msg.linear.x=speed() if callable(speed) else speed; drive.publish(msg)
            trace.write(json.dumps({'phase':name,'time':sim(),'states':dict(states),'poses':dict(poses)})+'\n')
            if until and until(): break
            time.sleep(.05)
        result={'time':sim(),'poses':dict(poses),'states':dict(states),'depth':depth()}
        report['stages'][name]=result; print(name,json.dumps({'t':sim(),'depth':depth(),'bales':{k:v[:3] for k,v in poses.items() if k.startswith('bale')}}),flush=True)
        return result
    try:
        deadline=time.monotonic()+25
        while not (all(k in states for k in ['lift','tilt','contact']) and all(k in poses for k in ['bale_1','bale_2','attachment_carrier'])):
            if time.monotonic()>deadline: raise RuntimeError('No complete Gazebo telemetry; inspect server.log')
            time.sleep(.1)
        phase('settle',2)
        target=cfg['bale']['width']*cfg['test']['insertion_fraction']
        phase('insert',18,speed=lambda:clip((target-depth())*.8,.15),until=lambda:abs(target-depth())<.012)
        reached=abs(target-depth())<.025
        phase('stop',1)
        phase('lift',8,lift=.20,angle=.015)
        phase('hold',3,lift=.20,angle=.015)
        raised=poses['bale_1'][2]>cfg['bale']['height']/2+.25
        report.update(insertion_reached=reached,bale_raised=raised)
        if args.case=='A': report['passed']=reached and raised and states['contact'][2]>0
        elif args.case in ['B','C']:
            report['passed']=True; report['interpretation']='Executed sensitivity experiment; inspect depth and lift, not a required failure.'
        elif reached and raised:
            # Raise above the lower bale before advancing, then lower to contact.
            phase('raise_for_stack',10,lift=.56,angle=0)
            phase('align_stack',45,lift=.56,angle=0,speed=lambda:clip((poses['bale_2'][0]-poses['bale_1'][0])*.6,.2),until=lambda:abs(poses['bale_2'][0]-poses['bale_1'][0])<.025)
            phase('place',10,lift=.365,angle=0)
            phase('unload_spears',6,lift=.345,angle=0)
            phase('withdraw',12,lift=.345,angle=0,speed=-.12,until=lambda:depth()<-.12)
            phase('stack_settle',3,lift=.345,angle=0)
            h=cfg['bale']['height']; stable=abs(poses['bale_1'][2]-poses['bale_2'][2]-h)<.08
            report['stacked']=stable
            if not stable: raise RuntimeError('Stack did not remain stable after withdrawal')
            phase('lower_for_second',10,lift=0,angle=-.03)
            phase('insert_lower',20,speed=lambda:clip((target-depth('bale_2'))*.8,.12),until=lambda:abs(target-depth('bale_2'))<.012)
            phase('lift_both',10,lift=.20,angle=.015)
            report['both_raised']=poses['bale_2'][2]>h/2+.25 and abs(poses['bale_1'][2]-poses['bale_2'][2]-h)<.1
            phase('forward',5,lift=.20,angle=.015,speed=.2)
            phase('brake',2,lift=.20,angle=.015)
            phase('reverse',5,lift=.20,angle=.015,speed=-.2)
            phase('final_hold',3,lift=.20,angle=.015)
            report['passed']=report['both_raised'] and poses['bale_2'][2]>h/2+.25 and abs(poses['bale_1'][2]-poses['bale_2'][2]-h)<.1
    except Exception as exc: report['error']=str(exc)
    finally:
        if server.poll() is None:
            os.killpg(server.pid,signal.SIGINT)
            try: server.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(server.pid,signal.SIGKILL); server.wait()
        report['server_exit']=server.returncode
        (run/'report.json').write_text(json.dumps(report,indent=2)); trace.close(); log.close()
    print(json.dumps({k:v for k,v in report.items() if k!='stages'},indent=2),flush=True)
    return 0 if report['passed'] else 1

if __name__=='__main__': raise SystemExit(main())
