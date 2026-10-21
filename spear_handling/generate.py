#!/usr/bin/env python3
"""Generate an isolated spear world; coordinates: penetration X, bale length Y."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as E
ROOT = Path(__file__).resolve().parents[1]

def validate_config(config):
    """Reject nonphysical geometry/parameters before writing or starting Gazebo."""
    for section in ('bale', 'attachment', 'channel', 'friction', 'resistance', 'physics', 'test'):
        for key, value in config[section].items():
            if key == 'enabled':
                if not isinstance(value, bool):
                    raise ValueError('resistance.enabled must be boolean')
            elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{section}.{key} must be finite numeric')
    b, a, ch = (config[n] for n in ('bale', 'attachment', 'channel'))
    for name, values in [('bale', b), ('attachment', a), ('channel', ch), ('physics', config['physics'])]:
        for key, value in values.items():
            if key not in ('root_x', 'root_z') and value <= 0:
                raise ValueError(f'{name}.{key} must be positive')
    if not isinstance(a['count'], int) or a['count'] < 2:
        raise ValueError('attachment.count must be an integer >= 2')
    spacing = (a['frame_width']-2*a['side_margin'])/(a['count']-1)
    channels=config.get('interaction',{}).get('mode','channels')=='channels'
    if channels and not (a['diameter'] < ch['width'] < spacing and ch['height'] > a['diameter']):
        raise ValueError('Shafts must fit separated channels')
    if channels and a['frame_width']-2*a['side_margin']+ch['width'] >= b['length']:
        raise ValueError('Outer channels must fit inside bale length')
    if channels and not (0 < ch['height_from_bottom']-ch['height']/2 <
            ch['height_from_bottom']+ch['height']/2 < b['height']):
        raise ValueError('Channels must leave intact top and bottom slabs')
    if any(v < 0 for v in config['friction'].values()):
        raise ValueError('Friction coefficients must be nonnegative')
    if any(v < 0 for k, v in config['resistance'].items() if k != 'enabled'):
        raise ValueError('Resistance coefficients must be nonnegative')
    fraction = config['test']['insertion_fraction']
    if not (0 < fraction <= 1 and fraction*b['width'] <= a['spear_length']):
        raise ValueError('Insertion fraction must be positive and reachable by finite shafts')
    if config['test']['transport_speed'] <= 0:
        raise ValueError('Transport speed must be positive')
    material=config.get('interaction',{})
    if material.get('mode','channels') not in ('channels','material'):
        raise ValueError('interaction.mode must be channels or material')
    if material.get('mode')=='material':
        for key in ('samples','transverse_stiffness','transverse_damping','bearing_limit_per_m',
                    'axial_stiffness','axial_damping','confinement_per_m','cutting_force'):
            value=material[key]
            positive=key in ('samples','transverse_stiffness','bearing_limit_per_m','axial_stiffness')
            if (isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value)
                    or (value<=0 if positive else value<0)):
                raise ValueError(f'Invalid interaction.{key}')
        if not isinstance(material['samples'],int) or not 4<=material['samples']<=256:
            raise ValueError('interaction.samples must be an integer from 4 to 256')
        if config['resistance']['enabled']:
            raise ValueError('Material interaction already includes axial resistance')
        for key in ('insertion_preload_N','release_preload_N'):
            if config['test'].get(key,1)<=0:
                raise ValueError(f'test.{key} must be positive')
        for key in ('entry_lift','insertion_lift'):
            if not 0<=config['test'].get(key,0)<=1.44:
                raise ValueError(f'test.{key} exceeds lift limits')

def el(parent, tag, value=None, **attrs):
    n = E.SubElement(parent, tag, attrs)
    if value is not None: n.text = str(value)
    return n

def box(link, name, size, pos, visual=False, color='0.65 0.47 0.17 1'):
    n = el(link, 'visual' if visual else 'collision', name=name)
    el(n, 'pose', ' '.join(map(str, [*pos,0,0,0])))
    el(el(el(n,'geometry'),'box'),'size',' '.join(map(str,size)))
    if visual:
        mat=el(n,'material'); el(mat,'ambient',color); el(mat,'diffuse',color)
    return n

def inertia(link, parts):
    mass=sum(p[0] for p in parts)
    c=[sum(p[0]*p[1][i] for p in parts)/mass for i in range(3)]
    n=el(link,'inertial'); el(n,'mass',mass); el(n,'pose',' '.join(map(str,[*c,0,0,0])))
    tensor=el(n,'inertia')
    for i,key in enumerate(['ixx','iyy','izz']):
        el(tensor,key,sum(p[2][i]+p[0]*sum((p[1][j]-c[j])**2 for j in range(3) if j!=i) for p in parts))
    for i,j,key in [(0,1,'ixy'),(0,2,'ixz'),(1,2,'iyz')]:
        el(tensor,key,-sum(p[0]*(p[1][i]-c[i])*(p[1][j]-c[j]) for p in parts))

def boxpart(m,size,pos):
    return m,pos,[m*sum(size[j]**2 for j in range(3) if i!=j)/12 for i in range(3)]

def collision_mask(collision, category, mask):
    surface=collision.find('surface')
    if surface is None: surface=el(collision,'surface')
    contact=el(surface,'contact')
    el(contact,'category_bitmask',category)
    el(contact,'collide_bitmask',mask)

def generate(config, output):
    validate_config(config)
    b,a,ch=config['bale'],config['attachment'],config['channel']
    material=config.get('interaction',{}).get('mode','channels')=='material'
    d,l,h=b['width'],b['length'],b['height']
    spacing=(a['frame_width']-2*a['side_margin'])/(a['count']-1)
    centers=[(i-(a['count']-1)/2)*spacing for i in range(a['count'])]
    tree=E.parse(ROOT/'worlds/simple_tractor.sdf'); world=tree.getroot().find('world')
    for p in list(world.findall('plugin')):
        if 'Sensors' in p.get('name',''): world.remove(p)
    for link in world.findall('model/link'):
        for s in list(link.findall('sensor')): link.remove(s)
    for model in list(world.findall('model')):
        if model.get('name','').startswith('bale_'): world.remove(model)
    tractor=world.find("model[@name='simple_tractor']")
    old=tractor.find("link[@name='attachment_carrier']"); pose=old.findtext('pose'); tractor.remove(old)
    for n in list(tractor):
        if n.tag in ('joint','link') and n.get('name','').startswith('grip_'): tractor.remove(n)
    hyd=tractor.find("plugin[@name='bale_sim::LoaderHydraulics']")
    hyd.set('filename',str(ROOT/'spear_handling/build/libBaleLoaderHydraulics.so'))
    for n in list(hyd):
        if n.tag.startswith('grip_'): hyd.remove(n)
    pub=tractor.find("plugin[@name='gz::sim::systems::JointStatePublisher']")
    for n in list(pub):
        if n.tag=='joint_name' and n.text.startswith('grip_'): pub.remove(n)
    carrier=el(tractor,'link',name='attachment_carrier'); el(carrier,'pose',pose)
    size=[.1,a['frame_width'],.6]; pos=[0,0,.35]
    box(carrier,'frame',size,pos); box(carrier,'frame_visual',size,pos,True,'0.18 0.2 0.22 1')
    parts=[boxpart(a['frame_mass'],size,pos)]
    radius=a['diameter']/2; length=a['spear_length']
    mass=math.pi*radius**2*length*a['steel_density']
    for i,y in enumerate(centers):
        pos=[a['root_x']+length/2,y,a['root_z']]
        parts.append((mass,pos,[mass*radius**2/2,mass*(3*radius**2+length**2)/12,mass*(3*radius**2+length**2)/12]))
        for tag in ['collision','visual']:
            n=el(carrier,tag,name=f'spear_{i}_{tag}')
            el(n,'pose',' '.join(map(str,[*pos,0,math.pi/2,0])))
            cyl=el(el(n,'geometry'),'cylinder'); el(cyl,'radius',radius); el(cyl,'length',length)
            if tag=='collision' and material:
                # Categories: other=1, bale=2, spear=4. Both sides exclude the
                # native spear/bale pair; ground and frame contact remain.
                collision_mask(n,4,5)
            if tag=='visual':
                mat=el(n,'material'); el(mat,'diffuse','0.25 0.27 0.29 1')
    inertia(carrier,parts)
    for index,x in enumerate([config['test']['entry_x']+d/2,8.5]):
        model=el(world,'model',name=f'bale_{index+1}')
        el(model,'pose',f"{x} {config['test']['bale_y'] if index==0 else 0} {h/2+.002} 0 0 {config['test']['bale_yaw'] if index==0 else 0}")
        link=el(model,'link',name='bale_link'); inertia(link,[boxpart(b['mass'],[d,l,h],[0,0,0])])
        box(link,'straw',[d,l,h],[0,0,0],True)
        if material:
            collision_mask(box(link,'bale_body',[d,l,h],[0,0,0]),2,3)
        else:
            bottom=ch['height_from_bottom']-ch['height']/2; top=bottom+ch['height']
            box(link,'bale_bottom',[d,l,bottom],[0,0,(bottom-h)/2])
            box(link,'bale_top',[d,l,h-top],[0,0,top/2])
            edges=[-l/2]
            for y in centers: edges.extend([y-ch['width']/2,y+ch['width']/2])
            edges.append(l/2)
            for i in range(0,len(edges),2):
                low,high=edges[i:i+2]
                box(link,f'bale_web_{i}',[d,high-low,ch['height']],[0,(low+high)/2,ch['height_from_bottom']-h/2])
        # Visual ties wrap around the bale; never affect collision support.
        for y in centers:
            box(link,f'tie_{y}',[d+.002,.012,h+.002],[0,y,0],True,'0.25 0.22 0.12 1')
    plugin=el(world,'plugin',filename=str(ROOT/'spear_handling/build/libSpearContact.so'),name='spear_handling::SpearContact')
    params=dict(config['friction'],bale_depth=d,bale_length=l,bale_height=h,spear_length=length,root_x=a['root_x'],root_z=a['root_z'],count=a['count'],spacing=spacing,resistance=str(config['resistance']['enabled']).lower(),resistance_stiffness=config['resistance']['force_per_meter_per_spear'],resistance_viscous=config['resistance']['viscous_per_spear'],resistance_cap=config['resistance']['max_force_per_spear'])
    for key,value in params.items(): el(plugin,key,value)
    el(plugin,'material',str(material).lower())
    if material:
        for key,value in config['interaction'].items():
            if key!='mode': el(plugin,'material_'+key,value)
    world.find('physics/max_step_size').text=str(config['physics']['step'])
    E.indent(tree); output=Path(output); output.parent.mkdir(parents=True,exist_ok=True); tree.write(output,encoding='unicode')

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--config',type=Path,default=Path(__file__).with_name('config.json')); p.add_argument('--output',type=Path,default=ROOT/'runs/spear/world.sdf'); args=p.parse_args()
    generate(json.loads(args.config.read_text()),args.output); print(args.output)
