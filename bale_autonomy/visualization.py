"""Collision-free ground visuals for live Gazebo approach demonstrations."""
import math
import json
import select
import subprocess
import sys
import xml.etree.ElementTree as ET

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_factory_pb2 import EntityFactory
from .scene import ROOT


def line_model(name, points, color, width=.10, height=.025):
    """One static model, with visual-only boxes connecting world XY points."""
    sdf = ET.Element("sdf", version="1.10")
    model = ET.SubElement(sdf, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="path")
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length < .005:
            continue
        visual = ET.SubElement(link, "visual", name=f"segment_{i}")
        ET.SubElement(visual, "cast_shadows").text = "false"
        ET.SubElement(visual, "pose").text = (
            f"{(a[0]+b[0])/2} {(a[1]+b[1])/2} {height} 0 0 {math.atan2(dy,dx)}")
        box = ET.SubElement(ET.SubElement(visual, "geometry"), "box")
        ET.SubElement(box, "size").text = f"{length+.01} {width} .012"
        material = ET.SubElement(visual, "material")
        for field in ("ambient", "diffuse", "emissive"):
            ET.SubElement(material, field).text = " ".join(map(str,color)) + " 1"
    return ET.tostring(sdf, encoding="unicode")


class LivePath:
    def __init__(self, environment, directory):
        # Gazebo Transport's synchronous Python requests can block their own
        # response thread while image callbacks wait for the GIL. Isolate the
        # request client from high-rate subscriptions in a small persistent worker.
        self.log = (directory / "path_visuals.log").open("w")
        self.worker = subprocess.Popen([sys.executable,"-m","bale_autonomy.visualization"],
            cwd=ROOT,env=environment,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
            stderr=self.log,text=True,bufsize=1)
        self.revision = 0
        self.current_plan = None
        self.current_goal = None
        self.last_position = None
        self.trail_count = 0

    def create(self, name, points, color, **kwargs):
        self.request(dict(operation="create",name=name,sdf=line_model(name,points,color,**kwargs)))

    def remove(self, name):
        if name is None:
            return
        self.request(dict(operation="remove",name=name))

    def request(self, message):
        self.worker.stdin.write(json.dumps(message)+"\n")
        self.worker.stdin.flush()
        if not select.select([self.worker.stdout],[],[],4)[0]:
            raise RuntimeError("Gazebo path visualization worker timed out")
        response = json.loads(self.worker.stdout.readline())
        if not response.get("ok"):
            raise RuntimeError(f"Gazebo path visual {message['name']}: {response}")

    def close(self):
        self.worker.stdin.close()
        try:
            self.worker.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.worker.kill()
            self.worker.wait(timeout=2)
        self.log.close()

    def set_plan(self, path, goal):
        self.revision += 1
        name = f"autonomy_plan_{self.revision}"
        goal_name = f"autonomy_goal_{self.revision}"
        self.create(name,path,(.05,.35,1.0),width=.13)
        x,y,yaw = goal
        # A yellow ring plus heading arrow marks where the tractor will stop.
        ring = [(x+.40*math.cos(i*math.pi/16),y+.40*math.sin(i*math.pi/16))
                for i in range(33)]
        tip = (x+.9*math.cos(yaw),y+.9*math.sin(yaw))
        self.create(goal_name,ring+[(x,y),tip],(1.0,.75,.02),width=.075,height=.045)
        self.remove(self.current_plan)
        self.remove(self.current_goal)
        self.current_plan,self.current_goal = name,goal_name

    def set_target(self, target):
        from .planning import rectangle
        corners = rectangle(*target["position_world"][:2],target["yaw_world"],1.35,-1.35,.75)
        self.create("autonomy_selected_bale",list(corners)+[corners[0]],
                    (.95,.08,.85),width=.10,height=.035)

    def update_trail(self, pose):
        position = tuple(pose[:2])
        if self.last_position is None:
            self.last_position = position
        elif math.dist(position,self.last_position) >= .18:
            self.trail_count += 1
            self.create(f"autonomy_trail_{self.trail_count}",
                        [self.last_position,position],(.05,.95,.2),width=.08,height=.065)
            self.last_position = position


def worker_main():
    from gz.transport13 import Node
    node = Node()
    for line in sys.stdin:
        message = json.loads(line)
        try:
            if message["operation"] == "create":
                request = EntityFactory(sdf=message["sdf"],allow_renaming=False)
                ok, reply = node.request("/world/tractor_world/create",request,EntityFactory,Boolean,2000)
            else:
                request = Entity(name=message["name"],type=Entity.MODEL)
                ok, reply = node.request("/world/tractor_world/remove",request,Entity,Boolean,2000)
            response = dict(ok=bool(ok and reply.data),transport_ok=bool(ok),accepted=reply.data)
        except Exception as exc:
            response = dict(ok=False,error=str(exc))
        print(json.dumps(response),flush=True)


if __name__ == "__main__":
    worker_main()
