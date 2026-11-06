"""Optional physical components owned by the existing Component Manager.

Read generated scene configurations; spawn rigid palletized timber and relay
measured pose. No teleports, synthetic carrying, robot controllers or trajectories.
"""
import atexit
import json
import math
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from cask_interfaces.srv import ModifyWorldModel, QueryOntology
from .generated_cargo import config_path

A='http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#'
S='urn:cask:scene:'


def timber_sdf(config):
    from ament_index_python.packages import get_package_share_directory
    template=Path(get_package_share_directory('cask_robot_description'))/'urdf/timber_component.sdf'
    root=ET.parse(template).getroot(); model=root.find('model')
    model.set('name',config['model'])
    model.find('link/inertial/mass').text=str(config['mass'])
    # Mass-dependent box inertia in this rigid pallet approximation.
    for key,coefficient in (('ixx',(1.6**2+.8**2)/12),('iyy',(1.2**2+.8**2)/12),('izz',(1.2**2+1.6**2)/12)):
        model.find('link/inertial/inertia/'+key).text=str(config['mass']*coefficient)
    return ET.tostring(root,encoding='unicode')


class GeneratedComponents:
    def __init__(self,node):
        self.node=node; self.entries={}; self.closed=False
        self.pending=set(); self.errors=set()
        self.timer=node.create_timer(1.,self.poll)
        atexit.register(self.close)

    def poll(self):
        if self.closed or not self.node.query_client.service_is_ready(): return
        directory=config_path('AnyTask').parent
        for path in directory.glob('*.json'):
            task=path.stem
            if task in self.entries or task in self.pending or task in self.errors: continue
            self.pending.add(task)
            request=QueryOntology.Request()
            request.query=f'SELECT ?profile WHERE {{ <{A+task}> <{S}executionProfile> ?profile }}'
            future=self.node.query_client.call_async(request)
            future.add_done_callback(lambda done,t=task:self.queried(done,t))

    def queried(self,future,task):
        self.pending.discard(task)
        try:
            response=future.result()
            if not response.success: raise RuntimeError(response.message)
            rows=json.loads(response.result_json)
            if not rows: return
            if len(rows)!=1: raise RuntimeError('Ambiguous generated scene.')
            config=json.loads(rows[0]['profile'])
            if config['task_id']!=task or config['profile']!='pallet_transfer': raise ValueError('Unsupported generated scene.')
            entry=dict(config=config,pose=None,pose_time=0.,bridge=None,ready=False,marking=False)
            entry['publisher']=self.node.create_publisher(PoseStamped,'/cask/'+task.lower()+'/cargo_pose',10)
            entry['subscription']=self.node.create_subscription(TFMessage,'/model/'+config['model']+'/pose',
                lambda msg,t=task:self.on_pose(t,msg),10)
            self.entries[task]=entry
            threading.Thread(target=self.spawn,args=(task,),daemon=True).start()
        except Exception as error:
            self.errors.add(task); self.node.get_logger().error('Generated component '+task+': '+str(error))

    def spawn(self,task):
        entry=self.entries[task]; c=entry['config']; p=c['pickup']
        try:
            bridge=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',
                '/model/'+c['model']+'/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/cask/jekko/'+task.lower()+'/state@std_msgs/msg/String[ignition.msgs.StringMsg'],
                stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            entry['bridge']=bridge
            result=subprocess.run(['ros2','run','ros_gz_sim','create','-world','empty',
                '-name',c['model'],'-allow_renaming','false','-string',timber_sdf(c),
                '-x',str(p[0]),'-y',str(p[1]),'-z',str(p[2]),'-Y',str(p[3])],
                capture_output=True,text=True,timeout=25)
            if result.returncode: raise RuntimeError(result.stdout+result.stderr)
            self.node.get_logger().info('GENERATED_COMPONENT_SPAWNED: '+task+' -> '+c['model'])
        except Exception as error:
            self.errors.add(task); self.node.get_logger().error('Generated component spawn failed: '+str(error))

    def on_pose(self,task,message):
        entry=self.entries[task]
        for transform in message.transforms:
            if transform.child_frame_id.replace('::','/').split('/')[-1]!=entry['config']['model']: continue
            p,q=transform.transform.translation,transform.transform.rotation
            if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)): return
            if abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1.)>.05: return
            pose=PoseStamped(); pose.header.stamp=transform.header.stamp; pose.header.frame_id='world'
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=p.x,p.y,p.z
            pose.pose.orientation=q
            entry['publisher'].publish(pose)
            entry['pose']=dict(frame='world',x=p.x,y=p.y,z=p.z,
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            entry['pose_time']=time.monotonic()
            if not entry['ready'] and not entry['marking'] and self.node.modify_client.service_is_ready():
                entry['marking']=True
                request=ModifyWorldModel.Request(); request.operation='set'
                request.subject=A+task; request.predicate=S+'gazeboReady'; request.object='true'
                request.object_is_literal=True; request.save_to_file=False
                future=self.node.modify_client.call_async(request)
                def finished(done):
                    entry['marking']=False
                    entry['ready']=bool(done.result().success)
                future.add_done_callback(finished)
            return

    def apply(self,state,context):
        entry=self.entries.get(state.task_id)
        if entry is None: return False
        pose=entry['pose']
        if pose is None or time.monotonic()-entry['pose_time']>2.: return True
        self.node.publish_component_marker(state,context,pose)
        if self.node.execution_stage(state.status)=='SUCCESS':
            self.node.persist_success_state(state,context,pose)
        # Failed loaded transfers are intentionally NOT synthesized here.
        # Existing task-specific recovery remains in its original code.
        return True

    def close(self):
        self.closed=True
        for entry in self.entries.values():
            process=entry.get('bridge')
            if process is not None and process.poll() is None:
                process.terminate()
                try: process.wait(timeout=1.)
                except subprocess.TimeoutExpired: process.kill()
