"""Feedback-driven Innok223 preparation and transport with a native cargo joint."""
import json
import math
from pathlib import Path
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,CancelResponse,GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist,PoseStamped
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String,Float64,Empty
from ros_interfaces.action import ExecuteSkill
from cask_gazebo_motion.gazebo_attachment import NativeAttachment
from cask_gazebo_motion.trailer_navigation import TrailerPlanner,bed_pose,arrived
from cask_gazebo_motion.task005_geometry import angle
from cask_gazebo_motion.innok223_steering import steering_commands,STEERING_LIMIT


class Innok223Server(Node):
    def __init__(self):
        super().__init__('innok223_transporting_server')
        from ament_index_python.packages import get_package_share_directory
        default=str(Path(get_package_share_directory('cask_component_manager'))/'config/final_scene.json')
        self.declare_parameter('final_scene_config',default)
        self.declare_parameter('drive_speed',.30)
        self.declare_parameter('motion_timeout',600.)
        self.declare_parameter('steering_limit_deg',85.)
        self.steering_limit=math.radians(float(self.get_parameter('steering_limit_deg').value))
        if not 0.<self.steering_limit<=STEERING_LIMIT:
            raise ValueError('Front steering limit must be within 85 degrees')
        self.scene=json.loads(Path(self.get_parameter('final_scene_config').value).read_text())
        self.lock=threading.Lock();self.group=ReentrantCallbackGroup()
        self.base=self.cargo=self.hitch=self.steering=None
        self.steering_seen=0.
        self.base_seen=self.cargo_seen=self.hitch_seen=0.
        self.other={};self.stationary_since=None;self.last_base=None
        self.locked=False;self.velocity=(0.,0.)
        self.publisher=self.create_publisher(Twist,'/innok223/cmd_vel',10)
        self.steering_pub=self.create_publisher(Float64,'/innok223/steering_cmd',10)
        self.state_pub=self.create_publisher(String,'/cask/final/carrier_state',10)
        self.create_subscription(TFMessage,'/model/innok223/pose',self.on_pose,10)
        self.create_subscription(TFMessage,'/model/jekko/pose',self.on_jekko,10)
        self.create_subscription(JointState,'/innok223/joint_states',self.on_joint,qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'/cask/final/cargo_pose',self.on_cargo,10)
        self.attachment=NativeAttachment(child_model=self.scene['cargo']['model'],
            parent_model='innok223',parent_link='innok223_trailer_link',topic='/cask/innok223/final_steel')
        self.carrier_native_revision=0
        self.carrier_native_state=None
        self.detach_pub=self.create_publisher(Empty,self.attachment.TOPIC+'/detach',1)
        self.create_subscription(String,self.attachment.TOPIC+'/state',self.on_native_state,10)
        self.servers=[ActionServer(self,ExecuteSkill,endpoint,
            execute_callback=lambda goal,op=operation:self.execute(goal,op),
            goal_callback=lambda _:GoalResponse.ACCEPT,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
            for operation,endpoint in [('prepare','/innok223/navigate'),('transport','/innok223_01/transporting')]]
        self.create_timer(.05,self.publish_state,callback_group=self.group)
        self.get_logger().info('Innok223 continuous front steering enabled; limit=+/-85 deg; arc navigation; no in-place turns')

    def on_joint(self,message):
        for name,position in zip(message.name,message.position):
            if name.rsplit('::',1)[-1]=='innok223_trailer_hitch_joint' and math.isfinite(position):
                self.hitch,self.hitch_seen=position,time.monotonic()
            if name.rsplit('::',1)[-1]=='innok223_caster_swivel_joint' and math.isfinite(position):
                self.steering,self.steering_seen=position,time.monotonic()

    @staticmethod
    def model_pose(message,name):
        for t in message.transforms:
            if t.child_frame_id.replace('::','/').split('/')[-1]!=name:continue
            p,q=t.transform.translation,t.transform.rotation
            values=(p.x,p.y,p.z,q.x,q.y,q.z,q.w)
            if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1)>.05:continue
            return (p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    def on_pose(self,message):
        pose=self.model_pose(message,'innok223')
        if pose is None:return
        now=time.monotonic()
        stable=self.last_base and math.dist(pose[:2],self.last_base[:2])<.001 and abs(angle(pose[3]-self.last_base[3]))<.002
        self.stationary_since=(self.stationary_since or now) if stable and self.velocity==(0.,0.) else None
        self.base,self.base_seen,self.last_base=pose,now,pose

    def on_jekko(self,message):
        pose=self.model_pose(message,'jekko')
        if pose:self.other['jekko']=(pose,time.monotonic())

    def on_cargo(self,message):
        p,q=message.pose.position,message.pose.orientation
        self.cargo=(p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.cargo_seen=time.monotonic()

    def on_native_state(self,message):
        if message.data in ('attached','detached'):
            # One immutable snapshot prevents mixing an old state with a new
            # revision while the action callback checks the acknowledgement.
            self.carrier_native_revision+=1
            self.carrier_native_state=(self.carrier_native_revision,message.data)

    def state(self):
        if self.base is None or self.hitch is None:return None
        return (self.base[0],self.base[1],self.base[3],angle(self.base[3]+self.hitch))

    def stationary(self):
        return self.stationary_since is not None and time.monotonic()-self.stationary_since>.5

    def publish_state(self):
        if self.state() is None or time.monotonic()-min(self.base_seen,self.hitch_seen)>1.:return
        x,y,yaw=bed_pose(self.state())
        message=String();message.data=json.dumps(dict(bed=[x,y,self.base[2]+self.scene['trailer']['deck_height'],yaw],
            stationary=self.stationary(),cargo_locked=self.locked,steering_angle=self.steering,steering_limit=self.steering_limit))
        self.state_pub.publish(message)

    def drive(self,v=0.,w=0.):
        target,v,w=steering_commands(v,w,self.steering or 0.,self.steering_limit)
        self.steering_pub.publish(Float64(data=float(target)))
        self.velocity=(v,w)
        message=Twist();message.linear.x=float(v);message.angular.z=float(w)
        self.publisher.publish(message)

    def check(self,goal,cargo=False):
        if goal.is_cancel_requested:raise RuntimeError('CANCELED')
        if self.state() is None or time.monotonic()-min(self.base_seen,self.hitch_seen)>2.:
            raise RuntimeError('INNOK_FEEDBACK_LOST')
        if cargo and (self.cargo is None or time.monotonic()-self.cargo_seen>2.):
            raise RuntimeError('CARGO_FEEDBACK_LOST')
        if self.steering is None or time.monotonic()-self.steering_seen>2.:
            raise RuntimeError('STEERING_FEEDBACK_LOST')
        if abs(self.steering)>self.steering_limit+.02:
            raise RuntimeError('FRONT_STEERING_LIMIT_EXCEEDED')
        if abs(self.hitch)>1.2:raise RuntimeError('TRAILER_ARTICULATION_LIMIT')

    def feedback(self,goal,progress,status):
        message=ExecuteSkill.Feedback();message.progress=float(progress);message.status=status
        goal.publish_feedback(message)

    def obstacles(self):
        scene=self.scene;b=scene['workbench'];cx,cy,_,yaw=scene['robots']['crane']
        result=[(cx,cy,yaw,-1.3,1.3,-1.1,1.1),
                (*b['center'][:2],0.,-b['size'][0]/2,b['size'][0]/2,-b['size'][1]/2,b['size'][1]/2)]
        for pose,seen in self.other.values():
            if time.monotonic()-seen>2.:raise RuntimeError('JEKKO_FEEDBACK_LOST')
            result.append((pose[0],pose[1],pose[3],-.95,.95,-.65,.65))
        return result

    def navigate(self,goal,target):
        cx,cy=self.scene['site']['center'];sx,sy=self.scene['site']['size']
        bounds=(cx-sx/2,cx+sx/2,cy-sy/2,cy+sy/2)
        until=time.monotonic()+float(self.get_parameter('motion_timeout').value)
        path=[];index=0;replans=0
        while time.monotonic()<until:
            self.check(goal,self.locked)
            current=self.state()
            if arrived(current,target,xy=.045,yaw=.04):
                self.drive()
                if self.stationary():return
                time.sleep(.05);continue
            obstacles=self.obstacles()
            if not path or index>=len(path):
                self.drive();replans+=1
                self.feedback(goal,10.,'PLANNING_TRACTOR_TRAILER_ARCS | attempt='+str(replans))
                planner=TrailerPlanner(bounds,obstacles,lambda:self.check(goal,self.locked))
                # Retain samples for curved-path tracking; do not merge an arc
                # into one endpoint and then align to it while standing still.
                path=planner.plan(current,target);index=0
                if not path:continue
            direction=1. if path[index][1]>=0. else -1.
            end=index
            while end+1<len(path) and path[end+1][1]*direction>0.:
                end+=1
            # Match locally in the current gear, including measured heading.
            nearest=min(range(index,min(end+1,index+13)),key=lambda i:
                math.dist(current[:2],path[i][0][:2])+.12*abs(angle(current[2]-path[i][0][2])))
            index=nearest
            endpoint=path[end][0]
            remaining=math.dist(current[:2],endpoint[:2])
            endpoint_tolerance=.018 if end==len(path)-1 else .055
            dx,dy=endpoint[0]-current[0],endpoint[1]-current[1]
            along=direction*(dx*math.cos(endpoint[2])+dy*math.sin(endpoint[2]))
            lateral_end=abs(-dx*math.sin(endpoint[2])+dy*math.cos(endpoint[2]))
            # A gear-change point is an approach-line crossing, not a tiny disk
            # to chase after passing it. The final cargo arrival test stays exact.
            crossed=(end<len(path)-1 and along<=.025 and lateral_end<.10)
            if remaining<endpoint_tolerance or crossed:
                self.drive()
                if self.stationary():
                    index=end+1
                    if index>=len(path):path=[]
                time.sleep(.05);continue
            reference,_,curvature=path[index]
            checker=TrailerPlanner(bounds,obstacles)
            if not checker.clear(reference):
                self.drive();path=[];time.sleep(.1);continue
            deviation=math.dist(current[:2],reference[:2])
            if deviation>.30:
                self.drive();path=[];continue
            yaw_error=angle(reference[2]-current[2])
            cross=-(reference[0]-current[0])*math.sin(current[2])+(reference[1]-current[1])*math.cos(current[2])
            limit=min(float(self.get_parameter('drive_speed').value),.18/max(abs(curvature),1e-6))
            speed=direction*min(limit,max(.035,.8*remaining))
            if abs(yaw_error)>.5:speed=math.copysign(min(abs(speed),.07),speed)
            # Reverse tracking must stabilize the trailer heading as well as
            # the tractor; tractor-only alignment lets the trailer drift across
            # the bay and forces repeated pull-forward/back-up corrections.
            trailer_error=angle(reference[3]-current[3])
            if direction>0.:
                correction=1.25*yaw_error+.9*cross
            else:
                correction=speed*(-2.*cross-2.3118*yaw_error+10.9812*trailer_error)
            omega=max(-.23,min(.23,speed*curvature+correction))
            requested,_,_=steering_commands(speed,omega,self.steering,self.steering_limit)
            # Make the planned turn while the front wheel is still slewing,
            # slowing both commands together so curvature stays consistent.
            lag_scale=max(.15,min(1.,.08/max(abs(requested-self.steering),.08)))
            speed*=lag_scale;omega*=lag_scale
            _,applied_speed,applied_omega=steering_commands(speed,omega,self.steering,self.steering_limit)
            x,y,theta,beta=current
            predicted=(x+.5*applied_speed*math.cos(theta),y+.5*applied_speed*math.sin(theta),
                       angle(theta+.5*applied_omega),
                       angle(beta+.5*(applied_speed*math.sin(theta-beta)-.22*applied_omega*math.cos(theta-beta))/1.4))
            if not checker.clear(current):
                self.drive();path=[];time.sleep(.1);continue
            if not checker.clear(predicted):
                # A lagging steering actuator can momentarily predict an unsafe
                # bend. Brake while allowing it to reach the safe requested angle.
                self.drive()
                requested,_,_=steering_commands(speed,omega,self.steering,self.steering_limit)
                self.steering_pub.publish(Float64(data=float(requested)))
                time.sleep(.1);continue
            self.drive(speed,omega)
            self.feedback(goal,20.,'TRANSPORTING_WITH_STEERING' if self.locked else 'DRIVING_TO_LOADING_BAY_WITH_STEERING')
            time.sleep(.05)
        raise RuntimeError('INNOK_NAVIGATION_TIMEOUT')

    def attachment_state(self,goal,wanted):
        until=time.monotonic()+10.
        while time.monotonic()<until:
            self.check(goal,True)
            _,state=self.attachment.state_snapshot()
            if state==wanted:return
            self.drive();time.sleep(.05)
        raise RuntimeError('CARRIER_ATTACHMENT_NOT_CONFIRMED: '+wanted)

    def release_cargo(self,goal,delivery):
        """Release a parked load using a fresh native acknowledgement."""
        before_ros=self.carrier_native_revision
        before_native,_=self.attachment.state_snapshot()
        began=time.monotonic();last_report=-math.inf

        def confirmed():
            native=self.carrier_native_state
            revision,state=self.attachment.state_snapshot()
            return ((native is not None and native[0]>before_ros and native[1]=='detached')
                    or (revision>before_native and state=='detached'))

        def tick():
            nonlocal last_report
            self.check(goal,True)
            self.drive()
            if math.dist(self.cargo[:3],delivery[:3])>.15:
                raise RuntimeError('CARGO_DELIVERY_NOT_VERIFIED')
            now=time.monotonic()
            if now-last_report>=.3:
                revision,state=self.attachment.state_snapshot()
                native=self.carrier_native_state
                self.feedback(goal,90.,'RELEASING_CARRIER_CARGO | awaiting_native_detach=true'
                    +f' | elapsed={now-began:.3f} | native_state={state} | native_revision={revision}'
                    +f' | ros_state={native[1] if native else None}')
                last_report=now

        tick()
        if self.detach_pub.get_subscription_count()>0:
            next_publish=-math.inf
            while time.monotonic()-began<.75:
                tick()
                if confirmed():break
                now=time.monotonic()
                if now>=next_publish:
                    self.detach_pub.publish(Empty())
                    next_publish=now+.15
                time.sleep(.01)
        if not confirmed():
            # Keep the original revision baseline when falling back; a late
            # native acknowledgement remains valid. This path retries too.
            self.attachment.detach(tick=tick,confirmed=confirmed)
        if not confirmed():
            raise RuntimeError('CARRIER_ATTACHMENT_NOT_CONFIRMED: detached')
        self.locked=False
        self.feedback(goal,95.,'CARRIER_RELEASE_CONFIRMED | native_detached=true'
                      +f' | elapsed={time.monotonic()-began:.3f}')

    def execute(self,goal,operation):
        result=ExecuteSkill.Result()
        if not self.lock.acquire(False):
            goal.abort();result.message='ROBOT_BUSY';return result
        try:
            names,values,units=goal.request.parameter_names,goal.request.parameter_values,goal.request.parameter_units
            if not(len(names)==len(values)==len(units)) or len(set(names))!=len(names) or not all(math.isfinite(v) for v in values):raise ValueError('INVALID_PARAMETERS')
            p=dict(zip(names,values))
            if p.get('final_scene')!=1.:raise ValueError('FINAL_SCENE_PARAMETERS_REQUIRED')
            until=time.monotonic()+15.
            while self.state() is None or self.steering is None:
                if goal.is_cancel_requested:raise RuntimeError('CANCELED')
                if time.monotonic()>until:raise RuntimeError('INNOK_FEEDBACK_NOT_READY')
                time.sleep(.05)
            pickup=tuple(p['pickup_'+k] for k in ('x','y','z','yaw'))
            delivery=tuple(p['delivery_'+k] for k in ('x','y','z','yaw'))
            if operation=='prepare':
                if self.locked:raise RuntimeError('LOAD_ALREADY_ATTACHED')
                self.navigate(goal,(pickup[0],pickup[1],pickup[3]))
            else:
                self.check(goal,True)
                if not arrived(self.state(),(pickup[0],pickup[1],pickup[3])):raise RuntimeError('TRAILER_NOT_AT_PICKUP')
                if math.dist(self.cargo[:3],pickup[:3])>.13:raise RuntimeError('CARGO_NOT_ON_TRAILER')
                self.drive();self.attachment.attach();self.locked=True
                self.attachment_state(goal,'attached')
                self.navigate(goal,(delivery[0],delivery[1],delivery[3]))
                if math.dist(self.cargo[:3],delivery[:3])>.15:raise RuntimeError('CARGO_DELIVERY_NOT_VERIFIED')
                self.release_cargo(goal,delivery)
                until=time.monotonic()+1.
                while time.monotonic()<until:self.check(goal,True);self.drive();time.sleep(.05)
                if math.dist(self.cargo[:3],delivery[:3])>.15:raise RuntimeError('CARGO_UNSTABLE_AFTER_RELEASE')
            self.feedback(goal,100.,'SUCCESS | measured_motion_verified=true')
            goal.succeed();result.success=True;result.message='Innok223 '+operation+' completed'
        except Exception as error:
            result.success=False;result.message=str(error)+'; carrying_phase='+str(self.locked).lower()
            if goal.is_cancel_requested:goal.canceled()
            else:goal.abort()
            self.get_logger().error(result.message)
        finally:
            self.drive();self.lock.release()
        return result

    def destroy_node(self):
        self.drive();self.attachment.close();return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=Innok223Server()
    executor=MultiThreadedExecutor(num_threads=6);executor.add_node(node)
    try:executor.spin()
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
