"""Articulated tractor/trailer lattice planning using measured initial geometry."""
import heapq
import itertools
import math
import time
from .task005_geometry import angle, rectangles_overlap

HITCH = .22
AXLE = 1.4
BED = 1.3


def bed_pose(state):
    x,y,yaw,trailer = state
    return (x-HITCH*math.cos(yaw)-BED*math.cos(trailer),
            y-HITCH*math.sin(yaw)-BED*math.sin(trailer),trailer)


def advance(state, distance, curvature):
    x,y,yaw,beta = state
    # Off-axle hitch: trailer axle has no lateral velocity.
    rate = (math.sin(yaw-beta)-HITCH*curvature*math.cos(yaw-beta))/AXLE
    return (x+distance*math.cos(yaw),y+distance*math.sin(yaw),
            angle(yaw+distance*curvature),angle(beta+distance*rate))


def arrived(state, target, xy=.065, yaw=.045):
    bed=bed_pose(state)
    return (math.dist(bed[:2],target[:2])<xy
            and abs(angle(bed[2]-target[2]))<yaw
            and abs(angle(state[2]-state[3]))<.10)


class TrailerPlanner:
    def __init__(self, bounds, obstacles, health=lambda:None, timeout=15.):
        self.bounds,self.obstacles,self.health,self.timeout=bounds,obstacles,health,timeout

    def clear(self, state):
        x,y,theta,beta=state
        if abs(angle(theta-beta))>1.15:return False
        bx,by,_=bed_pose(state)
        shapes=((x,y,theta,-.52,.82,-.44,.44),
                (bx,by,beta,-.96,.96,-.69,.69))
        for rect in shapes:
            cx,cy,yaw,x0,x1,y0,y1=rect
            for u in (x0,x1):
                for v in (y0,y1):
                    px,py=cx+u*math.cos(yaw)-v*math.sin(yaw),cy+u*math.sin(yaw)+v*math.cos(yaw)
                    if not(self.bounds[0]<px<self.bounds[1] and self.bounds[2]<py<self.bounds[3]):return False
            if any(rectangles_overlap(rect,o,.06) for o in self.obstacles):return False
        return True

    def plan(self, start, target):
        from .trailer_maneuver import maneuver_connection
        connection = maneuver_connection(start,target,self.clear,self.health)
        if connection is not None:
            return connection
        deadline=time.monotonic()+self.timeout
        serial=itertools.count()
        def heuristic(s):
            bx,by,beta=bed_pose(s)
            return math.hypot(bx-target[0],by-target[1])+1.2*abs(angle(beta-target[2]))+.4*abs(angle(s[2]-beta))
        def key(s,direction):
            near=math.dist(bed_pose(s)[:2],target[:2])<.8
            r,a=(.055,.055) if near else (.20,.14)
            return tuple(round(v/q) for v,q in zip(s,(r,r,a,a)))+(direction,)
        if not self.clear(start):raise RuntimeError('TRAILER_START_IN_COLLISION')
        # Nodes retain parent indices; quantization never changes a physical state.
        nodes=[(tuple(start),None,0.,0.,0.)]
        queue=[(2.*heuristic(start),next(serial),0)]
        costs={key(start,1):0.}
        while queue:
            if next(serial)%64==0:
                self.health()
                if time.monotonic()>deadline:raise RuntimeError('TRAILER_PLANNING_TIMEOUT')
            _,_,index=heapq.heappop(queue)
            state,parent,cost,previous_distance,_=nodes[index]
            if arrived(state,target,xy=.045,yaw=.035):
                result=[]
                while nodes[index][1] is not None:
                    s,parent,_,distance,curvature=nodes[index]
                    result.append((s,distance,curvature));index=parent
                return list(reversed(result))
            near=math.dist(bed_pose(state)[:2],target[:2])<.8
            length=.10 if near else .36
            for direction in (1,-1):
                for curvature in (0.,-.35,.35,-.7,.7):
                    current=state; samples=[]
                    for _ in range(4):
                        current=advance(current,direction*length/4,curvature)
                        if not self.clear(current):break
                        samples.append(current)
                    if len(samples)!=4:continue
                    newcost=cost+length*(1. if direction>0 else 1.35)+.06*abs(curvature)
                    if previous_distance*direction<0:newcost+=.35
                    k=key(current,direction)
                    if newcost>=costs.get(k,math.inf):continue
                    costs[k]=newcost
                    nodes.append((current,index,newcost,direction*length,curvature))
                    heapq.heappush(queue,(newcost+2.*heuristic(current),next(serial),len(nodes)-1))
        raise RuntimeError('TRAILER_NO_COLLISION_FREE_ROUTE')


def compact(path):
    """Join identical controls after every original sample passed collision checks."""
    result=[]
    for state,distance,control in path:
        same = (result and (distance==0.)==(result[-1][1]==0.) and
                ((control*result[-1][2]>0.) if distance==0. else
                 (distance*result[-1][1]>0. and control==result[-1][2])))
        if same:
            previous=result[-1]
            result[-1]=(state,previous[1]+distance,
                        previous[2]+control if distance==0. else control)
        else:
            result.append((state,distance,control))
    return result
