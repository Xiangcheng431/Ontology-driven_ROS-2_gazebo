"""Align outside the bay, then follow a straight terminal entry."""
import math
from .trailer_navigation import advance, bed_pose, angle


def arc_path(start, params):
    state, path = start, []
    for curvature, distance in zip(params[::2],params[1::2]):
        count = max(2,math.ceil(abs(distance)/.035),math.ceil(abs(curvature*distance)/.035))
        for _ in range(count):
            state = advance(state,distance/count,curvature)
            path.append((state,distance/count,curvature))
    return state,path


def maneuver_connection(start, target, check=lambda s:True, health=lambda:None):
    def residual(state):
        bed = bed_pose(state)
        return [bed[0]-target[0],bed[1]-target[1],
                1.5*angle(bed[2]-target[2]),angle(state[2]-target[2])]

    def outside_bay(state):
        # Same 2 x 1.35 m visual bay used by Component Manager, with a buffer.
        x,y,_ = bed_pose(state)
        dx,dy = x-target[0],y-target[1]
        c,s = math.cos(target[2]),math.sin(target[2])
        return abs(c*dx+s*dy)>1.02 or abs(-s*dx+c*dy)>.70

    norm = lambda values:sum(v*v for v in values)

    def fit(initial, params, directions, staged):
        count = len(params)
        for iteration in range(100):
            if iteration%10 == 0:
                health()
            state,path = arc_path(initial,params)
            values = residual(state)
            error = norm(values)
            tail_count = max(2,math.ceil(abs(params[-1])/.035))
            entry = path[-tail_count-1][0]
            if (error < .00015 and abs(angle(entry[2]-target[2])) < .025
                    and abs(angle(entry[3]-target[2])) < .03
                    and all(check(q) for q,_,_ in path)
                    and (not staged or all(outside_bay(q) for q,_,_ in path[:-tail_count]))):
                return path
            jacobian = []
            for j in range(count):
                trial = params[:]
                trial[j] += .001
                result = residual(arc_path(initial,trial)[0])
                jacobian.append([(a-b)/.001 for a,b in zip(result,values)])
            # Hold the final curvature at zero: alignment happens before entry.
            jacobian[-2] = [0.]*4
            solution = solve([[sum(j[i]*j[k] for j in jacobian)+(.002 if i==k else 0.)
                               for k in range(4)] for i in range(4)],values)
            if solution is None:
                break
            delta = [sum(j[k]*solution[k] for k in range(4)) for j in jacobian]
            found = False
            for alpha in (1.,.5,.2,.05):
                trial = []
                for i in range(count):
                    if i%2 == 0:
                        lower,upper = -2.,2.
                    else:
                        minimum = 1.2 if i==count-1 else .02
                        maximum = 4. if i==count-1 else 6.
                        lower,upper = (minimum,maximum) if directions[i//2]>0 else (-maximum,-minimum)
                    trial.append(0. if i==count-2 else max(lower,min(upper,params[i]-alpha*delta[i])))
                if norm(residual(arc_path(initial,trial)[0])) < error:
                    params,found = trial,True
                    break
            if not found:
                break
        return None

    # A short straight pull-back, when needed, creates turning room BEFORE the
    # trailer enters the bay. The remaining approach is forward in one gear.
    for pullback in (0.,1.2,2.,2.4):
        initial,prefix = arc_path(start,[0.,-pullback]) if pullback else (start,[])
        if not all(check(q) for q,_,_ in prefix):
            continue
        distance = math.dist(bed_pose(initial)[:2],target[:2])
        heading = angle(math.atan2(target[1]-bed_pose(initial)[1],target[0]-bed_pose(initial)[0])-initial[2])
        for seed in (heading,-heading,.7,-.7,1.4,-1.4):
            params = [seed,distance*.25,-seed,distance*.25,-seed,distance*.25,
                      seed,distance*.25,0.,max(1.2,distance*.2)]
            path = fit(initial,params,(1,1,1,1,1),True)
            if path is not None:
                return prefix+path

    # Where there is no room for the forward staging approach, permit a planned
    # straight reverse entry. This is one approach, not repeated goal chasing.
    distance = math.dist(bed_pose(start)[:2],target[:2])
    heading = angle(math.atan2(target[1]-bed_pose(start)[1],target[0]-bed_pose(start)[0])-start[2])
    for directions in ((1,1,1,1),(1,1,1,-1),(-1,1,1,1),(-1,-1,1,1)):
        for seed in (heading,-heading,.7,-.7,1.4,-1.4):
            initial = max(.1,distance*.3)
            params = [seed,initial*directions[0],-seed,initial*directions[1],
                      seed,initial*directions[2],0.,max(1.2,distance*.2)*directions[3]]
            path = fit(start,params,directions,False)
            if path is not None:
                return path
    return None


def solve(matrix, rhs):
    a=[list(row)+[v] for row,v in zip(matrix,rhs)]
    for i in range(len(a)):
        k=max(range(i,len(a)),key=lambda k:abs(a[k][i]));a[i],a[k]=a[k],a[i]
        if abs(a[i][i])<1e-10:return None
        d=a[i][i];a[i]=[v/d for v in a[i]]
        for k in range(len(a)):
            if k!=i:
                d=a[k][i];a[k]=[x-d*y for x,y in zip(a[k],a[i])]
    return [r[-1] for r in a]
