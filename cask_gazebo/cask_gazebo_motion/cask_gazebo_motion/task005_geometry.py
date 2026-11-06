import math

def angle(a):
    return math.atan2(math.sin(a), math.cos(a))



def rotate(vector, quaternion, inverse=False):
    """Quaternion rotation, including measured chassis roll and pitch."""
    x,y,z,w = quaternion
    if inverse:
        x,y,z = -x,-y,-z
    vx,vy,vz = vector
    tx,ty,tz = 2*(y*vz-z*vy),2*(z*vx-x*vz),2*(x*vy-y*vx)
    return (vx+w*tx+y*tz-z*ty,vy+w*ty+z*tx-x*tz,vz+w*tz+x*ty-y*tx)



def rectangles_overlap(first, second, margin=0.):
    """Separating-axis test for two oriented rectangles (x,y,yaw,xmin,xmax,ymin,ymax)."""
    def corners(rect, padding):
        x,y,yaw,x0,x1,y0,y1 = rect
        c,s = math.cos(yaw),math.sin(yaw)
        return [(x+c*a-s*b,y+s*a+c*b) for a,b in
                ((x0-padding,y0-padding),(x0-padding,y1+padding),
                 (x1+padding,y0-padding),(x1+padding,y1+padding))]
    a,b = corners(first,margin),corners(second,0.)
    for yaw in (first[2],second[2]):
        for axis in ((math.cos(yaw),math.sin(yaw)),(-math.sin(yaw),math.cos(yaw))):
            pa = [x*axis[0]+y*axis[1] for x,y in a]
            pb = [x*axis[0]+y*axis[1] for x,y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True

