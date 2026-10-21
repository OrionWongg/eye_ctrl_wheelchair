"""Convex polygon collision, clearance and observed-boundary checks in metres."""
import math
import numpy as np
import shapely
from shapely.geometry import Polygon

def cross(a,b,c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
def edges(poly): return zip(poly,poly[1:]+poly[:1])
def convex(poly):
    if len(poly)<3 or any(not math.isfinite(v) for p in poly for v in p): return False
    signs=[cross(poly[i-1],poly[i],poly[(i+1)%len(poly)]) for i in range(len(poly))]
    return (all(s>=-1e-10 for s in signs) or all(s<=1e-10 for s in signs)) and any(abs(s)>1e-8 for s in signs)
def inside(point,poly):
    c=[cross(a,b,point) for a,b in edges(poly)]
    return all(x>=-1e-9 for x in c) or all(x<=1e-9 for x in c)
def point_segment(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]; length=dx*dx+dy*dy
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length)) if length else 0.
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
def overlaps(a,b):
    for poly in (a,b):
        for p,q in edges(poly):
            nx,ny=p[1]-q[1],q[0]-p[0]
            pa=[x*nx+y*ny for x,y in a]; pb=[x*nx+y*ny for x,y in b]
            if max(pa)<min(pb) or max(pb)<min(pa): return False
    return True
def distance(a,b):
    if overlaps(a,b): return 0.
    return min([point_segment(p,u,v) for p in a for u,v in edges(b)]+[point_segment(p,u,v) for p in b for u,v in edges(a)])
def transform(poly,pose):
    x,y,t=pose;c,s=math.cos(t),math.sin(t)
    return [(x+c*px-s*py,y+s*px+c*py) for px,py in poly]
def wrap(t): return math.atan2(math.sin(t),math.cos(t))
def clearance(poly,obstacles,boundary,margin):
    if any(not inside(p,boundary) for p in poly): return -1.
    gap=min(point_segment(p,a,b) for p in poly for a,b in edges(boundary))
    if gap<=margin: return gap
    # Cheap box separation skips exact distances for distant obstacles.
    ax0,ax1=min(p[0] for p in poly),max(p[0] for p in poly)
    ay0,ay1=min(p[1] for p in poly),max(p[1] for p in poly)
    if isinstance(obstacles,IndexedObstacles):
        b=obstacles.bounds
        if not len(b):return gap
        lower=np.hypot(np.maximum(0.,np.maximum(b[:,0]-ax1,ax0-b[:,1])),
                       np.maximum(0.,np.maximum(b[:,2]-ay1,ay0-b[:,3])))
        native_poly=Polygon(poly)
        nearest=int(np.argmin(lower))
        if lower[nearest]-1e-10<gap:
            gap=min(gap,float(shapely.distance(native_poly,obstacles.native[nearest])))
        if gap<=margin:return gap
        candidates=obstacles.native[lower-1e-10<gap]
        if len(candidates):
            # GEOS computes the same full polygon distance in native code. Keep
            # the scalar path below as an independent differential-test oracle.
            gap=min(gap,float(np.min(shapely.distance(native_poly,candidates))))
        return gap
    for obs in obstacles:
        bx0,bx1,by0,by1=polygon_bounds(obs)
        lower=math.hypot(max(0.,bx0-ax1,ax0-bx1),max(0.,by0-ay1,ay0-by1))
        if lower>=gap: continue
        gap=min(gap,distance(poly,obs))
        if gap<=margin: return gap
    return gap


def polygon_bounds(poly):
    cached=getattr(poly,'aabb',None)
    if cached is not None:return cached
    if not poly:return math.inf,-math.inf,math.inf,-math.inf
    return min(p[0] for p in poly),max(p[0] for p in poly),min(p[1] for p in poly),max(p[1] for p in poly)

class IndexedPolygon(tuple):
    """Immutable vertices with cached broad-phase bounds for large static maps."""
    def __new__(cls,points):
        obj=super().__new__(cls,points);obj.aabb=polygon_bounds(obj);obj.native=Polygon(obj);return obj

def nearby_obstacles(state,obstacles,cfg):
    # Deliberately overbound every rollout: initial momentum/braking plus twice
    # the complete assist horizon, footprint and an additional metre. Filtering
    # only removes obstacles whose bounding boxes cannot reach any candidate.
    velocity=max(abs(state.v),cfg.forward_speed)
    radius=(cfg.radius+cfg.padding+velocity*cfg.reaction+velocity**2/cfg.linear_brake
            +cfg.forward_speed*(2*cfg.assist_prediction+cfg.reaction+2*cfg.minimum_action
                               +2*cfg.action_ttl+2*cfg.transport_delay+cfg.startup_delay)+1.)
    selected=[]
    for obs in obstacles:
        x0,x1,y0,y1=polygon_bounds(obs)
        dx=max(0.,x0-state.x,state.x-x1);dy=max(0.,y0-state.y,state.y-y1)
        d2=dx*dx+dy*dy
        if d2<=radius*radius:selected.append((d2,obs))
    selected.sort(key=lambda p:p[0])
    return IndexedObstacles(p[1] for p in selected)


class IndexedObstacles(tuple):
    """Immutable map polygons with cached native geometry for rollout distances."""
    def __new__(cls,polygons):
        obj=super().__new__(cls,(p if isinstance(p,IndexedPolygon) else IndexedPolygon(p) for p in polygons))
        obj.bounds=np.asarray([p.aabb for p in obj],dtype=float).reshape((-1,4))
        obj.native=np.asarray([p.native for p in obj],dtype=object)
        obj.bounds.flags.writeable=False;obj.native.flags.writeable=False
        return obj


def collision_diagnostic(poly,obstacles,boundary,padding):
    from shapely.ops import nearest_points
    shape=Polygon(poly);edge=Polygon(boundary).boundary
    target=edge;identifier='known_free_boundary';gap=shape.distance(edge)
    if not Polygon(boundary).covers(shape):gap=-1.
    else:
        for i,obstacle in enumerate(obstacles):
            native=getattr(obstacle,'native',None)
            if native is None:native=Polygon(obstacle)
            d=shape.distance(native)
            if d<gap:gap=d;target=native;identifier=getattr(obstacle,'obstacle_id',f'local_obstacle_{i}')
    a,b=nearest_points(shape,target)
    return dict(obstacle_id=identifier,raw_clearance=gap,hard_padding=padding,
                body_point=list(a.coords[0]),obstacle_point=list(b.coords[0]))
