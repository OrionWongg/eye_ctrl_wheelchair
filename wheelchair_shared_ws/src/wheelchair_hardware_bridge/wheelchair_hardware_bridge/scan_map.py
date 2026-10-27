"""Rolling local occupancy from finite scan returns, with unknown space blocked.

This is local 2-D perception, not a complete-world map. ObstacleMap represents
both occupied and unobserved cells as forbidden polygons inside its boundary.
"""
import math
import uuid
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32, Polygon
from tf2_ros import Buffer, TransformListener, TransformException
from wheelchair_interfaces.msg import ObstacleMap
from wheelchair_control.nav2_transport import cell_rectangles

# Measured chassis outline in base_link (x_min, x_max, y_min, y_max).
BODY=(-.45,.55,-.30,.30)
# Extended self-echo exclusion zone: covers chassis structures that protrude
# beyond the declared body boundary (rear frame, handrail, etc.) whose laser
# returns land ~0.60 m behind robot centre.  Applied only to self_hit so that
# corner_body (free-space marking) stays conservatively bounded by BODY.
SELF_ECHO=(-.65,.55,-.35,.35)


def grid_index(x,y,resolution,size):
    """Integer lower-left cell of the rolling window centred on (x, y)."""
    return math.floor((x-size/2)/resolution),math.floor((y-size/2)/resolution)


def rasterize(ranges, angle_min, increment, range_min, range_max, pose,
              resolution=.1, size=10.):
    """Only finite measured rays establish free space; missing bins stay unknown.

All four cell corners must lie before returns in both neighbouring angle bins.
The measured chassis footprint is excluded from self-occupancy; obstacle hits
are applied last so a return is never erased by another ray or the footprint.
"""
    ranges=np.asarray(ranges,dtype=float)
    if len(ranges)<180 or increment<=0 or abs(len(ranges)*increment-2*math.pi)>.06:
        raise ValueError('expected full-circle scan')
    n=int(round(size/resolution))
    if not 0<resolution<=.2 or n<=0 or n*n>250000:raise ValueError('invalid map dimensions')
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    ix0,iy0=grid_index(x,y,resolution,size)
    origin=(ix0*resolution,iy0*resolution)
    # Neighbouring cells share corners: evaluate each corner once.
    wx,wy=np.meshgrid(origin[0]+np.arange(n+1)*resolution-x,
                      origin[1]+np.arange(n+1)*resolution-y)
    bx=c*wx+s*wy;by=-s*wx+c*wy
    valid=np.isfinite(ranges)&(ranges>=range_min)&(ranges<=range_max)
    angles=(np.arctan2(by,bx)-angle_min)%(2*math.pi)
    i=np.floor(angles/increment).astype(int)%len(ranges);j=(i+1)%len(ranges)
    corner_free=valid[i]&valid[j]&(np.hypot(bx,by)<np.minimum(ranges[i],ranges[j])-resolution)
    corner_body=(bx>=BODY[0])&(bx<=BODY[1])&(by>=BODY[2])&(by<=BODY[3])
    def all_corners(a):return a[:-1,:-1]&a[:-1,1:]&a[1:,:-1]&a[1:,1:]
    cells=np.full((n,n),-1,dtype=np.int8);cells[all_corners(corner_free)|all_corners(corner_body)]=0
    angles=angle_min+np.arange(len(ranges))[valid]*increment+yaw
    hx=x+ranges[valid]*np.cos(angles);hy=y+ranges[valid]*np.sin(angles)
    # Drop self-hits: returns from the chassis structure at the footprint
    # edge. The body-cell exclusion above only frees cells whose every
    # corner lies inside BODY, so edge cells hit by self-returns would
    # otherwise stay occupied forever and block all motion.
    hxb=c*(hx-x)+s*(hy-y);hyb=-s*(hx-x)+c*(hy-y)
    self_hit=(hxb>=SELF_ECHO[0])&(hxb<=SELF_ECHO[1])&(hyb>=SELF_ECHO[2])&(hyb<=SELF_ECHO[3])
    hx,hy=hx[~self_hit],hy[~self_hit]
    ix=np.floor((hx-origin[0])/resolution).astype(int)
    iy=np.floor((hy-origin[1])/resolution).astype(int)
    inside=(ix>=0)&(ix<n)&(iy>=0)&(iy<n)
    cells[iy[inside],ix[inside]]=100
    return cells,origin


def fuse(layers,ix0,iy0,n):
    """Combine recent (ix0, iy0, cells) layers into the window at (ix0, iy0).

Any return in any layer stays occupied; otherwise one free observation proves
the cell free. Cells outside every layer remain unknown.
"""
    occupied=np.zeros((n,n),dtype=bool);free=np.zeros((n,n),dtype=bool)
    for lx,ly,cells in layers:
        dx,dy=lx-ix0,ly-iy0;m=cells.shape[0]
        x0,x1=max(0,dx),min(n,dx+m);y0,y1=max(0,dy),min(n,dy+m)
        if x0>=x1 or y0>=y1:continue
        part=cells[y0-dy:y1-dy,x0-dx:x1-dx]
        occupied[y0:y1,x0:x1]|=part==100;free[y0:y1,x0:x1]|=part==0
    out=np.full((n,n),-1,dtype=np.int8);out[free]=0;out[occupied]=100
    return out


def clear_blind_band(cells,origin,resolution,pose,band):
    """Free UNKNOWN cells within band of the chassis sides and rear.

The sensors cannot observe this ring (self-occlusion and minimum range). The
front is excluded: it stays observable and must be proven free. Occupied
cells are never changed. Remaining unknown cells are at least band away.
"""
    if band<=0:return cells
    n=cells.shape[0];x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    # Cell-to-chassis distance: the minimum over cell corners against the body
    # rectangle and body corners against each cell box (both convex).
    wx,wy=np.meshgrid(origin[0]+np.arange(n+1)*resolution-x,
                      origin[1]+np.arange(n+1)*resolution-y)
    bx=c*wx+s*wy;by=-s*wx+c*wy
    corner=np.hypot(np.maximum(0,np.maximum(BODY[0]-bx,bx-BODY[1])),
                    np.maximum(0,np.maximum(BODY[2]-by,by-BODY[3])))
    distance=np.minimum.reduce([corner[:-1,:-1],corner[:-1,1:],corner[1:,:-1],corner[1:,1:]])
    lo_x=origin[0]+np.arange(n)*resolution;lo_y=origin[1]+np.arange(n)*resolution
    for px,py in ((BODY[0],BODY[2]),(BODY[0],BODY[3]),(BODY[1],BODY[2]),(BODY[1],BODY[3])):
        ox,oy=x+c*px-s*py,y+s*px+c*py
        dx=np.maximum(0,np.maximum(lo_x-ox,ox-(lo_x+resolution)))
        dy=np.maximum(0,np.maximum(lo_y-oy,oy-(lo_y+resolution)))
        distance=np.minimum(distance,np.hypot(dx[None,:],dy[:,None]))
    cx,cy=np.meshgrid(lo_x+resolution/2-x,lo_y+resolution/2-y)
    behind_front=c*cx+s*cy<=BODY[1]
    cells=cells.copy();cells[(cells==-1)&(distance<band)&behind_front]=0
    return cells


def polygon(points):
    return Polygon(points=[Point32(x=float(x),y=float(y),z=0.) for x,y in points])


class ScanMap(Node):
    def __init__(self):
        super().__init__('scan_map')
        self.resolution=float(self.declare_parameter('resolution',.05).value)
        self.size=float(self.declare_parameter('size',10.).value)
        self.max_age=float(self.declare_parameter('max_age',.3).value)
        # Free space older than history_window is dropped: a person stepping
        # into a cell after it was observed free must not stay hidden.
        self.history_window=float(self.declare_parameter('history_window',.25).value)
        self.blind_band=float(self.declare_parameter('blind_band',.10).value)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.session=uuid.uuid4().hex;self.sequence=0;self.pending=None
        self.layers=deque();self.last_stamp=None
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub=self.create_publisher(OccupancyGrid,'/wheelchair/planning_map',qos)
        self.map_pub=self.create_publisher(ObstacleMap,'/wheelchair/obstacle_map',1)
        self.create_subscription(LaserScan,'/wheelchair/scan',self.receive,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        # Scans are processed on arrival; the timer only retries a scan whose
        # TF was not yet available.
        self.create_timer(.01,self.publish)

    def receive(self,msg):
        self.pending=msg;self.publish()

    def publish(self):
        msg=self.pending
        if msg is None:return
        stamp=Time.from_msg(msg.header.stamp)
        age=(self.get_clock().now()-stamp).nanoseconds/1e9
        if not -.05<=age<self.max_age or (self.last_stamp is not None and stamp<=self.last_stamp):
            self.pending=None;return
        try:
            t=self.tf.lookup_transform('odom',msg.header.frame_id,stamp).transform
        except TransformException as exc:
            self.get_logger().warning(f'Waiting for system-time odom/scan TF: {exc}',throttle_duration_sec=5.)
            return
        self.pending=None;self.last_stamp=stamp
        q=t.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        pose=(t.translation.x,t.translation.y,yaw)
        try:
            cells,origin=rasterize(msg.ranges,msg.angle_min,msg.angle_increment,
                                   msg.range_min,msg.range_max,pose,self.resolution,self.size)
        except ValueError as exc:
            self.get_logger().error(str(exc));return
        ix0,iy0=grid_index(pose[0],pose[1],self.resolution,self.size)
        seconds=stamp.nanoseconds/1e9
        while self.layers and seconds-self.layers[0][0]>self.history_window:self.layers.popleft()
        self.layers.append((seconds,ix0,iy0,cells))
        cells=fuse([layer[1:] for layer in self.layers],ix0,iy0,cells.shape[0])
        cells=clear_blind_band(cells,origin,self.resolution,pose,self.blind_band)
        grid=OccupancyGrid();grid.header=msg.header;grid.header.frame_id='odom'
        grid.info.map_load_time=msg.header.stamp;grid.info.resolution=self.resolution
        grid.info.width=grid.info.height=cells.shape[0]
        grid.info.origin.position.x=origin[0];grid.info.origin.position.y=origin[1]
        grid.info.origin.orientation.w=1.;grid.data=cells.ravel().tolist()
        out=ObstacleMap();out.header=grid.header;out.session_id=self.session
        self.sequence+=1;out.sequence=self.sequence
        width=cells.shape[0]*self.resolution;x,y=origin
        out.known_free_boundary=polygon(((x,y),(x+width,y),(x+width,y+width),(x,y+width)))
        out.obstacles=[polygon(rect) for rect in cell_rectangles(
            cells,grid.info.width,grid.info.height,self.resolution,origin)]
        # Complete conservative representation WITHIN this bounded local grid:
        # all unobserved cells are obstacles; outside the boundary is forbidden.
        out.complete=True
        self.grid_pub.publish(grid);self.map_pub.publish(out)


def main(args=None):
    rclpy.init(args=args);node=ScanMap()
    try:rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
