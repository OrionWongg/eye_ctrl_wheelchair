"""ROS adapters for Nav2 maps and the asynchronous discrete path service."""
import math
import time
import numpy as np
from .protocol import Packet,StreamGuard
from .geometry import IndexedPolygon


def occupied_rectangles(data,width,height,resolution,origin):
    """Compress lethal/unknown raster cells to exact cell-area rectangles."""
    return [IndexedPolygon(r) for r in cell_rectangles(data,width,height,resolution,origin)]


def cell_rectangles(data,width,height,resolution,origin):
    """Vertex tuples of occupied_rectangles without collision indexing."""
    cells=np.asarray(data,dtype=np.int16).reshape(height,width)
    occupied=(cells<0)|(cells>=100)
    # Extract all row runs in one native array operation; avoid allocating
    # padded arrays and scanning twice per row in the safety node callback.
    edges=np.diff(np.pad(occupied,((0,0),(1,1))).astype(np.int8),axis=1)
    starts_y,starts_x=np.nonzero(edges==1)
    _,ends_x=np.nonzero(edges==-1)
    offsets=np.r_[0,np.cumsum(np.bincount(starts_y,minlength=height))]
    active={};rectangles=[]
    for row in range(height+1):
        spans=[]
        if row<height:
            spans=list(zip(starts_x[offsets[row]:offsets[row+1]],ends_x[offsets[row]:offsets[row+1]]))
        current=set(spans)
        for interval,start in list(active.items()):
            if interval not in current:
                x0,x1=interval;y0,y1=start,row
                a,b=origin[0]+x0*resolution,origin[0]+x1*resolution
                c,d=origin[1]+y0*resolution,origin[1]+y1*resolution
                rectangles.append(((a,c),(b,c),(b,d),(a,d)))
                del active[interval]
        for interval in spans:active.setdefault(interval,row)
    return rectangles


class Nav2PathClient:
    def __init__(self,node):
        from wheelchair_interfaces.srv import ComputeDiscretePath
        from wheelchair_interfaces.msg import InputHealth
        from nav_msgs.msg import Path,OccupancyGrid
        from rclpy.qos import QoSProfile,DurabilityPolicy
        self.node=node;self.kind=ComputeDiscretePath;self.counter=0
        self.backend=node.get_parameter('path_planner_backend').value
        self.client=node.create_client(ComputeDiscretePath,'/wheelchair/compute_discrete_path')
        self.guard=StreamGuard(.5);self.valid=False;self.session=None;self.changed=False
        self.grid_at=-math.inf;self.grid_stamp=-math.inf;self.obstacles=[]
        self.grid_signature=None;self.grid_compute_ms=0.
        self.requests={}
        self.last_report={}
        self.plan_pub=node.create_publisher(Path,'/wheelchair/local_plan',1)
        node.create_subscription(InputHealth,'/wheelchair/path_planner_health',self.health,1)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid,'/wheelchair/local_costmap/costmap',self.grid,qos)

    def ros_now(self):return self.node.get_clock().now().nanoseconds/1e9
    def health(self,m):
        now=time.monotonic();stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        ok,changed,_=self.guard.accept(Packet(m.source_session_id,m.sequence,stamp,m.ttl_ms/1000),self.ros_now(),now)
        if changed and self.session is not None:self.changed=True
        self.session=m.source_session_id
        self.valid=ok and m.source_valid and m.protocol_version==1 and math.isfinite(m.source_age_ms) and 0<=m.source_age_ms<500

    def grid(self,m):
        now=time.monotonic();stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        q=m.info.origin.orientation;p=m.info.origin.position
        if (m.header.frame_id!='odom' or not 0<m.info.resolution<=.2 or
            m.info.width==0 or m.info.height==0 or
            len(m.data)!=m.info.width*m.info.height or m.info.width*m.info.height>2_000_000 or
            not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w,stamp)) or
            abs(q.x)+abs(q.y)+abs(q.z)>1e-6 or abs(q.w-1)>1e-6):
            self.grid_at=-math.inf;return
        if not -.05<=self.ros_now()-stamp<self.node.cfg.costmap_timeout:return
        # Soft costs do not change the hard collision geometry. Reuse exact
        # rectangles while the occupied mask and grid placement are unchanged.
        occupied=(np.asarray(m.data)<0)|(np.asarray(m.data)>=100)
        signature=(m.info.width,m.info.height,m.info.resolution,p.x,p.y,occupied.tobytes())
        if signature!=self.grid_signature:
            self.obstacles=occupied_rectangles(m.data,m.info.width,m.info.height,m.info.resolution,(p.x,p.y))
            self.grid_signature=signature
        self.grid_compute_ms=(time.monotonic()-now)*1000
        self.grid_at=now;self.grid_stamp=stamp

    def healthy(self,now):
        return (self.valid and self.guard.fresh(now) and now-self.grid_at<self.node.cfg.costmap_timeout
                and -.05<=self.ros_now()-self.grid_stamp<self.node.cfg.costmap_timeout)

    def request(self,task_id,start,goals,max_length,allow_detour=True):
        from geometry_msgs.msg import Pose2D
        self.counter+=1;token=self.counter
        m=self.kind.Request();m.header.frame_id='odom';m.header.stamp=self.node.get_clock().now().to_msg()
        m.task_id=task_id;m.request_id=token;m.start=Pose2D(x=float(start[0]),y=float(start[1]),theta=float(start[2]))
        m.goals=[Pose2D(x=float(p[0]),y=float(p[1]),theta=float(p[2])) for p in goals]
        # No corridor band: the service field stays zero (unconstrained).
        m.max_path_length=float(max_length);m.corridor_half_width=0.
        m.corridor_origin=Pose2D(x=0.,y=0.,theta=0.)
        m.allow_detour=allow_detour
        self.last_report=dict(request_id=token,task_id=task_id,start=start,goals=goals,
            frame_id=m.header.frame_id,max_path_length=max_length,allow_detour=allow_detour,
            request_stamp=self.ros_now(),
            costmap_stamp=self.grid_stamp,reason='REQUESTED')
        self.requests[token]=(self.client.call_async(m),time.monotonic(),task_id)
        return token

    def poll(self,token):
        row=self.requests.get(token)
        if row is None:return 'FAILED','CANCELLED'
        future,born,task_id=row
        if time.monotonic()-born>self.node.cfg.path_timeout:
            self.cancel(token);return 'FAILED','PATH_SERVICE_TIMEOUT'
        if not future.done():return 'WAIT',None
        self.requests.pop(token,None)
        try:r=future.result()
        except Exception:return 'FAILED','PATH_SERVICE_ERROR'
        if r.task_id!=task_id or r.request_id!=token or r.header.frame_id!='odom':return 'FAILED','INVALID_PATH_RESPONSE'
        self.last_report.update(reason=r.reason,expansions=r.expansions,compute_ms=r.compute_ms,
            costmap_stamp=r.costmap_stamp.sec+r.costmap_stamp.nanosec/1e9)
        if not r.success:return 'FAILED',r.reason
        poses=[]
        for point in r.path.poses:
            p=point.pose.position;q=point.pose.orientation
            values=(p.x,p.y,q.x,q.y,q.z,q.w)
            if point.header.frame_id!='odom' or not all(math.isfinite(x) for x in values) or abs(sum(x*x for x in values[2:])-1)>.01:
                return 'FAILED','INVALID_PATH_POSE'
            poses.append((p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
        if not poses:return 'FAILED','EMPTY_PATH'
        self.plan_pub.publish(r.path)
        return 'READY',poses

    def cancel(self,token):
        row=self.requests.pop(token,None)
        if row is not None:row[0].cancel()


def map_main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from wheelchair_interfaces.msg import ObstacleMap
    from nav_msgs.msg import OccupancyGrid
    import shapely
    from shapely.geometry import Polygon
    rclpy.init();node=Node('navigation_map_adapter')
    resolution=float(node.declare_parameter('resolution',.05).value)
    pub=node.create_publisher(OccupancyGrid,'/wheelchair/planning_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    signature=[None]
    def receive(m):
        if not m.complete or m.header.frame_id!='odom':return
        boundary=tuple((p.x,p.y) for p in m.known_free_boundary.points)
        obstacles=tuple(tuple((p.x,p.y) for p in poly.points) for poly in m.obstacles)
        current=(boundary,obstacles)
        if current==signature[0]:return
        shape=Polygon(boundary)
        if not shape.is_valid or shape.area<=0:return
        minx,miny,maxx,maxy=shape.bounds
        minx=math.floor(minx/resolution)*resolution;miny=math.floor(miny/resolution)*resolution
        width=math.ceil((maxx-minx)/resolution);height=math.ceil((maxy-miny)/resolution)
        if width*height>2_000_000:raise ValueError('Map too large')
        cells=np.zeros((height,width),dtype=np.int8)
        xx,yy=np.meshgrid(minx+(np.arange(width)+.5)*resolution,miny+(np.arange(height)+.5)*resolution)
        known=np.ones_like(cells,dtype=bool)
        for dx,dy in ((-.5,-.5),(-.5,.5),(.5,-.5),(.5,.5)):
            known &= shapely.covers(shape,shapely.points(xx+dx*resolution,yy+dy*resolution))
        cells[~known]=-1
        for poly in obstacles:
            s=Polygon(poly);x0,y0,x1,y1=s.bounds
            a=max(0,int(math.floor((x0-minx)/resolution)));b=min(width,int(math.ceil((x1-minx)/resolution)))
            c=max(0,int(math.floor((y0-miny)/resolution)));d=min(height,int(math.ceil((y1-miny)/resolution)))
            if a>=b or c>=d:continue
            if abs(s.area-(x1-x0)*(y1-y0))<1e-8:cells[c:d,a:b]=100
            else:
                xs,ys=xx[c:d,a:b],yy[c:d,a:b]
                hit=shapely.intersects(s,shapely.box(xs-resolution/2,ys-resolution/2,xs+resolution/2,ys+resolution/2))
                cells[c:d,a:b][hit]=100
        out=OccupancyGrid();out.header=m.header;out.info.map_load_time=m.header.stamp
        out.info.resolution=resolution;out.info.width=width;out.info.height=height
        out.info.origin.position.x=minx;out.info.origin.position.y=miny;out.info.origin.orientation.w=1.
        out.data=cells.ravel().tolist();pub.publish(out);signature[0]=current
    node.create_subscription(ObstacleMap,'/wheelchair/obstacle_map',receive,1)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
