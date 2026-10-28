#!/usr/bin/env python3
"""Exercise the real Humble Smac plugin through its new service; no chassis node."""
import json
import math
import os
from pathlib import Path
import subprocess
import time
from process_cleanup import stop_group


def main():
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid,Path as NavPath
    from wheelchair_interfaces.msg import InputHealth
    from wheelchair_interfaces.srv import ComputeDiscretePath
    from geometry_msgs.msg import Pose2D
    from wheelchair_control.path_tracking import segments
    from wheelchair_control.geometry import transform,clearance
    from wheelchair_control.nav2_transport import occupied_rectangles
    from wheelchair_control.planner import Config
    cfg=Config(footprint=((-.45,-.30),(.55,-.30),(.55,.30),(-.45,.30)))
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='artifacts/smac-lattice-20261016/interface')
    root=Path(parser.parse_args().output);root.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']='96';os.environ['ROS_LOCALHOST_ONLY']='1'
    log=(root/'planner.log').open('w')
    command=['ros2','run','wheelchair_nav2','smac_lattice_interface','--ros-args',
        '--params-file','src/wheelchair_control/config/narrow_control.yaml',
        '--params-file','src/wheelchair_nav2/config/smac_lattice.yaml',
        '-p','use_sim_time:=false','-p','Lattice.lattice_filepath:='+str(Path('src/wheelchair_nav2/config/wheelchair_lattice.json').resolve())]
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('smac_interface_validation')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub=node.create_publisher(OccupancyGrid,'/wheelchair/local_costmap/costmap',qos)
    health=[];raw=[]
    node.create_subscription(InputHealth,'/wheelchair/smac/path_planner_health',lambda m:health.append(m),1)
    node.create_subscription(NavPath,'/wheelchair/smac/raw_plan',lambda m:raw.append(m),1)
    client=node.create_client(ComputeDiscretePath,'/wheelchair/smac/compute_discrete_path')
    grid=OccupancyGrid();grid.header.frame_id='odom';grid.info.resolution=.05;grid.info.width=grid.info.height=200
    grid.info.origin.position.x=grid.info.origin.position.y=-5.;grid.info.origin.orientation.w=1.
    grid.data=[0]*40000;publishing=True;request_id=0
    def publish():
        if publishing:grid.header.stamp=node.get_clock().now().to_msg();pub.publish(grid)
    node.create_timer(.1,publish)
    def spin(duration,predicate):
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError('Smac interface exited; see planner.log')
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return True
        return False
    def call(name,start,goal,detour=True,max_length=6.,old=False):
        nonlocal request_id
        request_id+=1;r=ComputeDiscretePath.Request();r.header.frame_id='odom';r.header.stamp=node.get_clock().now().to_msg()
        if old:r.header.stamp.sec-=2
        r.task_id=name;r.request_id=request_id;r.start=Pose2D(x=float(start[0]),y=float(start[1]),theta=float(start[2]))
        r.goals=[Pose2D(x=float(goal[0]),y=float(goal[1]),theta=float(goal[2]))];r.max_path_length=max_length;r.allow_detour=detour
        future=client.call_async(r);assert spin(3,future.done),name+' timed out';m=future.result()
        assert m.task_id==name and m.request_id==request_id
        poses=[(p.pose.position.x,p.pose.position.y,math.atan2(2*p.pose.orientation.w*p.pose.orientation.z,1-2*p.pose.orientation.z**2)) for p in m.path.poses]
        (root/(name+'-output.json')).write_text(json.dumps(poses)+'\n')
        if m.success:
            parts=segments(poses)
            assert parts or math.dist(start[:2],goal[:2])<1e-5
            assert math.dist(poses[0][:2],start[:2])<1e-6 and math.dist(poses[-1][:2],goal[:2])<1e-6
            assert abs(math.atan2(math.sin(poses[-1][2]-goal[2]),math.cos(poses[-1][2]-goal[2])))<1e-6
            obstacles=occupied_rectangles(grid.data,200,200,.05,(-5.,-5.))
            for pose in poses:
                assert clearance(transform(cfg.footprint,pose),obstacles,[(-5,-5),(5,-5),(5,5),(-5,5)],cfg.padding)>cfg.padding
        spin(.05,lambda:False)
        if raw:
            native=[(v.pose.position.x,v.pose.position.y,math.atan2(2*v.pose.orientation.w*v.pose.orientation.z,1-2*v.pose.orientation.z**2)) for v in raw[-1].poses]
            (root/(name+'-raw.json')).write_text(json.dumps(native)+'\n')
        item=dict(case=name,success=m.success,reason=m.reason,compute_ms=m.compute_ms,poses=poses)
        results.append(item);(root/'partial-results.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps({k:v for k,v in item.items() if k!='poses'}),flush=True)
        return m
    results=[]
    try:
        assert spin(12,lambda:health and health[-1].source_valid and client.service_is_ready()),'Smac startup not ready'
        assert call('straight',(0,0,0),(1,0,0)).success
        assert call('uturn',(0,0,0),(0,0,math.pi)).success
        assert call('off_grid_heading',(.013,.017,.2),(1.037,.113,.349)).success
        assert call('locked_heading',(0,0,0),(0,0,.349),False).success
        assert not call('expired_request',(0,0,0),(1,0,0),old=True).success
        assert not call('short_budget',(0,0,0),(1,0,0),max_length=.2).success
        cells=list(grid.data)
        for y in range(95,105):
            for x in range(120,124):cells[y*200+x]=100
        grid.data=cells;spin(.4,lambda:False)
        detour=call('obstacle_detour',(0,0,0),(2.3,0,0))
        assert detour.success and detour.reason=='PLANNED'
        assert spin(.5,lambda:any(m.poses for m in raw)),'no native Smac raw plan was published'
        assert not call('locked_obstacle',(0,0,0),(2.3,0,0),False).success
        for y in range(200):
            for x in range(130,134):cells[y*200+x]=-1
        grid.data=cells;spin(.4,lambda:False)
        assert not call('unknown_wall',(0,0,0),(2.3,0,0)).success
        grid.info.resolution=.1;spin(.3,lambda:False)
        assert call('resolution_mismatch',(0,0,0),(.1,0,0)).reason=='COSTMAP_STALE'
        grid.info.resolution=.05;spin(.3,lambda:False)
        publishing=False;spin(.7,lambda:False)
        assert not call('stale_map',(0,0,0),(1,0,0)).success
        (root/'result.json').write_text(json.dumps(dict(status='PASS',cases=results,native_plans=sum(bool(m.poses) for m in raw)),indent=2)+'\n')
    finally:
        node.destroy_node();rclpy.shutdown();stop_group(process);log.close()


if __name__=='__main__':main()
