"""Shared geometry fixtures for planner and navigation tests."""
import math
from wheelchair_control.protocol import Packet, STOP, FORWARD
from wheelchair_control.planner import Config
from wheelchair_control.geometry import IndexedObstacles

BOUND=[(-10,-10),(10,-10),(10,10),(-10,10)]
def box(x0,x1,y0,y1):return [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
def config():return Config(footprint=((-0.45,-.30),(.55,-.30),(.55,.30),(-.45,.30)),margin=.06,path_heading_deadband=math.radians(.8))
def walls(width,length=7):return IndexedObstacles([box(-1,length,width/2,width/2+.12),box(-1,length,-width/2-.12,-width/2)])
def side_door_map(both_sides=False,blocked=False):
    polygons=[box(-1,1.,-.57,-.45),box(2.,6.,-.57,-.45)]
    polygons+=([box(-1,1.,.45,.57),box(2.,6.,.45,.57)] if both_sides else [box(-1,6.,.45,.57)])
    if blocked:polygons.append(box(1.4,1.5,-.3,.3))
    return IndexedObstacles(polygons)


def nav_authorize(core,state,obs,kind=4):
    core.intent(Packet('manager',1,0.,.25,STOP,False),True)
    core.tick(state,obs,BOUND,0.)
    assert core.ready
    p=Packet('manager',2,0.,.25,FORWARD,True,task_id='nav',task_kind=kind,
             reference_x=state.x,reference_y=state.y,reference_yaw=state.yaw)
    core.intent(p)
    return p
