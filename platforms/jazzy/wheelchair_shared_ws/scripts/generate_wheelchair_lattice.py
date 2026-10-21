#!/usr/bin/env python3
"""Generate Humble Smac JSON with forward lines and in-place rotations only."""
import json
import math
from pathlib import Path


def generate(resolution=.05):
    vectors=[(1,0),(2,1),(1,1),(1,2),(0,1),(-1,2),(-1,1),(-2,1),
             (-1,0),(-2,-1),(-1,-1),(-1,-2),(0,-1),(1,-2),(1,-1),(2,-1)]
    angles=[math.atan2(y,x)%(2*math.pi) for x,y in vectors];primitives=[]
    def add(start,end,poses,length,left=True):
        primitives.append(dict(trajectory_id=len(primitives),start_angle_index=start,end_angle_index=end,
            left_turn=left,trajectory_radius=0.,trajectory_length=length,arc_length=0.,
            straight_length=length,poses=poses))
    for i,(x,y) in enumerate(vectors):
        # One grid step keeps the lattice connected; longer straight primitives
        # reduce search depth. Consecutive straight segments merge for execution.
        for scale in (1,3):
            dx,dy=x*resolution*scale,y*resolution*scale;length=math.hypot(dx,dy)
            count=max(1,math.ceil(length/.02))
            add(i,i,[[dx*j/count,dy*j/count,angles[i]] for j in range(1,count+1)],length)
        for sign in (-1,1):
            end=(i+sign)%len(angles)
            delta=math.atan2(math.sin(angles[end]-angles[i]),math.cos(angles[end]-angles[i]))
            count=math.ceil(abs(delta)/.02)
            add(i,end,[[0.,0.,(angles[i]+delta*j/count)%(2*math.pi)] for j in range(1,count+1)],0.,sign>0)
    return dict(version=1,date_generated='2026-10-16',lattice_metadata=dict(motion_model='diff',
        turning_radius=.2,grid_resolution=resolution,stopping_threshold=5,num_of_headings=len(angles),
        heading_angles=angles,number_of_trajectories=len(primitives)),primitives=primitives)


if __name__=='__main__':
    target=Path(__file__).resolve().parents[1]/'src/wheelchair_nav2/config/wheelchair_lattice.json'
    target.write_text(json.dumps(generate(),indent=2)+'\n');print(target)
