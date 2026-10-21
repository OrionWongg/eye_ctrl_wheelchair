#!/usr/bin/env python3
"""Validate forward-then-turn fallback with actual Gazebo geometry and Nav2."""
import json
import math
from pathlib import Path
from validate_directional import run_case


def exercise(name,kind):
    def run(h):
        d=h.data;origin=d['pose'];first=len(d['samples']);h.event(kind)
        if name=='blocked':
            assert h.spin(10,lambda:d['status'].reason.startswith('TURN_ADJUST_HELP_REQUIRED:') and h.stopped()),d['status'].reason
            assert max(math.dist(p[:2],origin[:2]) for p in d['samples'][first:])<.01,'moved without a feasible combination'
        elif name=='stop':
            assert h.spin(10,lambda:d['status'].phase=='TURN_ADJUST_EXECUTE' and d['v']>.04),'adjustment did not start'
            h.event(5);assert h.spin(3,h.stopped),'STOP failed'
        else:
            assert h.spin(30,lambda:d['status'].reason=='NAV_TURN_COMPLETED' and h.stopped()),d['status'].reason
            assert any(p.startswith('TURN_ADJUST_') for p in d['phase']),'did not use position adjustment'
            advance=d['pose'][0]-origin[0]
            assert .15<advance<=.66,advance
            assert abs(d['pose'][1]-origin[1])<.03
            target=math.pi if kind==6 else (1 if kind==2 else -1)*math.radians(20)
            error=math.atan2(math.sin(d['pose'][2]-origin[2]-target),math.cos(d['pose'][2]-origin[2]-target))
            assert abs(error)<.04,error
        end=d['pose'];h.spin(1.)
        assert h.stopped() and math.dist(end[:2],d['pose'][:2])<.01,'unexpected automatic cruise'
        assert not d.get('runtime_faults'),str(d.get('runtime_faults'))
        assert all(a in (0,1,2,3) for a in d.get('actions',[])),'reverse or invalid action'
    return run


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='artifacts/turn-adjustment-20261016/simulation')
    parser.add_argument('--case',choices=['left','right','uturn','blocked','stop','all'],default='all')
    args=parser.parse_args();root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    cases=[('left',2),('right',3),('uturn',6),('blocked',6),('stop',6)]
    results=[]
    for name,kind in cases:
        if args.case!='all' and name!=args.case:continue
        result=run_case(name,root,scene_override='turn_adjust_blocked' if name=='blocked' else 'turn_adjust_bay',exercise=exercise(name,kind))
        results.append(result);print(json.dumps(result),flush=True)
    (root/'result.json').write_text(json.dumps(results,indent=2)+'\n')
    raise SystemExit(0 if all(r['status']=='PASS' for r in results) else 1)
