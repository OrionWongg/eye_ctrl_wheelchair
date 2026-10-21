#!/usr/bin/env python3
"""Validate directional navigation in the unchanged imported Clearpath office.

Uses the normal origin spawn and production parameters. Tests express user
intent, never target scene coordinates or alter the map/controller.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from validate_directional import run_case

CASES=('cruise_stop_resume','left_intent','right_intent','uturn','left_route','junction_resume','corridor_exit','parked_left','parked_right','parked_uturn','parked_replace')


def exercise(name):
    def run(h):
        d=h.data
        def status():return d['status'].phase+':'+d['status'].reason
        if name.startswith('parked_'):
            initial=d['pose'];first=len(d['samples'])
            kind=2 if name in ('parked_left','parked_replace') else 3 if name=='parked_right' else 6
            h.event(kind)
            if name=='parked_replace':
                assert h.spin(6,lambda:d['w']>.1),'left turn did not start'
                h.event(3)
            assert h.spin(35,lambda:d['status'].reason=='NAV_TURN_COMPLETED' and h.stopped()),'stationary turn did not finish: '+status()
            angle=math.atan2(math.sin(d['pose'][2]-initial[2]),math.cos(d['pose'][2]-initial[2]))
            if name!='parked_replace':
                target=math.pi if kind==6 else (1 if kind==2 else -1)*math.radians(20)
                assert abs(math.atan2(math.sin(angle-target),math.cos(angle-target)))<.04,(angle,target)
            assert max(math.dist(p[:2],initial[:2]) for p in d['samples'][first:])<.03,'translation during stationary turn'
            finished=d['pose'];h.spin(1.)
            assert h.stopped() and math.dist(finished[:2],d['pose'][:2])<.01,'auto-resumed motion'
            h.event(1)
            assert h.spin(6,lambda:math.dist(finished[:2],d['pose'][:2])>.2),'W did not start in new direction'
            h.event(5);assert h.spin(3,h.stopped),'STOP failed'
            assert not d.get('runtime_faults'),str(d.get('runtime_faults'))
            return
        h.event(1)
        assert h.spin(6,lambda:abs(d['v'])>.1 or abs(d['w'])>.1),'START did not move: '+status()
        initial=d['pose']
        if name=='corridor_exit':
            h.event(2)
            assert h.spin(80,lambda:d['pose'][0]>10.6),'did not cross corridor exit: '+status()
            assert 'PASSAGE_END_OR_BRANCH' not in d['reasons'],'stopped at corridor topology boundary'
            assert abs(d['pose'][1]-1.05)<.3,'left the corridor axis'
        elif name=='junction_resume':
            h.event(2)
            assert h.spin(20,lambda:d['pose'][0]>2.4),'did not reach junction approach: '+status()
            h.event(5);assert h.spin(3,h.stopped),'approach STOP failed'
            before=d['pose'];h.spin(.7);h.event(1)
            assert h.spin(24,lambda:d['pose'][0]>3.8),'junction resume did not cross: '+status()
            assert d['pose'][0]-before[0]>1.,'insufficient junction progress'
        elif name=='cruise_stop_resume':
            assert h.spin(16,lambda:math.dist(initial[:2],d['pose'][:2])>=2.),'cruise did not advance 2 m: '+status()
            h.event(5);assert h.spin(3,h.stopped),'STOP did not brake'
            stopped=d['pose'];h.spin(.7)
            assert math.dist(stopped[:2],d['pose'][:2])<.03,'motion after STOP'
            h.event(1)
            assert h.spin(6,lambda:math.dist(stopped[:2],d['pose'][:2])>.2),'resume did not advance: '+status()
        elif name=='uturn':
            h.event(6)
            def completed():
                angle=math.atan2(math.sin(d['pose'][2]-initial[2]),math.cos(d['pose'][2]-initial[2]))
                return d['status'].phase=='CRUISE' and abs(abs(angle)-math.pi)<.04
            assert h.spin(35,completed),'U-turn did not complete: '+status()
            pose=d['pose']
            assert h.spin(6,lambda:math.dist(pose[:2],d['pose'][:2])>.2),'no forward motion after U-turn: '+status()
        else:
            sign=1 if name in ('left_intent','left_route') else -1
            h.event(2 if sign>0 else 3)
            def completed():
                angle=math.atan2(math.sin(d['pose'][2]-initial[2]),math.cos(d['pose'][2]-initial[2]))
                return d['status'].phase=='CRUISE' and sign*angle>.28
            assert h.spin(35,completed),'direction intent did not complete: '+status()
            pose=d['pose']
            distance=3. if name=='left_route' else .2
            duration=25 if name=='left_route' else 6
            assert h.spin(duration,lambda:math.dist(pose[:2],d['pose'][:2])>distance),'forward progress after intent not reached: '+status()
        h.event(5);assert h.spin(3,h.stopped),'final STOP did not brake'
        h.spin(1.)
        assert not d.get('runtime_faults'),str(d.get('runtime_faults'))
    return run


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=(*CASES,'all'),default='all')
    parser.add_argument('--output',default='artifacts/nav2-office')
    args=parser.parse_args();root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    model=Path('src/wheelchair_sim/models/clearpath_office')
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in model.rglob('*') if p.is_file()}
    results=[]
    for name in CASES if args.case=='all' else (args.case,):
        result=run_case(name,root,scene_override='clearpath_office',exercise=exercise(name))
        results.append(result);print(json.dumps(result),flush=True)
    report=dict(scene='clearpath_office',spawn='default origin; no teleport',controller_parameters='production defaults; no overrides',
        model_sha256=hashes,model_unchanged=all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in hashes.items()),results=results)
    (root/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if all(r['status']=='PASS' for r in results) and report['model_unchanged'] else 1

if __name__=='__main__':raise SystemExit(main())
