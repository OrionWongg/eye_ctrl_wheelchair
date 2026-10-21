#!/usr/bin/env python3
"""Compare the optional Smac interface with the established simulation flows."""
import argparse
import json
from pathlib import Path
from validate_directional import run_case
from validate_office_directional import exercise as office_exercise
from validate_turn_adjustment import exercise as adjustment_exercise


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--case',default='all',choices=['all','cruise_stop_resume','parked_left','parked_right','parked_uturn','adjust_uturn','adjust_blocked','smac_detour'])
    parser.add_argument('--output',default='artifacts/smac-lattice-20261016/simulation')
    args=parser.parse_args();root=Path(args.output);root.mkdir(parents=True,exist_ok=True);results=[]
    cases=['cruise_stop_resume','parked_left','parked_right','parked_uturn','adjust_uturn','adjust_blocked','smac_detour']
    for name in cases if args.case=='all' else [args.case]:
        scene='clearpath_office';exercise=office_exercise(name)
        if name=='smac_detour':
            scene='path_obstacle'
            def exercise(h):
                d=h.data;h.event(1)
                assert h.spin(30,lambda:d['pose'][0]>2.),d['status'].reason
                assert max(abs(p[1]) for p in d['samples'])>.3,'no detour around obstacle'
                h.event(5);assert h.spin(3,h.stopped)
                assert not d.get('runtime_faults'),str(d.get('runtime_faults'))
        if name.startswith('adjust_'):
            blocked=name=='adjust_blocked';scene='turn_adjust_blocked' if blocked else 'turn_adjust_bay'
            exercise=adjustment_exercise('blocked' if blocked else 'uturn',6)
        r=run_case(name,root,scene_override=scene,exercise=exercise,launch_args=['path_planner:=smac_lattice'])
        log=(root/name/'gazebo.log').read_text()
        import re
        r['smac_clean_shutdown']=not bool(re.search(r'\[smac_lattice_interface-[^]]+\]: process has died',log))
        if not r['smac_clean_shutdown']:r['status']='FAIL';r['error']='SMAC_PROCESS_EXIT_FAILURE'
        results.append(r);print(json.dumps(r),flush=True)
    (root/'result.json').write_text(json.dumps(results,indent=2)+'\n')
    return 0 if all(r['status']=='PASS' for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
