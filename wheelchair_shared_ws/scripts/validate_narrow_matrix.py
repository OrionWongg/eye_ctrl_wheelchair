#!/usr/bin/python3
"""Sequential real-Gazebo acceptance; do not run alongside another simulation."""
import argparse,json,subprocess,shutil
from pathlib import Path
root=Path(__file__).resolve().parents[1]
cases={
'passage080':['--scenario','passage','--test-scene','passage_080','--heading','0'],
'passage090':['--scenario','passage','--test-scene','passage_090','--heading','5'],
'passage100':['--scenario','passage','--test-scene','passage_100','--heading','-5'],
'passage110':['--scenario','passage','--test-scene','passage_110','--heading','10'],
'door090':['--scenario','door','--test-scene','door_090','--heading','5'],
'hard_reject':['--scenario','passage','--test-scene','passage_080','--heading','10','--expect-blocked'],
'exit_reject':['--scenario','door','--test-scene','door_blocked_exit','--heading','0','--expect-blocked'],
'office_original':['--scenario','passage','--test-scene','clearpath_office','--initial-x','2.8325045531343336','--lateral','.9422187278698693','--heading','21.43163051'],
'legacy_b1':['--scenario','b1'],
'legacy_assist':['--scenario','assist'],
}
p=argparse.ArgumentParser();p.add_argument('--cases',nargs='*',choices=cases);args=p.parse_args()
for name in args.cases or cases:
    print('RUN',name,flush=True);options=cases[name]
    completed=subprocess.run(['/usr/bin/python3',str(root/'scripts/validate_shared.py'),*options],capture_output=True,text=True)
    (root/'artifacts'/f'narrow-final-{name}.log').write_text(completed.stdout+completed.stderr)
    if completed.returncode:
        print(completed.stdout[-2000:]+completed.stderr[-2000:],flush=True);raise SystemExit(completed.returncode)
    scenario=options[options.index('--scenario')+1]
    for filename in (f'gazebo-{scenario}.log',f'trace-{scenario}.json'):
        source=root/'artifacts'/filename
        if source.exists():shutil.copyfile(source,root/'artifacts'/f'narrow-final-{name}-{filename}')
    data=json.load(open(root/'artifacts'/f'result-{scenario}.json'));data['case']=name
    (root/'artifacts'/f'narrow-final-{name}.json').write_text(json.dumps(data,indent=2)+'\n')
    results=[json.load(open(root/'artifacts'/f'narrow-final-{case}.json')) for case in cases if (root/'artifacts'/f'narrow-final-{case}.json').exists()]
    (root/'artifacts/narrow-final-matrix.json').write_text(json.dumps(results,indent=2)+'\n')
    print(name,data['status'],'travel=',data.get('travel_m'),'reject=',data.get('correctly_rejected'),'max_ms=',data.get('max_compute_ms'),flush=True)
