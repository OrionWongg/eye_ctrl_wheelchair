#!/usr/bin/env python3
"""Verify the actual terminal keyboard displays the assistance-required notice."""
import json
import os
import subprocess
from pathlib import Path
from validate_directional import run_case
from process_cleanup import stop_group

root=Path('artifacts/turn-adjustment-20261016/accepted-keyboard');root.mkdir(parents=True,exist_ok=True)
log_path=root/'keyboard.txt'


def exercise(h):
    d=h.data;old_source=d['status'].source_session_id;d['health']=False
    with log_path.open('w') as log:
        process=subprocess.Popen(['script','-q','-c',str(Path('scripts/direction_keyboard.sh').resolve()),'/dev/null'],
            stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
            env=dict(os.environ,WHEELCHAIR_ROS_DOMAIN_ID='94'))
        try:
            assert h.spin(6,lambda:d['status'].source_session_id!=old_source and d['status'].can_start),'keyboard not ready'
            process.stdin.write(b'u');process.stdin.flush()
            assert h.spin(10,lambda:d['status'].reason.startswith('TURN_ADJUST_HELP_REQUIRED:') and h.stopped()),d['status'].reason
            h.spin(.5)
            expected='已停车：没有可安全完成的前进＋转向组合，需要协助。'
            assert expected in log_path.read_text(),'terminal did not display assistance notice'
            process.stdin.write(b'x');process.stdin.flush()
            assert h.spin(4,lambda:process.poll() is not None and h.stopped()),'keyboard exit failed'
        finally:
            stop_group(process)


if __name__=='__main__':
    result=run_case('keyboard_help',root,scene_override='turn_adjust_blocked',exercise=exercise)
    (root/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    raise SystemExit(0 if result['status']=='PASS' else 1)
