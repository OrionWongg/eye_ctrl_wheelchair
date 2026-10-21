#!/usr/bin/env python3
"""Willow Office smoke acceptance through the normal direction input interface."""
import json
import math
from pathlib import Path
from validate_directional import run_case


def exercise(h):
    d=h.data
    # Large static map rasterization can finish after manager can_start first
    # becomes true. Keep publishing health while the costmap initializes.
    h.spin(6.)
    h.event(1)
    assert h.spin(10,lambda:d['pose'][0]>.25),'W did not start: '+d['status'].reason
    assert h.spin(45,lambda:d['pose'][0]>6.),'main aisle cruise failed: '+d['status'].reason
    h.event(5)
    assert h.spin(4,h.stopped),'STOP failed'
    parked=d['pose'];h.spin(.7)
    assert math.dist(parked[:2],d['pose'][:2])<.03,'movement after STOP'
    h.event(3)
    assert h.spin(20,lambda:d['status'].reason=='NAV_TURN_COMPLETED' and h.stopped()),'parked right failed: '+d['status'].reason
    h.event(1)
    assert h.spin(8,lambda:math.dist(parked[:2],d['pose'][:2])>.3),'resume failed: '+d['status'].reason
    h.event(5)
    assert h.spin(4,h.stopped),'final STOP failed'
    assert not d.get('runtime_faults'),str(d.get('runtime_faults'))


if __name__=='__main__':
    root=Path('artifacts/willow-office-20261016')
    result=run_case('cruise_turn_resume',root,scene_override='willow_office',exercise=exercise)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['status']=='PASS' else 1)
