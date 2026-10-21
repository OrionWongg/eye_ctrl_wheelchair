import json
import pytest
from whill_runtime.drive import EyeDrive
from whill_runtime.eye_core import Blinks, Calibration


def sample(sequence, now, event=None, **kwargs):
    data=dict(schema=1,session='test',sequence=sequence,stamp_ns=int(now*1e9),source_t=now,
              horizontal=0.,fresh=True,calibrated=True,open=True,motion_valid=True,
              gesture_continuity=True,long_close=False,blink_event=event)
    data.update(kwargs)
    return json.dumps(data)


def event(ident, now):
    return dict(id=ident,start=now-.15,end=now,durations=[.15,.15])


def test_double_blink_and_timeout_require_new_authorization():
    gate=EyeDrive()
    gate.receive(sample(1,10,event(1,10)),10,int(10e9))
    assert gate.velocity(10)==(0.,0.)
    gate.receive(sample(2,10.5,event(2,10.5)),10.5,int(10.5e9))
    assert gate.velocity(10.5)==(.25,0.)
    assert gate.velocity(10.8)==(0.,0.)
    gate.receive(sample(3,10.9),10.9,int(10.9e9))
    assert gate.velocity(10.9)==(0.,0.)


def test_invalid_eyes_pause_and_single_blink_resumes():
    gate=EyeDrive()
    for i,t in enumerate([10.,10.5],1):
        gate.receive(sample(i,t,event(i,t)),t,int(t*1e9))
    gate.receive(sample(3,10.6,open=False,motion_valid=False),10.6,int(10.6e9))
    assert gate.velocity(10.6)==(0.,0.)
    gate.receive(sample(4,10.75,event(3,10.75)),10.75,int(10.75e9))
    assert gate.velocity(10.75)==(.25,0.)


def test_duplicate_and_retired_session_do_not_rearm():
    gate=EyeDrive()
    gate.receive(sample(1,10,event(1,10)),10,int(10e9))
    gate.receive(sample(2,10.1,event(1,10)),10.1,int(10.1e9))
    assert not gate.active
    gate.receive(sample(1,10.2,session='new'),10.2,int(10.2e9))
    gate.receive(sample(3,10.3,event(2,10.3)),10.3,int(10.3e9))
    assert gate.session=='new' and not gate.active


def test_stale_input_cannot_arm():
    gate=EyeDrive()
    gate.receive(sample(1,10,event(1,10)),10.5,int(10.5e9))
    assert gate.received < 0 and not gate.active


def test_binocular_blink_requires_overlap_and_explicit_closure():
    b=Blinks()
    assert b.update(['open','open'],10)[0] is None
    assert b.update(['closed','open'],10.05)[0] is None
    assert b.update(['closed','closed'],10.1)[0] is None
    assert b.update(['open','closed'],10.2)[0] is None
    e,_=b.update(['open','open'],10.25)
    assert e and e['durations']==pytest.approx([.15,.15])
    for t in [10.3,10.35,10.4]:
        assert b.update(['unknown','unknown'],t)[0] is None


def test_camera_gap_cannot_complete_blink():
    b=Blinks()
    b.update(['open','open'],10)
    b.update(['closed','closed'],10.1)
    assert b.update(['open','open'],10.5)[0] is None


def test_calibration_conflict_and_atomic_roundtrip(tmp_path):
    import numpy as np
    c=Calibration(tmp_path/'cal.json',{'camera':'test'})
    c.points=np.array([[[.3,.3],[.7,.7]],[[.5,.5],[.5,.5]],[[.7,.7],[.3,.3]]])
    c.selected[:]=True
    c.ready=True
    detection=[dict(features=[.7,.7],quality=[1.,1.]),dict(features=[.3,.3],quality=[1.,1.])]
    assert c.direction(detection)==pytest.approx((1.,True))
    detection[1]['features']=[.7,.7]
    assert c.direction(detection)==(0.,False)
    c.save()
    restored=Calibration(c.path,c.binding)
    assert restored.loaded and not restored.ready and restored.step==3
    changed=Calibration(c.path,{'camera':'other'})
    assert not changed.loaded
