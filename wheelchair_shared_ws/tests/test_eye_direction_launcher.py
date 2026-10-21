import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
import pytest


@pytest.mark.parametrize('arguments',[[],['--eyes','left'],['--center-file=/tmp/center.json'],
    ['--gaze-method=legacy','--calibration-file','/tmp/old.json'],
    ['--camera-focal-px','477','--gaze-span-deg','30','--recalibrate']])
def test_combined_launcher_selects_directional_stamped_eye_and_overrides_legacy_domain(monkeypatch,arguments):
    scripts=Path(__file__).parents[1]/'scripts'
    monkeypatch.syspath_prepend(str(scripts))
    spec=importlib.util.spec_from_file_location('eye_combined_launcher',scripts/'run_eye_sim.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    for key in ('WHEELCHAIR_INTERACTION_MODE','WHEELCHAIR_MODE','WHEELCHAIR_ROS_DOMAIN_ID'):
        monkeypatch.delenv(key,raising=False)
    monkeypatch.setenv('EYE_ROS_DOMAIN_ID','58');monkeypatch.setenv('ROS_DOMAIN_ID','58')
    monkeypatch.setattr(sys,'argv',['run_eye_sim.py','--ritnet',*arguments])
    calls=[]
    def launch(command,**kwargs):
        calls.append((command,kwargs));return SimpleNamespace(poll=lambda:0)
    monkeypatch.setattr(module.subprocess,'Popen',launch)
    monkeypatch.setattr(module,'stop_all',lambda children:None)
    monkeypatch.setattr(module.signal,'signal',lambda *args:None)
    assert module.main()==0
    sim,tracker=calls
    assert 'interaction_mode:=directional_intent' in sim[0]
    assert 'input_source:=eye' in sim[0] and 'eye_sample_mode:=stamped' in sim[0]
    assert '--publish-stamped' in tracker[0]
    assert tracker[0][1:]==['--publish-stamped',*arguments]
    for _,options in calls:
        assert all(options['env'][key]=='64' for key in ('ROS_DOMAIN_ID','EYE_ROS_DOMAIN_ID','WHEELCHAIR_ROS_DOMAIN_ID'))



def test_default_launcher_uses_fusion(monkeypatch):
    scripts=Path(__file__).parents[1]/'scripts'
    monkeypatch.syspath_prepend(str(scripts))
    spec=importlib.util.spec_from_file_location('fusion_launcher',scripts/'run_eye_sim.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    for key in ('WHEELCHAIR_INTERACTION_MODE','WHEELCHAIR_MODE'):
        monkeypatch.delenv(key,raising=False)
    monkeypatch.setattr(sys,'argv',['run_eye_sim.py','--eyes','both'])
    calls=[]
    monkeypatch.setattr(module.subprocess,'Popen',lambda command,**kw: calls.append((command,kw)) or SimpleNamespace(poll=lambda:0))
    monkeypatch.setattr(module,'stop_all',lambda children:None)
    monkeypatch.setattr(module.signal,'signal',lambda *args:None)
    assert module.main()==0
    assert 'eye_sample_mode:=fusion' in calls[0][0]
    assert calls[1][0][0].endswith('/start_fusion.sh')
    assert calls[1][0][1:]==['--publish-ros','--eyes','both']
