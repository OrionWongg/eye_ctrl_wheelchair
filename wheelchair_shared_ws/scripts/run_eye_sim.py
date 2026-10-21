#!/usr/bin/python3
"""Launch Gazebo shared control and EYETLE RITnet in one owned process tree."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from process_cleanup import stop_group

ROOT = Path(__file__).resolve().parents[1]
EYETLE = Path(os.environ.get('EYETLE_WORKSPACE', str(ROOT.parent/'projects/EYETLE')))

def stop_all(children):
    # Close the tracker and simulator together. Never signal unrelated processes.
    for child in children:
        if child.poll() is None:
            # ros2 launch forwards SIGINT itself; signaling its whole group here
            # would interrupt node cleanup twice. Group signals are escalation only.
            child.send_signal(signal.SIGINT)
    for child in children:stop_group(child,initial_signal=False)

def main():
    args=sys.argv[1:]
    fusion = args[:1] != ['--ritnet']
    if args[:1] in (['--ritnet'], ['--fusion']):args=args[1:]
    detector = 'start_fusion.sh' if fusion else 'start_ritnet.sh'
    if '--help' in args or '-h' in args:
        print('Usage: run_eye_sim.py [--fusion|--ritnet] [detector options]\n'
              'Starts Gazebo office + eye shared control + RITnet. Ctrl+C closes both.\n'
              'Defaults to calibrated RITnet + PuReST fusion; --ritnet selects the old interface.\n'
              'Detector options are passed through unchanged; see EYETLE help below.\n'
              'Environment: WHEELCHAIR_ROS_DOMAIN_ID (default 64), WHEELCHAIR_SCENE,\n'
              'WHEELCHAIR_MODE (stop_only/bounded_assist/corridor_assist), WHEELCHAIR_GUI (true/false),\n'
              'WHEELCHAIR_FIRST_PERSON (true default), WHEELCHAIR_FIRST_PERSON_VIEW (true default),\n'
              'WHEELCHAIR_INTERACTION_MODE (directional_intent default, intent_confirm or gaze_joystick), EYETLE_WORKSPACE.\n\nRITnet options:',flush=True)
        return subprocess.call([str(EYETLE/detector),'--help'])
    # EYETLE owns camera selection, estimation method and reference files.
    # Do not inject or reinterpret detector configuration here.
    env=os.environ.copy()
    interaction=env.get('WHEELCHAIR_INTERACTION_MODE','directional_intent')
    if interaction not in ('gaze_joystick','intent_confirm','directional_intent'):
        print('WHEELCHAIR_INTERACTION_MODE must be directional_intent, intent_confirm or gaze_joystick.',file=sys.stderr);return 2
    if fusion and interaction != 'directional_intent':
        print('Fusion requires directional_intent.', file=sys.stderr);return 2
    mode=env.get('WHEELCHAIR_MODE','corridor_assist' if interaction in ('intent_confirm','directional_intent') else 'stop_only')
    if mode not in ('stop_only','bounded_assist','corridor_assist'):
        print('WHEELCHAIR_MODE must be stop_only, bounded_assist or corridor_assist.',file=sys.stderr)
        return 2
    if interaction in ('intent_confirm','directional_intent') and mode!='corridor_assist':
        print('intent_confirm currently requires WHEELCHAIR_MODE=corridor_assist; use gaze_joystick for legacy comparisons.',file=sys.stderr)
        return 2
    domain=env.get('WHEELCHAIR_ROS_DOMAIN_ID','64')
    env.update(WHEELCHAIR_ROS_DOMAIN_ID=domain,EYE_ROS_DOMAIN_ID=domain,
               ROS_DOMAIN_ID=domain,ROS_LOCALHOST_ONLY='1',EYE_ROS_LOCALHOST_ONLY='1')
    env.setdefault('IGN_PARTITION','wheelchair_shared_demo')
    if interaction in ('intent_confirm','directional_intent'):
        env['EYE_WHEELCHAIR_SETUP']=str(ROOT/'install/setup.bash')
        flag = '--publish-ros' if fusion else '--publish-stamped'
        if flag not in args:args=[flag,*args]
    interaction_help=('Double blink starts; gaze left/right steers directly; a single blink resumes after pause.\n'
                      if fusion else 'Gaze selects W/A/D; double blink confirms. Long close or tracking loss stops.\n'
                      if interaction=='directional_intent' else 'Gaze selects only; double blink confirms one finite task. Long close cancels. Single blink never resumes.\n'
                      if interaction in ('intent_confirm','directional_intent') else
                      'Double blink once to enable; long close pauses. Resume: open eyes, then blink once (60-400ms).\n')
    print(f'Gazebo wheelchair + {detector}: ROS domain {domain}, input_source=eye, mode={mode}, interaction={interaction}.\n'+interaction_help+
          'Start keyboard.sh in another terminal for Space emergency stop.\n'
          'Close existing wheelchair simulation before starting this command.',flush=True)
    children=[];interrupted=False
    def on_signal(signum,frame):
        nonlocal interrupted
        interrupted=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,on_signal)
    try:
        children.append(subprocess.Popen([str(ROOT/'scripts/run_sim.sh'),'input_source:=eye',
            'mode:='+mode,
            'interaction_mode:='+interaction,
            'eye_sample_mode:='+('fusion' if fusion else 'stamped' if interaction in ('intent_confirm','directional_intent') else 'legacy'),
            *(['config:='+str(ROOT/'src/wheelchair_control/config/narrow_control.yaml')] if mode=='corridor_assist' else []),
            'scene:='+env.get('WHEELCHAIR_SCENE','clearpath_office'),
            'first_person:='+env.get('WHEELCHAIR_FIRST_PERSON','true'),
            'first_person_view:='+env.get('WHEELCHAIR_FIRST_PERSON_VIEW','true'),
            'gui:='+env.get('WHEELCHAIR_GUI','true')],env=env,start_new_session=True))
        children.append(subprocess.Popen([str(EYETLE/detector),*args],env=env,start_new_session=True))
        while not interrupted:
            for name,child in zip(('Gazebo','RITnet'),children):
                code=child.poll()
                if code is not None:
                    print(f'{name} exited ({code}); stopping the eye simulation.',flush=True)
                    return code if code>=0 else 128-code
            time.sleep(.1)
        return 130
    finally:stop_all(children)

if __name__=='__main__':sys.exit(main())
