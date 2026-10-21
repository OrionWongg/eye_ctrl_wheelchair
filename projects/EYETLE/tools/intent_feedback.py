"""Pure parsing/ASCII display of controller feedback; no gaze classification."""
import json
import math

TASK_STATES = {0: 'IDLE', 1: 'ACCEPTED', 2: 'RUNNING', 3: 'STOPPING',
               4: 'COMPLETED', 5: 'BLOCKED', 6: 'FAULT'}
ACTIONS = {0: 'STOP', 1: 'FORWARD', 2: 'LEFT', 3: 'RIGHT'}


def parse_feedback(payload, received_at):
    data = json.loads(payload) if isinstance(payload, str) else payload
    if not isinstance(data, dict):
        raise ValueError('feedback must be a JSON object')
    mode = data.get('interaction_mode', 'gaze_joystick')
    result = dict(stamp=received_at, interaction_mode=mode)
    if mode not in ('intent_confirm','directional_intent'):
        progress = float(data.get('resume_progress', 0.))
        if not math.isfinite(progress):
            raise ValueError('nonfinite resume progress')
        result.update(state=str(data.get('state', 'UNKNOWN')),
                      resume_phase=str(data.get('resume_phase', '')),
                      resume_progress=min(1., max(0., progress)))
        return result
    task = data.get('task') or {}
    if not isinstance(task, dict):
        raise ValueError('task feedback must be an object')
    preview = int(data.get('preview', 0))
    if preview not in (-1, 0, 1):
        raise ValueError('invalid preview direction')
    angle = float(data.get('preview_relative_yaw', -preview * math.radians(15)))
    candidate_angle = float(data.get('candidate_relative_yaw', angle))
    if not math.isfinite(angle) or not math.isfinite(candidate_angle):
        raise ValueError('nonfinite candidate angle')
    result.update(state=str(task.get('state','WAITING')) if mode=='directional_intent' else TASK_STATES.get(task.get('state'), 'WAITING'),
                  resume_phase='', resume_progress=0., preview=preview,
                  preview_degrees=abs(math.degrees(angle)), stable=bool(data.get('stable', False)),
                  candidate_kind=int(data.get('candidate_kind', 1 if preview == 0 else 2)),
                  candidate_relative_yaw=candidate_angle,
                  consumed=bool(data.get('consumed', False)), ack=str(data.get('ack', 'NONE')),
                  healthy=bool(data.get('healthy', False)),
                  action=ACTIONS.get(task.get('executed_action'), 'UNKNOWN'),
                  reason=str(data.get('reason', '')), task_reason=str(task.get('reason', '')),
                  task_id=str(task.get('task_id', '')))
    return result


def display_lines(feedback, fresh=True):
    """Three ASCII lines fit the OpenCV overlay without a Unicode font."""
    if not fresh:
        return ('Controller feedback unavailable', 'Task: UNKNOWN | action: UNKNOWN',
                'Wait for fresh controller feedback')
    direction = {-1: 'LEFT', 0: 'CONTINUE', 1: 'RIGHT'}[feedback['preview']]
    kind = feedback['candidate_kind']
    angle = feedback['candidate_relative_yaw']
    candidate = ('CONTINUE (limited task)' if kind == 1 else
                 ('LEFT' if angle > 0 else 'RIGHT') + f' {abs(math.degrees(angle)):.1f} deg'
                 if kind == 2 else 'NONE')
    if feedback['interaction_mode']=='directional_intent':
        candidate=('START / CONTINUE (W)' if kind==1 else
                   'LEFT INTENT (A)' if kind==2 and angle>0 else 'RIGHT INTENT (D)' if kind==2 else 'NONE')
    stable = 'YES' if feedback['stable'] else 'NO'
    if feedback['consumed']:
        stable = 'CONSUMED'
    reason = ' / '.join(dict.fromkeys(x for x in (feedback['reason'], feedback['task_reason']) if x))
    # Protocol reasons currently use ASCII codes; sanitize future user text too.
    reason = reason.encode('ascii', 'replace').decode('ascii')
    health = '' if feedback['healthy'] else ' | INPUT INVALID'
    return (f'Preview: {direction} | candidate: {candidate} | stable: {stable}',
            f"Task: {feedback['state']} | action: {feedback['action']} | ACK: {feedback['ack']}",
            f'Reason: {reason}{health}')
