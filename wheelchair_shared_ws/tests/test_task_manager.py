import math
from dataclasses import replace
import pytest
from wheelchair_control.task_manager import *

def feed(core,t,pose=(0.,0.,0.),v=0.,w=0.,ready=True,task_id=None):
    core.on_health(Health('source',int((t+100)*10000)+1,t,.25,True,0.,int((t+100)*10000)+1),t,t)
    core.on_lower(dict(session_id='planner',intent_session_id=core.session,task_id=task_id or '',
        state=0,requires_rearm=not ready,ready_to_rearm=ready,reason='USER_STOP'),t)
    core.on_execution(dict(session_id='executor',state=0 if not(v or w) else 1,
        stopped=not(v or w),executed_action=0 if not(v or w) else 1),t)
    core.on_pose(pose,v,w,t)

def event(core,t=0.,event_id=1,kind=CONTINUE,angle=0.):
    c=Candidate('source',event_id,t,1.5,'candidate'+str(event_id),event_id,kind,angle,True,True,core.session,core.context_version)
    assert core.on_candidate(c,t,t)
    return Event('source',event_id,t,1.5,kind,c.candidate_id,c.version,core.session,core.context_version,angle)

def active(kind=CONTINUE,angle=0.):
    c=TaskManager(session='manager');feed(c,-1.)
    e=event(c,t=-1.,kind=kind,angle=angle);assert c.on_event(e,-1.,-1.) is None
    assert not c.tick(-1.,-1.).enabled
    feed(c,0.)
    o=c.tick(0,0);assert o.enabled and c.acks[-1].accepted
    return c,e,o

def test_health_and_candidates_alone_never_start():
    c=TaskManager(session='manager')
    for i in range(20):
        t=i*.1;feed(c,t);event(c,t,i+1,ADJUST_HEADING,.26)
        assert not c.tick(t,t).enabled and c.active is None

def test_single_event_runs_with_scanning_candidates_and_duplicate_does_not_reset_budget():
    c,e,o=active();task=c.active
    for i in range(1,21):
        t=i*.05;feed(c,t,(t*.2,0.,0.),v=.2,task_id=o.task_id)
        event(c,t,i+10,ADJUST_HEADING,-.26)
        c.on_event(e,t,t)  # transport retries preserve identity and original stamp
        result=c.tick(t,t)
        assert result.task_id==o.task_id and result.command==FORWARD
        assert c.active is task and c.active.started==0.
    assert c.active.distance==pytest.approx(.2)

def test_duplicate_after_completion_cannot_restart_and_id_mutation_rejected():
    c,e,o=active(ADJUST_HEADING,.26)
    feed(c,.1,task_id=o.task_id);c.lower['reason']='TASK_HEADING_COMPLETED'
    assert not c.tick(.1,.1).enabled and c.state==COMPLETED
    ack=c.on_event(e,.2,.2)
    assert ack.accepted and not c.tick(.2,.2).enabled
    assert not c.on_event(replace(e,relative_yaw=-.26),.2,.2).accepted

def test_replacement_stops_then_locks_adjust_target_at_actual_stopped_pose():
    c,e,o=active();feed(c,.1,(.01,0.,.1),v=.2,task_id=o.task_id)
    e2=event(c,.1,2,ADJUST_HEADING,.26)
    assert c.on_event(e2,.1,.1) is None
    assert not c.tick(.1,.1).enabled and c.state==STOPPING and c.active is None
    feed(c,.6,(.1,0.,.2))
    out=c.tick(.6,.6)
    assert out.enabled and out.command==LEFT
    assert out.reference==(.1,0.,.2) and out.target_yaw==pytest.approx(.46)
    assert c.active.started==.6

def test_waiting_stop_event_expiry_cannot_be_extended_by_retries_or_health():
    c,e,o=active();feed(c,.1,v=.2,task_id=o.task_id)
    e2=event(c,.1,2,ADJUST_HEADING,.26);c.on_event(e2,.1,.1)
    for t in (.5,1.,1.5,1.61):
        feed(c,t,v=.2);c.on_event(e2,t,t)
        assert not c.tick(t,t).enabled
    assert c.pending is None and not c.acks[-1].accepted
    feed(c,1.7);assert not c.tick(1.7,1.7).enabled

@pytest.mark.parametrize('reason',['withdraw','version','context'])
def test_pending_candidate_revalidated_after_stop(reason):
    c=TaskManager(session='manager');feed(c,0,v=.2)
    e=event(c);c.on_event(e,0,0)
    if reason=='withdraw':c.candidate=replace(c.candidate,confirmable=False)
    elif reason=='version':c.candidate=replace(c.candidate,version=99)
    else:c.context_version+=1
    feed(c,.1);assert not c.tick(.1,.1).enabled
    assert c.pending is None and not c.acks[-1].accepted

def test_cancel_wins_and_invalidates_pre_cancel_confirmation():
    c,e,o=active();later=event(c,.1,2,ADJUST_HEADING,.26)
    cancel=Event('source',3,.1,1.5,CANCEL)
    assert c.on_event(cancel,.1,.1).accepted
    assert not c.on_event(later,.1,.1).accepted
    feed(c,.2);assert not c.tick(.2,.2).enabled
    assert c.active is None and c.pending is None

@pytest.mark.parametrize('failure',['invalid','timeout','planner','executor','clock'])
def test_failure_clears_authorization_and_recovery_does_not_resume(failure):
    c,e,o=active()
    if failure=='invalid':c.on_health(Health('source',2,.1,.25,False,0.,2),.1,.1)
    elif failure=='timeout':c.tick(.3,.3)
    elif failure=='planner':c.on_lower(dict(session_id='new',state=0),.1)
    elif failure=='executor':c.on_execution(dict(session_id='new',state=0),.1)
    else:c.set_clock_fault('CLOCK_JUMP',.1)
    assert c.active is None
    c.set_clock_fault('',.4);feed(c,.4)
    assert not c.tick(.4,.4).enabled
    c.on_event(e,.4,.4);assert not c.tick(.4,.4).enabled

def test_new_source_session_cancels_and_retired_session_cannot_resume():
    c,e,o=active()
    c.on_health(Health('new',1,.1,.25,True,0.,1),.1,.1)
    assert c.active is None and 'source' in c.retired
    assert not c.on_event(replace(e,event_id=99),.1,.1).accepted

def test_repeated_same_sample_does_not_make_old_source_fresh():
    c=TaskManager(session='manager');feed(c,0)
    for i in range(1,7):
        t=i*.05
        c.on_health(Health('source',1000002+i,t,.25,True,0.,1000001),t,t)
    assert not c.healthy(.30)

def test_source_age_and_network_age_are_added():
    c=TaskManager(session='manager')
    assert not c.on_health(Health('source',1,0.,.25,True,.2,1),.1,.1)

def test_time_budget_heartbeats_do_not_reset_and_finishes_stopped():
    c,e,o=active()
    for i in range(1,11):
        feed(c,float(i),task_id=o.task_id)
        output=c.tick(float(i),float(i))
    assert not output.enabled and c.reason=='TASK_TIME_LIMIT' and c.state==COMPLETED

def test_accumulated_path_counts_turns_and_return_trip_not_displacement():
    c,e,o=active();x=0.
    for i in range(1,100):
        t=i*.05;x+=.04 if i<40 else -.04
        feed(c,t,(x,0.,0.),v=.372,task_id=o.task_id)
        if not c.tick(t,t).enabled:break
    assert c.reason=='TASK_DISTANCE_LIMIT' and c.state==STOPPING
    assert abs(x)<1. and c.last_task.distance>2.6
    # Actual braking motion still contributes to the recorded task path.
    before=c.last_task.distance;feed(c,t+.05,(x-.01,0.,0.))
    c.tick(t+.05,t+.05)
    assert c.last_task.distance==pytest.approx(before+.01) and c.state==COMPLETED

def test_old_task_completion_feedback_cannot_complete_new_task():
    c,e,o=active();feed(c,.1)
    e2=event(c,.1,2,ADJUST_HEADING,.26);c.on_event(e2,.1,.1);assert not c.tick(.1,.1).enabled
    feed(c,.6);new=c.tick(.6,.6)
    feed(c,.7,task_id=o.task_id);c.lower['reason']='TASK_HEADING_COMPLETED'
    assert c.tick(.7,.7).task_id==new.task_id and c.active is not None

@pytest.mark.parametrize('angle',[float('nan'),float('inf'),0.,math.pi/2])
def test_bad_adjust_angles_never_authorize(angle):
    c=TaskManager(session='manager');feed(c,0)
    e=event(c,kind=ADJUST_HEADING,angle=angle)
    assert not c.on_event(e,0,0).accepted and not c.tick(0,0).enabled

def test_normal_map_or_health_renewals_do_not_churn_context():
    c=TaskManager(session='manager');feed(c,0);context=c.context_version
    for i in range(1,10):feed(c,i*.05);c.tick(i*.05,i*.05)
    assert c.context_version==context

def test_replacement_cannot_use_pre_stop_stationary_feedback():
    c,e,o=active(ADJUST_HEADING,.26)
    feed(c,.1,task_id=o.task_id)  # old task still STARTING, odom says zero
    new=event(c,.1,2,ADJUST_HEADING,.26)
    c.on_event(new,.1,.1)
    assert not c.tick(.1,.1).enabled
    assert c.pending is not None and c.active is None
    # Fresh health alone is not acknowledgment that queued wheel motion stopped.
    c.on_health(Health('source',1005002,.5,.25,True,0.,1005002),.5,.5)
    assert not c.tick(.5,.5).enabled
    assert c.active is None

def test_distance_budget_reserves_stop_before_zero_velocity_restart():
    c,e,o=active()
    c.active.distance=2.8
    feed(c,.1,task_id=o.task_id)
    assert not c.tick(.1,.1).enabled
    assert c.reason=='TASK_DISTANCE_LIMIT'

def test_source_time_lower_feedback_cannot_get_extra_receipt_lifetime():
    c,e,o=active()
    feed(c,.1,task_id=o.task_id)
    # Wrapper passes monotonic source-equivalent time, not receipt time.
    c.lower_at=.1-.34;c.execution_at=.1-.34
    c.on_health(Health('source',1001202,.12,.25,True,0.,1001202),.12,.12)
    assert not c.tick(.12,.12).enabled
    assert c.reason=='LOWER_HEALTH_TIMEOUT_OR_FAULT'

def test_unreached_heading_at_deadline_is_blocked_not_completed():
    c,e,o=active(ADJUST_HEADING,.26)
    feed(c,9.,task_id=o.task_id)
    assert not c.tick(9.,9.).enabled
    assert c.state==BLOCKED and c.reason=='TASK_HEADING_TIME_LIMIT'

def test_task_identity_includes_source_session_not_only_event_number():
    c,e,o=active()
    c.on_health(Health('new-source',1,.1,.25,True,0.,1),.1,.1)
    candidate=Candidate('new-source',1,.1,1.5,'new-candidate',1,CONTINUE,0.,True,True,c.session,c.context_version)
    c.on_candidate(candidate,.1,.1)
    event2=Event('new-source',e.event_id,.1,1.5,CONTINUE,'new-candidate',1,c.session,c.context_version)
    c.on_event(event2,.1,.1)
    assert c.pending['task_id']!=o.task_id

def test_clock_fault_does_not_turn_every_health_heartbeat_into_a_new_context():
    c=TaskManager(session='manager');c.set_clock_fault('CLOCK_UNAVAILABLE',0.)
    context=c.context_version
    for i in range(1,5):
        t=i*.05
        c.on_health(Health('source',i,0.,.25,True,0.,i),t,0.)
        c.set_clock_fault('CLOCK_UNAVAILABLE',t);assert not c.tick(t,0.).enabled
    assert c.context_version==context

def test_last_forward_grant_reserves_odom_age_and_both_lease_expiries():
    c,e,o=active()
    speed=c.cfg.forward_speed
    tail=speed*(.25+.15+.05+.05+c.cfg.odom_timeout)+speed**2/(2*c.cfg.linear_brake)
    for i in range(1,76):
        t=i*.05;distance=i*.04
        feed(c,t,(distance,0.,0.),v=speed,task_id=o.task_id)
        output=c.tick(t,t)
        if not output.enabled:break
        assert c.active.distance+tail<=c.cfg.max_distance
    assert not output.enabled and c.reason=='TASK_DISTANCE_LIMIT'
