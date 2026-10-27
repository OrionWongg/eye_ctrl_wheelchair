"""Task adapter safety and finite heading execution through the real executor."""
import math
from dataclasses import replace
import pytest
from wheelchair_control.executor import Executor
from wheelchair_control.geometry import IndexedObstacles, wrap
from wheelchair_control.planner import Planner, State, integrate, toward
from wheelchair_control.protocol import Packet, STOP, FORWARD, LEFT, RIGHT
from nav_fixtures import config, walls, BOUND


def authorize(core, state, obstacles, command=FORWARD, target=None, task_id='task-1'):
    core.intent(Packet('manager', 1, 0, .25, STOP, False), True)
    core.tick(state, obstacles, BOUND, 0)
    assert core.ready
    packet = Packet('manager', 2, 0, .25, command, True, task_id=task_id,
                    task_kind=1 if command == FORWARD else 2,
                    reference_x=state.x, reference_y=state.y, reference_yaw=state.yaw,
                    target_yaw=state.yaw if target is None else target)
    core.intent(packet)
    return packet


def test_open_scope_cannot_acquire_newly_discovered_corridor():
    core = Planner(config()); state = State(yaw=.05)
    authorize(core, state, IndexedObstacles([]))
    assert core.tick(state, IndexedObstacles([]), BOUND, .1).action == FORWARD
    reference = core.task_scope.reference
    # Walls appearing later do not attach this authorization to a new axis.
    result = core.tick(state, walls(1.3), BOUND, .2)
    assert result.action == FORWARD and result.reason == 'TASK_CONTINUE_FORWARD'
    assert core.task_scope.reference == reference


@pytest.mark.parametrize('field,value', [('target_yaw', .3), ('reference_yaw', .2),
                                         ('reference_x', 1.), ('task_kind', 2), ('command', LEFT)])
def test_same_task_cannot_change_authority(field, value):
    core = Planner(config()); state = State(); obs = IndexedObstacles([])
    packet = authorize(core, state, obs)
    core.tick(state, obs, BOUND, .1)
    core.intent(replace(packet, sequence=3, **{field: value}))
    result = core.tick(state, obs, BOUND, .2)
    assert result.action == STOP and result.reason == 'INVALID_TASK_SCOPE'
    assert core.task_scope is None


def test_continue_bounds_complete_prediction_from_locked_reference():
    core = Planner(config()); obs = IndexedObstacles([])
    authorize(core, State(), obs)
    core.tick(State(), obs, BOUND, .1)
    # Current pose fits, but continuing would leave the authorized lateral band.
    state = State(y=.249, yaw=.05, v=.372, current_action=FORWARD, observation_age=.03)
    result = core.tick(state, obs, BOUND, .2, stopped=False)
    assert result.action == STOP and result.reason == 'TASK_PREDICTED_SCOPE_LIMIT'


@pytest.mark.parametrize('command,angle', [(LEFT, 15), (RIGHT, -15), (LEFT, 3),
                                          (RIGHT, -3), (LEFT, 25), (RIGHT, -25)])
def test_heading_completes_or_explicitly_blocks_at_hard_limit(command, angle):
    c = config(); core = Planner(c); obs = IndexedObstacles([])
    initial = math.radians(179)  # target wraps across pi as well
    state = State(yaw=initial, observation_age=.03, execution_phase=0)
    target = wrap(initial+math.radians(angle))
    packet = authorize(core, state, obs, command, target)
    executor = Executor(c); executor.request(STOP, 0)
    executor.tick(0, 0, 0, True, True); executor.tick(.2, 0, 0, True, True)
    actions = []; completed = False; blocked = False
    for i in range(1, 201):
        now = .2+i*.05
        core.intent(replace(packet, sequence=i+2, stamp=now))
        stopped = executor.phase == 'STOPPED' and abs(state.v)<.015 and abs(state.w)<.025
        d = core.tick(state, obs, BOUND, now, stopped=stopped)
        if core.latch:
            assert abs(angle)==25 and d.reason in (
                'TASK_HEADING_AUTHORITY_UNEXECUTABLE','TASK_HEADING_NO_AUTHORIZED_MICRO_TURN'), (d,state)
            assert stopped and abs(wrap(state.yaw-initial))<=c.task_heading_limit
            assert abs(wrap(target-state.yaw))>c.task_heading_tolerance
            blocked=True
            break
        assert core.task_scope.within_heading_scope(state.yaw)
        if d.action!=STOP:
            assert all(core.task_scope.within_heading_scope(pose[2])
                       for pose in core.last_diagnostics['candidate']['poses'])
        if d.reason == 'TASK_HEADING_COMPLETED':
            assert stopped and abs(wrap(target-state.yaw)) <= c.task_heading_tolerance
            completed = True
            break
        executor.request(d.action, now, core.last_diagnostics['task_pulse_duration'] if d.reason=='TASK_HEADING_MICRO_TURN' else None)
        a, _ = executor.tick(now, state.v, state.w, True, True)
        actions.append(a)
        tv, tw = c.velocity(a)
        v = toward(state.v, tv, (c.linear_brake if a==STOP else c.linear_accel)*.05)
        w = toward(state.w, tw, (c.angular_brake if a==STOP else c.angular_accel)*.05)
        moved = integrate(state, (state.v+v)/2, (state.w+w)/2, .05)
        state = State(moved.x, moved.y, moved.yaw, v, w,
                      d.action if executor.phase=='STARTING' else a, .03,
                      {'STOPPED':0,'RUNNING':1,'STOPPING':2,'STARTING':3,'FAULT':4}[executor.phase])
    assert completed or blocked, (state, target, core.last_diagnostics)
    assert command in actions and set(actions) <= {STOP, command}
    assert state.x == 0 and state.y == 0
    for i in range(5):
        core.intent(replace(packet, sequence=300+i))
        result=core.tick(state, obs, BOUND, now+.1+i*.05)
        assert result.action==STOP
        if completed:assert result.reason=='TASK_HEADING_COMPLETED'
        else:assert core.latch


def test_heading_target_match_while_moving_is_not_completion():
    core = Planner(config()); obs = IndexedObstacles([])
    authorize(core, State(), obs, LEFT, .2)
    state = State(yaw=.2, w=.1, current_action=LEFT, execution_phase=1, observation_age=.03)
    assert core.tick(state, obs, BOUND, .1, stopped=False).reason == 'TASK_HEADING_WAIT_STOPPED'
    assert core.task_scope.phase != 'COMPLETED'


@pytest.mark.parametrize('phase', ['WAIT_STOPPED','TURN','SETTLE','COMPLETED'])
def test_cancel_clears_all_heading_phases(phase):
    core = Planner(config()); obs = IndexedObstacles([])
    authorize(core, State(), obs, LEFT, .2)
    core.tick(State(), obs, BOUND, .1)
    core.task_scope.phase = phase
    core.intent(Packet('manager', 3, .2, .25, STOP, False))
    assert core.task_scope is None
    assert core.tick(State(), obs, BOUND, .2).action == STOP


def test_new_task_relocks_reference_after_explicit_stop():
    core = Planner(config()); obs = IndexedObstacles([])
    authorize(core, State(), obs)
    core.tick(State(), obs, BOUND, .1)
    core.intent(Packet('manager', 3, .2, .25, STOP, False))
    state = State(x=1, yaw=.5)
    packet = Packet('manager', 4, .3, .25, FORWARD, True, task_id='task-2', task_kind=1,
                    reference_x=1, reference_yaw=.5)
    core.intent(packet)
    core.tick(state, obs, BOUND, .3)
    assert core.reference == state.pose


def test_constrained_exit_uses_bounded_turn_and_never_substitutes_direction():
    from wheelchair_sim.scenes import map_obstacles, boundary
    from wheelchair_control.geometry import IndexedPolygon, nearby_obstacles
    state = State(x=10.906735444657267, y=.873704798336599,
                  yaw=-.039533730318275256, observation_age=.03, execution_phase=0)
    for command, sign in ((LEFT, 1), (RIGHT, -1)):
        core = Planner(config())
        obs = nearby_obstacles(state, [IndexedPolygon(b.polygon()) for b in map_obstacles('clearpath_office')], core.cfg)
        bound = boundary('clearpath_office')
        core.intent(Packet('manager', 1, 0, .25, STOP, False), True)
        core.tick(state, obs, bound, 0)
        core.intent(Packet('manager', 2, .1, .25, command, True,
                           task_id='exit', task_kind=2, reference_x=state.x, reference_y=state.y,
                           reference_yaw=state.yaw, target_yaw=state.yaw+sign*math.radians(15)))
        assert core.tick(state, obs, bound, .1).action == STOP
        decision = core.tick(state, obs, bound, .3)
        # Braking-only check: each turn's braking sweep fits, and the
        # requested direction is never substituted.
        assert decision.action == command and decision.reason.startswith('TASK_HEADING_')


def test_packet_ros_conversion_preserves_task_metadata_and_legacy_defaults():
    from types import SimpleNamespace
    from wheelchair_control.nodes import packet as convert
    msg = SimpleNamespace(session_id='manager', sequence=1, ttl_ms=250,
                          header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
                          command=LEFT, enabled=True, valid=True, reason='')
    assert convert(msg).task_id == ''
    msg.task_id='id'; msg.task_kind=2; msg.reference_x=1.; msg.reference_y=2.
    msg.reference_yaw=.3; msg.target_yaw=.5
    converted=convert(msg)
    assert converted.task_id=='id' and converted.task_kind==2
    assert converted.reference_x==1. and converted.reference_y==2.
    assert converted.reference_yaw==.3 and converted.target_yaw==.5


@pytest.mark.parametrize('drift', [-8, 8])
def test_open_continue_restores_locked_heading_before_resuming_forward(drift):
    c=config(); core=Planner(c); obs=IndexedObstacles([])
    state=State(observation_age=.03, execution_phase=0)
    packet=authorize(core, state, obs)
    core.tick(state, obs, BOUND, .1)
    scope=core.task_scope
    state=replace(state, yaw=math.radians(drift))
    executor=Executor(c); executor.request(STOP, 0)
    executor.tick(0, 0, 0, True, True); executor.tick(.2, 0, 0, True, True)
    actions=[]; aligned=False
    for i in range(1, 201):
        now=.2+i*.05
        core.intent(replace(packet, sequence=i+2, stamp=now))
        stopped=executor.phase=='STOPPED' and abs(state.v)<.015 and abs(state.w)<.025
        decision=core.tick(state, obs, BOUND, now, stopped=stopped)
        assert not core.latch, (decision, state)
        assert core.task_scope is scope
        assert core.reference == (0., 0., 0.)
        if decision.reason=='TASK_CONTINUE_HEADING_ALIGNED':
            assert stopped and abs(state.yaw)<=c.task_heading_tolerance
            aligned=True
        if decision.action==FORWARD:
            assert aligned
            break
        assert decision.reason!='TASK_HEADING_COMPLETED'
        if decision.action!=STOP:
            assert all(scope.within_continue_scope(pose)
                       for pose in core.last_diagnostics['candidate']['poses'])
        executor.request(decision.action, now,
                         core.last_diagnostics['task_pulse_duration'] if decision.reason=='TASK_HEADING_MICRO_TURN' else None)
        action, _=executor.tick(now, state.v, state.w, True, True)
        actions.append(action)
        tv, tw=c.velocity(action)
        v=toward(state.v, tv, (c.linear_brake if action==STOP else c.linear_accel)*.05)
        w=toward(state.w, tw, (c.angular_brake if action==STOP else c.angular_accel)*.05)
        moved=integrate(state, (state.v+v)/2, (state.w+w)/2, .05)
        state=State(moved.x, moved.y, moved.yaw, v, w,
                    decision.action if executor.phase=='STARTING' else action, .03,
                    {'STOPPED':0,'RUNNING':1,'STOPPING':2,'STARTING':3,'FAULT':4}[executor.phase])
    else:
        pytest.fail(f'Heading hold did not resume: {state}, {core.last_diagnostics}')
    turn=LEFT if drift<0 else RIGHT
    assert turn in actions and set(actions)<={STOP, turn}


def test_cancel_discards_open_heading_hold_and_its_pending_pulse():
    core=Planner(config()); obs=IndexedObstacles([])
    packet=authorize(core, State(), obs)
    core.tick(State(), obs, BOUND, .1)
    state=State(yaw=.2)
    core.tick(state, obs, BOUND, .2)
    assert core.task_scope.open_correction is not None
    core.intent(replace(packet, sequence=3, command=STOP, enabled=False))
    assert core.task_scope is None
    assert core.tick(state, obs, BOUND, .3).action==STOP


@pytest.mark.parametrize('yaw', [-30., 26.])
def test_heading_actual_pose_outside_authority_stops_immediately(yaw):
    core=Planner(config()); obs=IndexedObstacles([])
    authorize(core, State(), obs, LEFT, math.radians(15))
    result=core.tick(State(yaw=math.radians(yaw)), obs, BOUND, .1)
    assert result.action==STOP and result.reason=='TASK_HEADING_SCOPE_LIMIT'
    assert core.task_scope is None


@pytest.mark.parametrize('target,yaw', [(15.,3.),(25.,11.)])
def test_heading_lease_loss_prediction_cannot_exceed_authorized_angle(target,yaw):
    core=Planner(config()); obs=IndexedObstacles([])
    authorize(core, State(), obs, LEFT, math.radians(target))
    core.tick(State(), obs, BOUND, .1)
    scope=core.task_scope; scope.phase='TURN'; scope.started=0.
    state=State(yaw=math.radians(yaw),w=.32,current_action=LEFT,observation_age=.03,execution_phase=1)
    result=core.tick(state, obs, BOUND, .2, stopped=False)
    assert result.action==STOP and result.reason=='TASK_HEADING_BRAKE'
    assert scope.phase=='SETTLE' and not core.latch
    predicted=core.last_diagnostics['candidate']['poses']
    assert max(math.degrees(p[2]) for p in predicted)>min(25.,target+2.)


def test_heading_at_hard_limit_does_not_restart_unachievable_minimum_pulse():
    core=Planner(config()); obs=IndexedObstacles([])
    authorize(core, State(), obs, LEFT, math.radians(25))
    core.tick(State(), obs, BOUND, .1)
    state=State(yaw=math.radians(22.9),observation_age=.03,execution_phase=0)
    scope=core.task_scope; scope.phase='SETTLE'; scope.brake_at=.1
    assert core.tick(state, obs, BOUND, .4).action==STOP
    result=core.tick(state, obs, BOUND, .6)
    assert result.action==STOP and result.reason=='TASK_HEADING_NO_AUTHORIZED_MICRO_TURN'


@pytest.mark.parametrize('command,direction', [(LEFT,1),(RIGHT,-1)])
def test_pulse_remaining_deducts_only_proven_same_direction_acceleration(command,direction):
    core=Planner(config()); obs=IndexedObstacles([])
    authorize(core, State(), obs, command, direction*.2)
    core.tick(State(), obs, BOUND, .1)
    scope=core.task_scope
    scope.pulse_duration=.3; scope.pulse_start_speed=.01
    # The same odometry sample proves at least 0.1 seconds of acceleration.
    state=State(w=direction*(.01+core.cfg.angular_accel*.1))
    assert scope.pulse_remaining(state,direction)==pytest.approx(.2)
    # Falling speed cannot claim more consumed exposure or tighten the bound.
    assert scope.pulse_remaining(State(w=direction*.03),direction)>.2
    assert scope.pulse_remaining(State(w=-direction*.1),direction)==pytest.approx(.3)
    assert scope.pulse_remaining(State(w=direction),direction)==0.


def test_reduced_pulse_brakes_from_speed_reachable_within_remainder():
    core=Planner(config()); obs=IndexedObstacles([])
    authorize(core, State(), obs, LEFT, .2)
    core.tick(State(), obs, BOUND, .1)
    scope=core.task_scope; scope.pulse_duration=.2; scope.pulse_start_speed=0.
    state=State(w=.08,current_action=LEFT,observation_age=.03,execution_phase=1)
    remaining=scope.pulse_remaining(state,1)
    assert remaining==pytest.approx(.1)
    prediction=core.predictor.check(state,LEFT,obs,BOUND,run=remaining)
    stop=core.predictor.check(state,STOP,obs,BOUND)
    assert prediction.safe and prediction.endpoint.w==pytest.approx(0.,abs=1e-12)
    assert prediction.endpoint.yaw>stop.endpoint.yaw


def test_maximum_safe_validated_pulse_is_frozen_until_settled_or_cancelled():
    core=Planner(config()); obs=IndexedObstacles([])
    packet=authorize(core, State(observation_age=.03), obs, LEFT, math.radians(15))
    core.tick(State(observation_age=.03),obs,BOUND,.1)
    scope=core.task_scope;scope.force_micro=True
    decision=core.tick(State(observation_age=.03),obs,BOUND,.3)
    assert decision.reason=='TASK_HEADING_MICRO_TURN'
    assert scope.pulse_duration==pytest.approx(.3)
    assert core.last_diagnostics['task_pulse_duration']==pytest.approx(.3)
    assert all(scope.within_heading_scope(pose[2]) for pose in core.last_diagnostics['candidate']['poses'])
    core.intent(replace(packet,sequence=3))
    assert core.task_scope is scope and scope.pulse_duration==pytest.approx(.3)
    core.intent(replace(packet,sequence=4,command=STOP,enabled=False))
    assert core.task_scope is None
    assert core.tick(State(),obs,BOUND,.4).action==STOP


@pytest.mark.parametrize('actual_accel',[0., .2, .4, .8])
@pytest.mark.parametrize('elapsed',[.05, .1, .2])
def test_pulse_remainder_never_underestimates_for_acceleration_within_model(actual_accel,elapsed):
    core=Planner(config()); obs=IndexedObstacles([])
    authorize(core, State(), obs, LEFT, .2)
    core.tick(State(), obs, BOUND, .1)
    scope=core.task_scope;scope.pulse_duration=.3;scope.pulse_start_speed=.01
    measured=State(w=.01+actual_accel*elapsed)
    assert scope.pulse_remaining(measured,1)>=.3-elapsed-1e-12
