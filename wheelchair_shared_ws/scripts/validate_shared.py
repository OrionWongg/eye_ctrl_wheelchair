#!/usr/bin/python3
"""Real Gazebo E2E, isolated domain, no real wheelchair connection."""
import argparse,json,math,os,signal,subprocess,time,uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray,String
from nav_msgs.msg import Odometry
from wheelchair_interfaces.msg import ControlIntent,ControlStatus,DiscreteAction,ExecutionStatus
from ros_gz_interfaces.msg import Contacts

parser=argparse.ArgumentParser();parser.add_argument('--scenario',choices=['b1','eye','assist','sensor','pause','replay','executor_exit','rotation_wall','narrow','both_blocked','dynamic','office','corridor','passage','door','micro_turn','smooth','side_choice','exit_recovery'],default='b1');parser.add_argument('--test-scene',default='');parser.add_argument('--heading',type=float,default=5.);parser.add_argument('--lateral',type=float,default=0.);parser.add_argument('--initial-x',type=float,default=0.);parser.add_argument('--expect-blocked',action='store_true');parser.add_argument('--expect-exit',action='store_true');args=parser.parse_args()
assert os.environ.get('ROS_DOMAIN_ID')=='97' and os.environ.get('IGN_PARTITION')=='wheelchair_validation', 'Use isolated ROS_DOMAIN_ID=97 IGN_PARTITION=wheelchair_validation'
root=Path(__file__).resolve().parents[1];log=open(root/'artifacts'/('gazebo-'+args.scenario+'.log'),'w')
launch_args=['ros2','launch','wheelchair_control','shared.launch.py','gui:=false']
if args.scenario not in ('assist','rotation_wall','narrow','both_blocked','dynamic','office','corridor','passage','door','smooth','side_choice','exit_recovery'):launch_args+=['scene:=straight_wall']
if args.scenario=='assist':launch_args+=['mode:=bounded_assist','scene:=slight_offset']
if args.scenario=='eye':launch_args+=['input_source:=eye']
if args.scenario=='office':launch_args+=['scene:=clearpath_office']
if args.scenario=='corridor':launch_args+=['scene:=clearpath_office','mode:=bounded_assist']
if args.scenario=='exit_recovery':launch_args+=['scene:=clearpath_office','mode:=corridor_assist']
if args.scenario=='smooth':launch_args+=['scene:=clearpath_office','mode:=corridor_assist']
if args.scenario=='side_choice':launch_args+=['scene:=side_door','mode:=corridor_assist']
if args.scenario in ('passage','door'):
    launch_args+=['scene:='+(args.test_scene or ('passage_090' if args.scenario=='passage' else 'door_090')),'mode:=corridor_assist']
if args.scenario in ('rotation_wall','narrow','both_blocked','dynamic'):launch_args+=['scene:='+args.scenario]
process=subprocess.Popen(launch_args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('shared_control_validation',parameter_overrides=[Parameter('use_sim_time',value=True)])
pub=node.create_publisher(ControlIntent,'/wheelchair/keyboard_intent',1)
eye_pub=node.create_publisher(Float32MultiArray,'/eye_tracking/signals',1)
eye_data=None
drive_target=None
base_pub=node.create_publisher(Twist,'/wheelchair/sim/drive',1)
state={'status':None,'odom':None,'execution':None,'contacts':0};samples=[];drive=[];statuses=[]
node.create_subscription(ControlStatus,'/wheelchair/control_status',lambda m:(state.update(status=m),statuses.append(m)),10)
node.create_subscription(Odometry,'/wheelchair/odom',lambda m:state.update(odom=m),qos_profile_sensor_data)
node.create_subscription(ExecutionStatus,'/wheelchair/executor_status',lambda m:state.update(execution=m),10)
node.create_subscription(DiscreteAction,'/wheelchair/action_safe',lambda m:samples.append(m),10)
node.create_subscription(Twist,'/wheelchair/sim/drive',lambda m:drive.append((m.linear.x,m.angular.z)),10)
def contacts(m):
    state['contacts']+=sum('wheelchair' in c.collision1.name or 'wheelchair' in c.collision2.name for c in m.contacts)
node.create_subscription(Contacts,'/wheelchair/sim/contacts',contacts,10)
session=str(uuid.uuid4());sequence=0;mode=0;enabled=False;publish=True;frozen=None

def spin(seconds):
    global sequence
    deadline=time.monotonic()+seconds;last=0.;last_eye=0.;last_drive=0.
    while time.monotonic()<deadline:
        if process.poll() is not None:raise RuntimeError('Launch exited')
        if publish and time.monotonic()-last>=.05:
            msg=ControlIntent();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='base_link'
            sequence+=1;msg.session_id=session;msg.sequence=sequence;msg.command=mode;msg.enabled=enabled;msg.valid=True;msg.ttl_ms=250;msg.reason='SYNTHETIC_TEST'
            pub.publish(frozen or msg);last=time.monotonic()
        if eye_data is not None and time.monotonic()-last_eye>=.025:
            eye_pub.publish(Float32MultiArray(data=eye_data));last_eye=time.monotonic()
        if drive_target is not None and time.monotonic()-last_drive>=.05:
            cmd=Twist();cmd.linear.x=drive_target;base_pub.publish(cmd);last_drive=time.monotonic()
        rclpy.spin_once(node,timeout_sec=.005)

def ready():
    global mode,enabled
    mode=0;enabled=False;end=time.monotonic()+10
    while time.monotonic()<end:
        spin(.1)
        if state['status'] and state['status'].ready_to_rearm:return
    raise AssertionError('No ready handshake: '+str(state['status']))
def speed():return state['odom'].twist.twist.linear.x,state['odom'].twist.twist.angular.z
def angle():
    q=state['odom'].pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def children(parent):
    result=[]
    for d in Path('/proc').iterdir():
        if not d.name.isdigit():continue
        try:
            stat=(d/'stat').read_text().split(') ')[1].split();ppid=int(stat[1])
            if ppid==parent:result.append(int(d.name));result+=children(int(d.name))
        except (OSError,ValueError,IndexError):pass
    return result
def own_child(executable):
    for pid in children(process.pid):
        try:cmd=Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\x00',b' ').decode()
        except OSError:continue
        if '/lib/'+executable in cmd:return pid
    raise AssertionError('Owned process not found: '+executable)

traces=[];measured=[]
node.create_subscription(String,'/wheelchair/decision_trace',lambda m:traces.append(json.loads(m.data)),10)
node.create_subscription(Odometry,'/wheelchair/odom',lambda m:measured.append((m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*(m.pose.pose.orientation.w*m.pose.pose.orientation.z+m.pose.pose.orientation.x*m.pose.pose.orientation.y),1-2*(m.pose.pose.orientation.y**2+m.pose.pose.orientation.z**2)))),qos_profile_sensor_data)
result={'scenario':args.scenario};extra=[];protected_contacts=None
try:
    deadline=time.monotonic()+35
    while (state['odom'] is None or state['status'] is None) and time.monotonic()<deadline:spin(.1)
    assert state['odom'] is not None,'No actual Gazebo odometry'
    if args.scenario=='side_choice':
        spin(1.);ready();mode=1;enabled=True;deadline=time.monotonic()+15
        while time.monotonic()<deadline and state['odom'].pose.pose.position.x<3.:
            spin(.1)
            assert not state['status'].requires_rearm,state['status'].reason
        assert state['odom'].pose.pose.position.x>=3.,'Did not continue straight past side door'
        assert abs(state['odom'].pose.pose.position.y)<.1 and abs(angle())<.1,'Automatically entered side door'
        result['straight_past_side_door_m']=state['odom'].pose.pose.position.x
        mode=0;enabled=False;spin(1.)
        req='name: "generic_wheelchair", position: {x: 1.7, y: 0, z: 0.005}, orientation: {w: 1}'
        placed=subprocess.run(['ign','service','-s','/world/wheelchair_arena/set_pose','--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','5000','--req',req],capture_output=True,text=True)
        assert placed.returncode==0 and 'true' in placed.stdout
        spin(1.)
        if state['status'].requires_rearm:ready()
        mode=3;enabled=True;spin(.20);deadline=time.monotonic()+8
        while time.monotonic()<deadline and angle()>-math.pi/2+.12:
            spin(.025)
            assert not state['status'].requires_rearm,state['status'].reason
        mode=0;enabled=False;spin(1.)
        assert abs(angle()+math.pi/2)<.15,angle()
        if state['status'].requires_rearm:ready()
        start=len(traces);mode=1;enabled=True;deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            spin(.1)
            if state['status'].requires_rearm:
                assert state['status'].reason=='PASSAGE_EXIT_CONFIRM_DIRECTION',state['status'].reason
                break
        assert state['odom'].pose.pose.position.y<-1.2,'Human-selected doorway was not traversed'
        assert abs(angle()+math.pi/2)<.2,'Snapped back onto old corridor'
        assert any(t.get('candidate',{}).get('passage',{}).get('kind')=='door' for t in traces[start:]),'New doorway not selected'
        result.update(door_entry_y=state['odom'].pose.pose.position.y,door_heading=angle(),exit_reason=state['status'].reason)
        mode=0;enabled=False;spin(1.)
        (root/'artifacts/trace-side-choice.json').write_text(json.dumps(traces))
    elif args.scenario=='smooth':
        spin(1.);ready();start_x=state['odom'].pose.pose.position.x
        drive.clear();traces.clear();statuses.clear()
        enabled=True;mode=1;spin(5.5)
        first=next(i for i,value in enumerate(drive) if value==(.372,0.))
        stop_count=sum(value==(0.,0.) for value in drive[first:])
        assert stop_count==0,('Spurious STOP while driving through open room',stop_count)
        assert not any(t['reason']=='PASSAGE_WAIT_OBSERVATION_STOP' for t in traces)
        cruise=[t['odom']['v'] for t in traces if t['intent_enabled'] and t['time']>traces[0]['time']+1.5]
        assert cruise and min(cruise)>.35,('Not continuous fixed-speed cruise',min(cruise) if cruise else None)
        result.update(straight_distance_m=state['odom'].pose.pose.position.x-start_x,
            cruise_speed_min=min(cruise),cruise_speed_max=max(cruise),unnecessary_stop_commands=stop_count,
            max_compute_ms=max(t['compute_ms'] for t in traces))
        # Holding the same input must still stop when the wall becomes unsafe.
        spin(7.)
        assert state['status'].requires_rearm and all(abs(v)<.025 for v in speed()),state['status'].reason
        assert state['status'].reason in ('PASSAGE_UNCERTAIN_BLOCKED','PASSAGE_END_OR_BRANCH','PASSAGE_TOO_NARROW'),state['status'].reason
        result['obstacle_stop_reason']=state['status'].reason
        (root/'artifacts/trace-smooth.json').write_text(json.dumps(traces))
    elif args.scenario=='micro_turn':
        from wheelchair_control.planner import Config,Predictor,State
        from wheelchair_control.protocol import LEFT,RIGHT
        measurements=[];spin(1.)
        for side in (LEFT,RIGHT):
            for duration in (.10,.20,.30):
                mode=0;enabled=False;spin(.6)
                if state['status'].requires_rearm:ready()
                initial=angle();enabled=True;mode=side;deadline=time.monotonic()+4
                while time.monotonic()<deadline:
                    spin(.01)
                    if state['execution'] and state['execution'].state==1 and state['execution'].executed_action==side:break
                else:raise AssertionError('No RUNNING feedback for calibration')
                spin(duration);mode=0;enabled=False;spin(1.2)
                actual=abs(math.atan2(math.sin(angle()-initial),math.cos(angle()-initial)))
                prediction=Predictor(Config()).check(State(),side,[],[(-10,-10),(10,-10),(10,10),(-10,10)],duration=duration)
                bound=abs(prediction.endpoint.yaw)
                assert .001<actual<=bound,('Turn is not reliable or exceeds model',duration,actual,bound)
                measurements.append(dict(action=side,command_duration=duration,measured_angle_deg=math.degrees(actual),predicted_envelope_deg=math.degrees(bound)))
        result['calibration']=measurements
    elif args.scenario=='exit_recovery':
        spin(1.);start_x=10.906735444657267;start_y=.873704798336599;start_yaw=-.039533730318275256
        req=f'name: "generic_wheelchair", position: {{x: {start_x}, y: {start_y}, z: 0.005}}, orientation: {{z: {math.sin(start_yaw/2)}, w: {math.cos(start_yaw/2)}}}'
        placed=subprocess.run(['ign','service','-s','/world/wheelchair_arena/set_pose','--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','5000','--req',req],capture_output=True,text=True)
        assert placed.returncode==0 and 'true' in placed.stdout
        spin(1.);ready();measured.clear();traces.clear();statuses.clear()
        mode=2;enabled=True;spin(.2);deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            spin(.05)
            assert not state['status'].requires_rearm,state['status'].reason
            if state['status'].reason=='USER_TURN_LIMIT_REACHED':break
        else:raise AssertionError('Micro-turn did not reach stopped limit')
        assert angle()-start_yaw>.015,'No actual corrective turn'
        assert all(abs(v)<.025 for v in speed()),'Turn limit did not settle'
        result['turn_rad']=angle()-start_yaw;result['corrected_yaw']=angle()
        assert any(t['reason']=='USER_MICRO_TURN' for t in traces)
        # Direct new direction, without R or resetting/teleporting the robot.
        mode=1;deadline=time.monotonic()+15
        while time.monotonic()<deadline and state['odom'].pose.pose.position.x<start_x+1.5:
            spin(.1)
            assert not state['status'].requires_rearm,state['status'].reason
        assert state['odom'].pose.pose.position.x>=start_x+1.5,'Still stuck after small left correction'
        result['travel_m']=state['odom'].pose.pose.position.x-start_x
        mode=0;enabled=False;spin(1.)
        assert all(abs(v)<.025 for v in speed())
        from wheelchair_control.planner import Config
        from wheelchair_control.geometry import clearance,transform,IndexedObstacles
        from wheelchair_sim.scenes import map_obstacles,boundary
        cfg=Config(footprint=((-0.45,-.30),(.55,-.30),(.55,.30),(-.45,.30)))
        obs=IndexedObstacles(b.polygon() for b in map_obstacles('clearpath_office'))
        gap=min(clearance(transform(cfg.footprint,pose),obs,boundary('clearpath_office'),cfg.padding) for pose in measured)
        assert gap>cfg.padding,(gap,cfg.padding)
        result.update(minimum_measured_clearance=gap,hard_padding=cfg.padding,max_compute_ms=max(t['compute_ms'] for t in traces))
        (root/'artifacts/trace-exit_recovery.json').write_text(json.dumps(traces))
    elif args.scenario in ('passage','door'):
        spin(1.);yaw=math.radians(args.heading)
        req=f'name: "generic_wheelchair", position: {{x: {args.initial_x}, y: {args.lateral}, z: 0.005}}, orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}'
        placed=subprocess.run(['ign','service','-s','/world/wheelchair_arena/set_pose','--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','5000','--req',req],capture_output=True,text=True)
        assert placed.returncode==0 and 'true' in placed.stdout
        spin(1.);measured.clear();traces.clear();statuses.clear()
        if args.expect_blocked:
            mode=1;enabled=True;spin(3.)
            assert state['status'].requires_rearm and abs(speed()[0])<.02
            assert state['odom'].pose.pose.position.x<args.initial_x+.2
            assert state['status'].reason.startswith(('INITIAL_HARD_BOUNDARY','PASSAGE_')),state['status'].reason
            result['correctly_rejected']=state['status'].reason
        else:
            ready();enabled=True;mode=1;deadline=time.monotonic()+30
            while time.monotonic()<deadline and state['odom'].pose.pose.position.x<args.initial_x+2.1:
                spin(.1)
                if state['status'].requires_rearm:
                    if args.scenario=='door' and state['status'].reason=='PASSAGE_EXIT_CONFIRM_DIRECTION':break
                    if args.expect_exit and state['status'].reason=='PASSAGE_END_OR_BRANCH':break
                    raise AssertionError('Passage rejected: '+state['status'].reason)
            assert state['odom'].pose.pose.position.x>=args.initial_x+(1.2 if args.expect_exit else 1.8 if args.scenario=='door' else 2.1),'No passage progress'
            result['travel_m']=state['odom'].pose.pose.position.x-args.initial_x
            if args.expect_exit:
                assert state['status'].reason=='PASSAGE_END_OR_BRANCH'
                result['exit_reason']=state['status'].reason
            result['corrections']=max((t.get('candidate',{}).get('corrections',0) for t in traces),default=0)
            mode=0;enabled=False;spin(1.)
            assert all(abs(v)<.025 for v in speed())
            from wheelchair_control.planner import Config
            from wheelchair_control.geometry import clearance,transform,IndexedObstacles
            from wheelchair_sim.scenes import map_obstacles,boundary
            cfg=Config(footprint=((-0.45,-.30),(.55,-.30),(.55,.30),(-.45,.30)),margin=.06)
            scene=args.test_scene or ('passage_090' if args.scenario=='passage' else 'door_090')
            obs=IndexedObstacles(b.polygon() for b in map_obstacles(scene))
            gap=min(clearance(transform(cfg.footprint,pose),obs,boundary(scene),cfg.padding) for pose in measured)
            assert gap>cfg.padding,('Hard clearance violated',gap,cfg.padding)
            result['minimum_measured_clearance']=gap;result['hard_padding']=cfg.padding
        result.update(scene=args.test_scene or args.scenario,initial_heading_deg=args.heading,lateral_offset=args.lateral,max_compute_ms=max((t['compute_ms'] for t in traces),default=0))
        (root/'artifacts'/('trace-'+args.scenario+'.json')).write_text(json.dumps(traces))
    elif args.scenario=='corridor':
        # Replay the user's exact approach from the saved ROS trace. All motion
        # after this stationary placement uses the regular protected intent path.
        spin(1.)
        pose=(2.8325045531343336,.9422187278698693,.3740525872945759)
        req=f'name: "generic_wheelchair", position: {{x: {pose[0]}, y: {pose[1]}, z: 0.005}}, orientation: {{z: {math.sin(pose[2]/2)}, w: {math.cos(pose[2]/2)}}}'
        completed=subprocess.run(['ign','service','-s','/world/wheelchair_arena/set_pose',
            '--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','5000','--req',req],capture_output=True,text=True)
        assert completed.returncode==0 and 'true' in completed.stdout,completed.stdout+completed.stderr
        spin(1.);ready();enabled=True;mode=1;spin(5.)
        assert state['status'].requires_rearm,'Expected bounded assistance to stop'
        first_reason=state['status'].reason
        assert first_reason in ('ASSIST_TIME_LIMIT','ASSIST_ALREADY_USED'),first_reason
        first_x=state['odom'].pose.pose.position.x
        ready();start=len(samples);enabled=True;mode=1;spin(1.5)
        assert any(m.action==1 for m in samples[start:]),'Fresh confirmation still blocked by expired assist'
        assert state['status'].reason!='ASSIST_TIME_LIMIT','Old timer still locked a new authorization'
        # The remaining cumulative angle budget may legitimately reject another
        # automatic correction; explicit RIGHT must still be usable below.
        result['rearm_reason']=state['status'].reason
        recovered_x=state['odom'].pose.pose.position.x
        mode=0;enabled=False;spin(1.2)
        # User explicitly selects RIGHT to align with the corridor, then FORWARD.
        # This is a deliberate new heading, not an automatic reset of assist bounds.
        if state['status'].requires_rearm:ready()
        start=len(samples);enabled=True;mode=3;end=time.monotonic()+4
        while time.monotonic()<end and angle()>.14:spin(.025)
        assert any(m.action==3 for m in samples[start:]),'Safe explicit right turn rejected'
        mode=0;enabled=False;spin(1.1)
        aligned_yaw=angle()
        assert abs(aligned_yaw)<.13,('Did not align',aligned_yaw)
        if state['status'].requires_rearm:ready()
        start=len(statuses);begin_x=state['odom'].pose.pose.position.x
        enabled=True;mode=1;spin(5.)
        # If a further bounded correction is needed, require a fresh user grant.
        if state['status'].requires_rearm:
            assert state['status'].reason in ('ASSIST_TIME_LIMIT','ASSIST_ALREADY_USED'),state['status'].reason
            ready();enabled=True;mode=1;spin(4.)
        assert state['odom'].pose.pose.position.x>begin_x+1.,'Did not travel through corridor'
        assert not any(m.state==3 for m in statuses[start:]),'Unexpected control fault'
        result.update(original_stop_reason=first_reason,first_stop_x=first_x,
            recovered_x=recovered_x,aligned_yaw=aligned_yaw,
            corridor_travel_m=state['odom'].pose.pose.position.x-begin_x)
        mode=0;enabled=False;spin(1.)
        assert all(abs(v)<.025 for v in speed())
    elif args.scenario=='eye':
        publish=False
        eye_data=[0.,0.,1.]
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            spin(.1)
            if state['status'] and state['status'].ready_to_rearm:break
        assert state['status'].ready_to_rearm,'Eye disabled STOP handshake did not become ready'
        eye_data=[0.,1.,1.];spin(.15)
        eye_data=[0.,0.,1.];spin(.15)
        eye_data=[0.,1.,1.];spin(.15)
        eye_data=[0.,0.,1.];spin(2.)
        eye_data=[0.,0.,0.];spin(.8)
        assert any(m.action==1 for m in samples),'Eye never enabled FORWARD'
        assert state['status'].requires_rearm and state['status'].executed_action==0,'Eye invalid did not stop'
        assert abs(speed()[0])<.02
        eye_data=[0.,0.,1.];spin(.8)
        assert state['status'].requires_rearm and state['status'].executed_action==0,'Signal recovery auto-restarted'
        result['eye_motion_and_invalid_stop']=True;result['eye_recovery_requires_confirmation']=True
    elif args.scenario=='both_blocked':
        spin(1.);mode=1;enabled=True;spin(3.)
        assert state['status'].requires_rearm and state['status'].executed_action==0
        assert abs(state['odom'].pose.pose.position.x)<.02
        result['no_safe_action_stopped']=True
    else:
        ready();mode=1;enabled=True
        if args.scenario=='b1':
            spin(8.)
            assert any(m.action==1 for m in samples),'Never moved'
            assert state['status'].requires_rearm and state['status'].executed_action==0,'Did not block'
            x=state['odom'].pose.pose.position.x
            assert .2<x<1.25,(x,state['status'])
            assert abs(speed()[0])<.02 and state['contacts']==0
            spin(1.)
            assert abs(state['odom'].pose.pose.position.x-x)<.01,'Old heartbeat restarted motion'
            result['stop_x_m']=x;result['persistent_forward_did_not_unlock']=True
            ready();mode=2;enabled=True;start_yaw=angle();spin(1.5)
            assert angle()>start_yaw+.08,('left turn failed',state['status'])
            begin=len(drive);mode=3;spin(2.)
            segment=drive[begin:]
            assert any(v==0 and w==0 for v,w in segment) and any(w<0 for v,w in segment),'No stopped direction conversion'
            assert not any(v!=0 for v,w in segment),'Explicit RIGHT generated forward'
            mode=0;enabled=False;spin(1.)
            assert all(abs(v)<.025 for v in speed())
            result['turn_conversion_and_user_stop']=True
        elif args.scenario=='office':
            spin(14.)
            assert any(m.action==1 for m in samples),'Office never moved'
            assert state['status'].requires_rearm and state['status'].executed_action==0,'Office obstacle did not stop motion'
            assert state['odom'].pose.pose.position.x>1.,'Office stopped before meaningful travel'
            assert abs(speed()[0])<.025 and state['contacts']==0
            result['protected_stop_x_m']=state['odom'].pose.pose.position.x
            protected_contacts=state['contacts']
            # Positive control in this isolated simulation verifies imported collision mesh.
            os.kill(own_child('wheelchair_control/executor'),signal.SIGKILL)
            os.kill(own_child('wheelchair_control/safety_gate'),signal.SIGKILL)
            drive_target=.372;spin(7.);drive_target=0.;spin(1.)
            assert state['contacts']>0,'Imported office collision mesh did not produce contacts'
            result['positive_control_contacts']=state['contacts']
            drive_target=None
        elif args.scenario=='assist':
            spin(9.)
            assert state['contacts']==0
            result['assisting_seen']=any(m.state==1 for m in statuses)
            result['reasons']=sorted(set(m.reason for m in statuses))
            assert result['assisting_seen'],'No bounded assist selected in offset scene'
            assert any(abs(w)>.1 for v,w in drive),'Assist did not physically execute a turn'
            result['yaw_rad']=angle();result['lateral_m']=state['odom'].pose.pose.position.y
            assert any(m.reason=='ASSIST_FORWARD' for m in samples),'Assist never reached safe forward stage'
            turn_at=next(i for i,(_,w) in enumerate(drive) if abs(w)>.1)
            assert any(v>.3 for v,w in drive[turn_at+1:]),'Forward after turn was cancelled before execution'
            assert abs(angle())<=math.radians(15)+.01
            assert abs(state['odom'].pose.pose.position.y)<=.25+.01
            assert state['status'].requires_rearm
        elif args.scenario=='rotation_wall':
            mode=2;spin(2.)
            assert state['status'].requires_rearm and state['status'].executed_action==0
            assert abs(angle())<.03
            result['footrest_rotation_rejected']=True
        elif args.scenario=='narrow':
            spin(3.)
            assert state['odom'].pose.pose.position.x>.3
            mode=2;spin(1.5)
            assert state['status'].requires_rearm and state['status'].executed_action==0
            result['narrow_forward_then_turn_blocked']=True
        elif args.scenario=='dynamic':
            spin(8.)
            assert state['status'].requires_rearm and state['status'].executed_action==0
            assert state['odom'].pose.pose.position.x<1.25
            result['conservative_moving_obstacle_corridor']=True
        elif args.scenario=='sensor':
            spin(1.5);pid=own_child('wheelchair_sim/obstacle_map');os.kill(pid,signal.SIGSTOP)
            try:
                spin(.8)
                assert state['status'].requires_rearm and state['status'].executed_action==0
                os.kill(pid,signal.SIGCONT);spin(.7)
                assert state['status'].requires_rearm and state['status'].executed_action==0,'Sensor recovery auto-restarted'
            finally:
                try:os.kill(pid,signal.SIGCONT)
                except ProcessLookupError:pass
            result['map_loss_and_recovery_latch']=True
        elif args.scenario=='pause':
            spin(1.2)
            def pause(value):
                subprocess.run(['ign','service','-s','/world/wheelchair_arena/control','--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean','--timeout','1000','--req','pause: '+value],check=True,stdout=log,stderr=log)
            pause('true');spin(.7);pause('false');spin(1.)
            assert state['status'].requires_rearm and state['status'].executed_action==0,'Pause/resume restarted old input'
            result['pause_resume_latch']=True
        elif args.scenario=='executor_exit':
            spin(1.4);pid=own_child('wheelchair_control/executor');os.kill(pid,signal.SIGKILL);spin(1.2)
            assert all(abs(v)<.025 for v in speed()),'Base watchdog did not stop physical robot'
            assert state['status'].requires_rearm
            result['executor_death_base_watchdog']=True
            restarted=subprocess.Popen(['ros2','run','wheelchair_control','executor','--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);extra.append(restarted)
            spin(1.5)
            assert state['status'].requires_rearm and state['status'].executed_action==0,'Executor restart accepted old intent'
            assert all(abs(v)<.025 for v in speed())
            result['executor_restart_requires_rearm']=True
        elif args.scenario=='replay':
            spin(1.2)
            frozen=ControlIntent();frozen.header.stamp=node.get_clock().now().to_msg();frozen.header.frame_id='base_link';frozen.session_id=session;frozen.sequence=sequence;frozen.command=1;frozen.enabled=True;frozen.valid=True;frozen.ttl_ms=250
            spin(.6);frozen=None;spin(.6)
            assert state['status'].requires_rearm and state['status'].executed_action==0
            result['replay_rejected']=True
        # Complete loss of upstream lease must stop regardless of previous phase.
        publish=False;spin(.7)
        assert all(abs(v)<.025 for v in speed())
    fixed={(0.,0.),(.372,0.),(0.,.389),(0.,-.403)}
    assert set(drive)<=fixed,('Non-fixed command!',set(drive))
    assert (state['contacts'] if protected_contacts is None else protected_contacts)==0, 'Physical collision'
    result.update(robot_contacts=state['contacts'] if protected_contacts is None else protected_contacts,drive_setpoints=sorted(set(drive)),status='PASS')
    (root/'artifacts'/('result-'+args.scenario+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
finally:
    mode=0;enabled=False;publish=True;frozen=None
    try:spin(.15)
    except Exception:pass
    node.destroy_node();rclpy.shutdown()
    for p in extra:
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL)
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=12)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    log.close()
