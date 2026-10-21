"""Eye pause/resume commands must pass the existing planner rearm handshake."""
from eye_shared_control.core import EyeMachine
from wheelchair_control.protocol import Packet, STOP, FORWARD
from wheelchair_control.planner import Planner, Config, State

BOUND=[(-10.,-10.),(10.,-10.),(10.,10.),(-10.,10.)]

class System:
    def __init__(self):
        self.eye=EyeMachine();self.planner=Planner(Config())
        self.now=0.;self.seq=0;self.obstacles=[];self.healthy=True
        self.action=STOP
    def step(self,seconds,h=0.,closed=False,valid=True):
        for _ in range(round(seconds/.02)):
            self.now+=.02;self.seq+=1
            self.eye.status(self.now,'safety-test',self.seq,self.planner.latch,self.planner.ready)
            intent=self.eye.signal(self.now,h,closed,valid)
            self.planner.intent(Packet('eye-test',self.seq,self.now,.25,int(intent.command),intent.enabled,intent.valid),self.seq==1)
            decision=self.planner.tick(State(),self.obstacles,BOUND,self.now,healthy=self.healthy,stopped=True)
            self.action=decision.action
        return self.action
    def arm(self):
        self.step(.4)
        for _ in range(2):self.step(.12,closed=True);self.step(.14)
        return self.step(.4)
    def resume(self):
        self.step(.2);self.step(.12,closed=True)
        return self.step(.2)


def test_pause_resume_and_obstacle_interlock():
    s=System();assert s.arm()==FORWARD
    assert s.step(6.)==FORWARD
    assert s.step(.7,closed=True)==STOP
    assert s.step(1.)==STOP
    assert s.resume()==FORWARD
    s.obstacles=[[(.2,-.1),(.3,-.1),(.3,.1),(.2,.1)]]
    assert s.step(.1)==STOP
    assert s.resume()==STOP
    assert s.planner.latch
    s.obstacles=[]
    assert s.step(1.)==STOP
    assert s.resume()==FORWARD


def test_fault_and_invalid_input_cannot_resume_from_center_alone():
    s=System();assert s.arm()==FORWARD
    s.healthy=False
    assert s.step(.1)==STOP
    assert s.resume()==STOP
    s.healthy=True
    assert s.step(1.)==STOP
    assert s.resume()==FORWARD
    assert s.step(.02,valid=False)==STOP
    assert s.step(1.)==STOP
    assert s.resume()==FORWARD
