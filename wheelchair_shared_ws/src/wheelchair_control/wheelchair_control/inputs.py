"""Human input arbitration. Selection is explicit; no automatic failover."""
import json
import select
import sys
import termios
import time
import tty
import uuid
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Empty,String
from wheelchair_interfaces.msg import ControlIntent,ControlStatus
from .protocol import STOP,FORWARD,LEFT,RIGHT,NAMES,StreamGuard,Packet,ClockGuard

QOS=QoSProfile(depth=1,durability=DurabilityPolicy.VOLATILE)
class IntentMux(Node):
    def __init__(self):
        super().__init__('wheelchair_intent_mux')
        self.declare_parameter('input_source','keyboard')
        source=self.get_parameter('input_source').value
        if source not in ('keyboard','eye'):raise ValueError('input_source must be keyboard or eye')
        self.add_on_set_parameters_callback(lambda p:SetParametersResult(successful=False,reason='Restart disarmed to switch input source'))
        self.pub=self.create_publisher(ControlIntent,'/wheelchair/intent',QOS)
        self.fault=self.create_publisher(String,'/wheelchair/input_fault',QOS)
        self.confirmed=False
        self.create_subscription(ControlIntent,'/wheelchair/'+source+'_intent',self.forward,QOS)
        self.get_logger().info('Selected '+source+'; source stamps/sequence/TTL preserved, no replay cache')

    def forward(self,msg):
        if msg.command==STOP and not msg.enabled:self.confirmed=True
        if not self.confirmed:
            self.fault.publish(String(data='MUX_RESTART_NEEDS_DISABLED_STOP'))
            return
        self.pub.publish(msg)

class Keyboard(Node):
    def __init__(self):
        super().__init__('wheelchair_keyboard')
        self.session=str(uuid.uuid4());self.sequence=0;self.armed=False;self.command=STOP
        self.status=None;self.status_at=-1e20;self.await_ack=False;self.motion_request_at=None
        self.locked=False;self.had_motion=False;self.manual_stop=False;self.stop_status_sequence=-1;self.last_notice=None
        self.guard=StreamGuard(.3);self.clock=ClockGuard();self.reason='STARTUP'
        self.pub=self.create_publisher(ControlIntent,'/wheelchair/keyboard_intent',QOS)
        self.estop=self.create_publisher(Empty,'/wheelchair/emergency_stop',QOS)
        self.create_subscription(ControlStatus,'/wheelchair/control_status',self.on_status,QOS)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def notice(self,text):
        if text!=self.last_notice:print(text,flush=True);self.last_notice=text

    def disarm(self,reason):
        user_stop=reason in ('USER_STOP','KEYBOARD_EXIT')
        expected_stop=self.manual_stop and reason.startswith(('USER_EMERGENCY_STOP','USER_STOP'))
        if not user_stop and not expected_stop and self.had_motion:
            newly_locked=not self.locked;self.locked=True
            if newly_locked:
                explanations={'REARM_REJECTED':'安全层未确认运动请求','STATUS_STALE':'安全状态超时',
                              'STATUS_SESSION_OR_TIMEOUT':'安全节点重启或状态中断','CLOCK_PAUSED':'仿真暂停',
                              'BLOCKED_FORWARD':'前方有障碍','BLOCKED_TURN':'转向空间不足'}
                self.notice(f'已停车：{explanations.get(reason,reason)} [{reason}]。停稳后按 R 恢复，再选择方向。')
        self.armed=False;self.command=STOP;self.reason=reason;self.await_ack=False
        self.motion_request_at=None

    def on_status(self,msg):
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds/1e9
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        ok,changed,reason=self.guard.accept(Packet(msg.session_id,msg.sequence,stamp,.3),ros_now,now)
        if not ok:self.disarm(reason);return
        if changed or now-self.status_at>.3:self.disarm('STATUS_SESSION_OR_TIMEOUT')
        self.status=msg;self.status_at=now
        if msg.requires_rearm and not msg.ready_to_rearm:self.disarm(msg.reason)
        elif not msg.requires_rearm:
            self.await_ack=False;self.motion_request_at=None
        elif self.armed and not self.await_ack:self.disarm(msg.reason)

    def ready(self):
        return (self.status is not None and self.guard.fresh(time.monotonic())
                and self.status.intent_session_id==self.session
                and ((self.status.requires_rearm and self.status.ready_to_rearm and self.status.sequence>self.stop_status_sequence) if self.manual_stop
                     else (self.status.ready_to_rearm or not self.status.requires_rearm)))

    def key(self,key):
        if key in (' ','s'):
            self.manual_stop=True;self.stop_status_sequence=self.status.sequence if self.status else -1;self.disarm('USER_STOP');self.estop.publish(Empty())
            self.notice('已停车。' if not self.locked else '已停车；安全锁定仍在，停稳后按 R 恢复。')
        elif key=='r':
            if not self.locked:return
            if self.ready():
                self.locked=False;self.reason='USER_RECOVERY';self.command=STOP
                self.notice('已恢复。按 W/A/D 选择方向。')
            else:self.notice('尚不能恢复：请等待停稳和传感器恢复。')
        elif key in ('w','a','d'):
            if self.locked:
                self.notice(f'保持停车 [{self.reason}]，按 R 恢复后再选择方向。');return
            if not self.ready():
                self.notice('正在等待仿真或停车确认，请稍后再按方向键。');return
            if not self.armed:
                # A fresh direction key supplies the normal keyboard confirmation.
                # The protocol still performs disabled STOP -> ready -> enabled.
                self.await_ack=self.status.requires_rearm;self.motion_request_at=None
            self.armed=True;self.had_motion=True;self.manual_stop=False
            self.command={'w':FORWARD,'a':LEFT,'d':RIGHT}[key];self.reason='KEYBOARD';self.last_notice=None

    def tick(self):
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds/1e9
        fault=self.clock.update(ros_now,now)
        if fault:self.disarm(fault)
        if not self.guard.fresh(now):self.disarm('STATUS_STALE')
        if self.armed and self.await_ack and self.command!=STOP:
            if self.motion_request_at is None:self.motion_request_at=now
            elif now-self.motion_request_at>.5 and self.status and self.status.requires_rearm:
                self.disarm('REARM_REJECTED')
        msg=ControlIntent();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link'
        self.sequence+=1;msg.session_id=self.session;msg.sequence=self.sequence
        msg.command=self.command if self.armed else STOP;msg.enabled=self.armed;msg.valid=True;msg.ttl_ms=250;msg.reason=self.reason
        self.pub.publish(msg)

def mux_main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=IntentMux()
    try:
        while rclpy.ok():rclpy.spin_once(node,timeout_sec=.1)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
def keyboard_main():
    if not sys.stdin.isatty():raise SystemExit('Use an interactive terminal')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Keyboard();old=termios.tcgetattr(sys.stdin)
    print('W 前进；A/D 转向；空格/S 停车；X 退出。动作保持到切换或停车，遇障碍/故障后按 R 恢复。')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin],[],[],.01)[0]:
                key=sys.stdin.read(1).lower()
                if key in ('','x','\x03'):break
                node.key(key)
            rclpy.spin_once(node,timeout_sec=.01)
    except KeyboardInterrupt:pass
    finally:
        try:termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old)
        except termios.error:pass
        if rclpy.ok():node.disarm('KEYBOARD_EXIT');node.tick();node.estop.publish(Empty())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
def status_main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('wheelchair_status_view');last=[None]
    def show(m):
        line=f'intent={NAMES[m.requested_command]} executed={NAMES[m.executed_action]} state={m.state} rearm={m.requires_rearm} ready={m.ready_to_rearm} {m.reason}'
        if line!=last[0]:print(line,flush=True);last[0]=line
    node.create_subscription(ControlStatus,'/wheelchair/control_status',show,QOS)
    try:
        while rclpy.ok():rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
