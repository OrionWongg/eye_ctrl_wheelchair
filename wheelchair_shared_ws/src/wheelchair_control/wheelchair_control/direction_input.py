"""Directional keyboard: one event per key gesture, independent link heartbeat."""
import argparse
import os
import select
import sys
import termios
import time
import tty
import uuid
from .direction_manager import START, PREFER_LEFT, PREFER_RIGHT, CANCEL_PREFERENCE, STOP_EVENT, TURN_AROUND


class KeyEvents:
    def __init__(self, quiet=.6):
        self.quiet=quiet
        self.last_key=None
        self.last_at=float('-inf')

    def key(self,key,now):
        kinds={'w':START,'a':PREFER_LEFT,'d':PREFER_RIGHT,'u':TURN_AROUND,'c':CANCEL_PREFERENCE,' ':STOP_EVENT,'s':STOP_EVENT}
        kind=kinds.get(key)
        repeated=key==self.last_key and now-self.last_at<self.quiet
        self.last_key,self.last_at=key,now
        return kind if kind is not None and (kind==STOP_EVENT or not repeated) else None


def navigation_notice(reason):
    if reason.startswith('TURN_ADJUST_HELP_REQUIRED:'):
        return '已停车：没有可安全完成的前进＋转向组合，需要协助。'
    if reason=='TURN_ADJUST_SEARCH':
        return '原地转向空间不足，正在搜索前方调整位置。'
    if reason=='TURN_ADJUST_ROUTE_READY':
        return '已找到调整位置，将短距离前进，停稳后转向。'
    return None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--quiet-interval',type=float,default=.6,
                        help='terminal has no key-up; minimum silent interval for a new same-key gesture')
    options,ros_args=parser.parse_known_args()
    if options.quiet_interval<=0 or not sys.stdin.isatty():
        parser.error('requires an interactive terminal and positive quiet interval')
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from std_msgs.msg import Empty
    from wheelchair_interfaces.msg import DirectionEvent,InputHealth,IntentAck,NavigationIntentStatus
    from .protocol import ClockGuard
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    node=Node('direction_keyboard')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.VOLATILE)
    eqos=QoSProfile(depth=10,durability=DurabilityPolicy.VOLATILE)
    events=node.create_publisher(DirectionEvent,'/wheelchair/keyboard/direction_event',eqos)
    health=node.create_publisher(InputHealth,'/wheelchair/keyboard/health',qos)
    estop=node.create_publisher(Empty,'/wheelchair/emergency_stop',qos)
    session=uuid.uuid4().hex
    keys=KeyEvents(options.quiet_interval)
    clock=ClockGuard()
    pending={}
    sequence=counter=0
    def ack(m):
        if m.source_session_id==session:
            if pending.pop(m.event_id,None) is not None:
                label='已接受' if m.status==1 else '已拒绝'
                print(f'指令 {m.event_id} {label}: {m.reason}',flush=True)
    node.create_subscription(IntentAck,'/wheelchair/intent_ack',ack,eqos)
    notices=set()
    def status(m):
        if m.source_session_id!=session:return
        message=navigation_notice(m.reason)
        if message and message not in notices:
            print(message,flush=True);notices.add(message)
    node.create_subscription(NavigationIntentStatus,'/wheelchair/navigation_status',status,qos)
    def submit(kind):
        nonlocal counter
        notices.clear()
        if kind==STOP_EVENT:
            pending.clear();estop.publish(Empty())
        counter+=1
        m=DirectionEvent();m.header.stamp=node.get_clock().now().to_msg();m.protocol_version=1
        m.source_session_id=session;m.event_id=counter;m.kind=kind;m.ttl_ms=1500
        events.publish(m)
        if kind!=STOP_EVENT:pending[counter]=(m,time.monotonic(),time.monotonic())
    old=termios.tcgetattr(sys.stdin)
    print('方向意图模式：W 启动/继续；A/D 左/右偏好；U 掉头；C 取消偏好；空格/S 停止；X 退出。',flush=True)
    print('停车时 A/D 转向一次，U 掉头，完成后保持停车；行驶时 A/D 为方向偏好。W 启动直行。',flush=True)
    print('原地转向受阻时会自动搜索前方最多 60 cm 的调整位置；无安全组合则停车提示需要协助。',flush=True)
    next_health=0.;previous_valid=False
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.01)
            now=time.monotonic();ros_now=node.get_clock().now().nanoseconds/1e9
            valid=(ros_now>0 and not clock.update(ros_now,now)
                   and os.tcgetpgrp(sys.stdin.fileno())==os.getpgrp())
            if previous_valid and not valid:submit(STOP_EVENT)
            previous_valid=valid
            if now>=next_health:
                sequence+=1
                h=InputHealth();h.header.stamp=node.get_clock().now().to_msg();h.protocol_version=1
                h.source_session_id=session;h.sequence=h.sample_sequence=sequence
                h.source_valid=valid;h.source_age_ms=0.;h.reception_timed=True;h.ttl_ms=250
                health.publish(h);next_health=now+.05
            if select.select([sys.stdin],[],[],0)[0]:
                key=sys.stdin.read(1).lower()
                if key in ('','x','\x03','\x04'):break
                kind=keys.key(key,now)
                if kind is not None and (valid or kind==STOP_EVENT):submit(kind)
            for event_id,(m,born,sent) in list(pending.items()):
                if now-born>=1.5 or not valid:pending.pop(event_id,None)
                elif now-sent>=.1:
                    events.publish(m);pending[event_id]=(m,born,now)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old)
        if rclpy.ok():
            submit(STOP_EVENT)
            for _ in range(3):rclpy.spin_once(node,timeout_sec=.02)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
