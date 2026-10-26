"""Directional keyboard: one event per key gesture, independent link heartbeat."""
import argparse
import json
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


class HoldToRun:
    """A GUI key-down lease; heartbeat loss must revoke motion authorization."""
    motion_keys = frozenset(('w', 'a', 'd', 'u'))

    def __init__(self, timeout=.20):
        self.timeout = timeout
        self.held = set()
        self.last_at = float('-inf')

    def update(self, key, action, now):
        self.active(now)
        if action == 'release':
            self.held.discard(key)
        elif key in self.motion_keys and action in ('press', 'hold'):
            # A heartbeat can renew an existing gesture, never create one.
            if action == 'press' or key in self.held:
                self.held.add(key)
                self.last_at = now
        elif key in (' ', 's'):
            self.held.clear()
        return self.active(now)

    def active(self, now):
        if now - self.last_at >= self.timeout:
            self.held.clear()
        return bool(self.held)


def gui_key(data, ros_now):
    """Reject queued/stale GUI gestures instead of granting a new motion lease."""
    try:
        payload = json.loads(data)
        key, stamp = payload['key'], payload['stamp']
        if (key in ('w', 'a', 'd', 'u', 'c', ' ', 's')
                and payload.get('action', 'press') in ('press', 'hold', 'release')
                and 0 <= ros_now - stamp < .25):
            return key
    except (ValueError, TypeError, KeyError):
        pass
    return None


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
    parser.add_argument('--hold-to-run', action='store_true',
                        help='GUI key-down/up with a 200 ms lease; terminal accepts stop/exit only')
    options,ros_args=parser.parse_known_args()
    if options.quiet_interval<=0 or not sys.stdin.isatty():
        parser.error('requires an interactive terminal and positive quiet interval')
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from std_msgs.msg import Empty,String
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
    hold=HoldToRun()
    hold_was_active=False
    clock=ClockGuard()
    pending={}
    sequence=counter=0
    def ack(m):
        if m.source_session_id==session:
            if pending.pop(m.event_id,None) is not None:
                label='已接受' if m.status==1 else '已拒绝'
                node.get_logger().info(f'指令 {m.event_id} {label}: {m.reason}')
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
        node.get_logger().info(f'已发送键盘指令 {counter}，等待控制器回执（kind={kind}）')
        if kind!=STOP_EVENT:pending[counter]=(m,time.monotonic(),time.monotonic())
    def receive_key(key, valid, origin):
        kind=keys.key(key,time.monotonic())
        if kind is None:return
        if valid or kind==STOP_EVENT:
            node.get_logger().info(f'收到{origin}按键 {key.upper()!r}')
            submit(kind)
        else:
            node.get_logger().warning('按键未发送：终端不在前台进程组，或控制时钟无效')
    def publish_health(valid):
        nonlocal sequence
        sequence+=1
        h=InputHealth();h.header.stamp=node.get_clock().now().to_msg();h.protocol_version=1
        h.source_session_id=session;h.sequence=h.sample_sequence=sequence
        h.source_valid=valid;h.source_age_ms=0.;h.reception_timed=True;h.ttl_ms=250
        health.publish(h)

    def on_gui_key(message):
        nonlocal hold_was_active
        ros_now=node.get_clock().now().nanoseconds/1e9
        key=gui_key(message.data,ros_now)
        if key is not None:
            valid=(previous_valid and ros_now>0
                   and os.tcgetpgrp(sys.stdin.fileno())==os.getpgrp())
            if options.hold_to_run:
                payload=json.loads(message.data)
                action=payload.get('action', 'press')
                active=hold.update(key,action,time.monotonic()-(ros_now-payload['stamp'])) if valid else False
                active=active and hold.active(time.monotonic())
                publish_health(valid and active)
                if hold_was_active and not active:
                    submit(STOP_EVENT)
                    keys.last_key=None
                hold_was_active=active
                if action != 'press':return
                # Heartbeats never retry a rejected gesture or rearm a fault.
                if key in hold.motion_keys:
                    keys.last_key=None
            receive_key(key,valid and (not options.hold_to_run or active or key=='c'),'GUI')
        else:
            node.get_logger().warning('GUI 按键已丢弃：消息无效或超过 250 ms')
    node.create_subscription(String,'/wheelchair/keyboard/gui_key',on_gui_key,eqos)
    old=termios.tcgetattr(sys.stdin)
    print('方向意图模式：W 启动/继续；A/D 左/右偏好；U 掉头；C 取消偏好；空格/S 停止；X 退出。',flush=True)
    print('停车时 A/D 转向一次，U 掉头，完成后保持停车；行驶时 A/D 为方向偏好。W 启动直行。',flush=True)
    print('原地转向受阻时会自动搜索前方最多 60 cm 的调整位置；无安全组合则停车提示需要协助。',flush=True)
    if options.hold_to_run:
        print('实车按住运行：在 iPad GUI 按住 W/A/D/U；松键即停。终端仅接受空格/S 停止和 X 退出。',flush=True)
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
            active=hold.active(now) if options.hold_to_run else True
            if options.hold_to_run and hold_was_active and (not active or not valid):
                hold.held.clear();hold_was_active=False;keys.last_key=None
                submit(STOP_EVENT)
                node.get_logger().warning('按键已松开或按住心跳超时，停止运动')
            if now>=next_health:
                publish_health(valid and active);next_health=now+.05
            if select.select([sys.stdin],[],[],0)[0]:
                key=sys.stdin.read(1).lower()
                if key in ('','x','\x03','\x04'):break
                if not options.hold_to_run or key in (' ', 's'):
                    if options.hold_to_run:hold.held.clear()
                    receive_key(key,valid,'终端')
                else:
                    node.get_logger().warning('按住运行模式请使用 iPad GUI；终端没有可靠松键事件')
            for event_id,(m,born,sent) in list(pending.items()):
                if now-born>=1.5 or not valid:
                    pending.pop(event_id,None)
                    node.get_logger().warning(f'指令 {event_id} 未收到回执或输入已失效，请检查 direction_manager')
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
