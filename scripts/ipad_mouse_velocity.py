#!/usr/bin/env python3
"""Mouse velocity experiment; releases on mouse-up, loss of focus or stale odometry."""
import time
import json
import tkinter as tk
import numpy as np
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from ipad_three_level_recorder import Recorder,TOPICS
from ipad_velocity_model import map_input, map_velocity, VELOCITIES, TARGETS
import whill_controller.controller_node as controller_module
controller_module.ROS_PUMP_MAX_CALLBACKS = 64
import rclpy
import signal
TOPICS['/experiment/velocity_mapping']='std_msgs/msg/String'
TOPICS['/experiment/cmd_vel']='geometry_msgs/msg/TwistStamped'
TOPICS['/experiment/measured_velocity']='geometry_msgs/msg/TwistStamped'

class MouseController(Recorder):
    def __init__(self):
        self.nav_topic="";self.nav_enabled=False;self.nav_received_at=0.
        self.sensor_queue_depth=2
        self.link_status="尚未收到cmd_vel";self.command_at=0.;self.received_target=np.zeros(2)
        self.dragging=False;self.desired=np.zeros(2);self.actual=np.zeros(2)
        self.control_xy=np.zeros(2);self.pose=None;self.origin=None
        self.predicted_pose=np.zeros(3);self.predicted_path=[];self.actual_path=[];self.speed_history=[];self.mouse_history=[]
        self.odom_at=0.;self.odom_stamp_age=None;self.mount=None;self.last_tick=time.monotonic()
        super().__init__()
        self.nav_topic=str(self.declare_parameter('cmd_vel_input_topic','').value)
        self.ros_pump_interval_ms=5
        self.level=3;self.calibration_coordinates=dict(self.full_points)
        self.readiness_log=self.output/'readiness.jsonl'
        self.last_readiness=None
        self.record_label.set('鼠标速度实验：下沿中点=停止，上=前进，左/右=转向；松开停止。')
        self.mapping_pub=self.create_publisher(String,'/experiment/velocity_mapping',10)
        self.pub=self.create_publisher(TwistStamped,'/experiment/cmd_vel',10)
        self.measured_pub=self.create_publisher(TwistStamped,'/experiment/measured_velocity',10)
        self.test_window=tk.Toplevel(self.root)
        self.test_window.title('鼠标直接操控与运动对照')
        self.test_window.protocol('WM_DELETE_WINDOW',self.close)
        tk.Label(self.test_window,text='按住左键直接操控iPad，松开停止；蓝色预测，红色实测；不自动修正输入').pack()
        self.info=tk.StringVar(value='等待 odom 与 TF')
        tk.Label(self.test_window,textvariable=self.info).pack()
        self.canvas=tk.Canvas(self.test_window,width=420,height=220,bg='white');self.canvas.pack()
        self.canvas.create_line(210,0,210,220,fill='gray');self.canvas.create_text(210,205,text='停止')
        self.canvas.bind('<ButtonPress-1>',self.press)
        self.canvas.bind('<B1-Motion>',self.motion)
        self.root.bind_all('<ButtonRelease-1>',lambda e:self.stop_mouse(),add='+')
        self.root.bind('<FocusOut>',self.on_mouse_focus_out,add='+')
        self.path_canvas=tk.Canvas(self.test_window,width=420,height=220,bg="white");self.path_canvas.pack()
        self.speed_canvas=tk.Canvas(self.test_window,width=420,height=150,bg="white");self.speed_canvas.pack()
        self.test_window.bind("<FocusOut>",self.on_mouse_focus_out,add="+")
        self.root.report_callback_exception=self.on_callback_error
        if self.nav_topic:
            self.create_subscription(Twist,self.nav_topic,self.receive_nav_velocity,1)
            self.nav_button=tk.Button(self.test_window,text='允许Nav2速度输入（需Arm）',command=self.toggle_nav)
            self.nav_button.pack()
        self.draw_levels()
        self.root.after(100,self.tick)

    def on_callback_error(self, exc_type, exc, tb):
        import traceback
        self.dragging=False
        try:
            self.stop_mouse()
            if self.keyboard_armed:self.toggle_keyboard_arm()
        finally:
            self.link_status=f'控制异常，已停止：{exc}'
            self.get_logger().error(''.join(traceback.format_exception(exc_type,exc,tb)))

    def refresh_label(self):
        super().refresh_label()
        if hasattr(self,'canvas'):self.record_label.set('鼠标拖动控制 | 松开停止 | 使用按钮切换录制/调整')

    def capture(self,topic,msg):
        super().capture(topic,msg)
        if topic=='/tf_static':
            for tf in msg.transforms:
                if tf.header.frame_id=='base_link' and tf.child_frame_id=='imu':
                    p=tf.transform.translation;q=tf.transform.rotation
                    self.mount=(np.array([p.x,p.y,p.z]),Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix())
        if topic=='/odin1/odometry_highfreq' and self.mount is not None:
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            # Device/header clock offset is diagnostic, not a manual-drive gate.
            self.odom_stamp_age=self.get_clock().now().nanoseconds/1e9-stamp
            q=msg.pose.pose.orientation;v=msg.twist.twist.linear;w=msg.twist.twist.angular
            p,rbi=self.mount;rob=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()@rbi.T
            wb=rbi@np.array([w.x,w.y,w.z]);vb=rob.T@np.array([v.x,v.y,v.z])-np.cross(wb,p)
            # Empirical center from pure rotation trials; not surveyed geometry.
            vc=vb+np.cross(wb,[-.266192,.078385,0])
            pos=msg.pose.pose.position
            center_pos=np.array([pos.x,pos.y,pos.z])-rob@p+rob@np.array([-.266192,.078385,0])
            self.pose=np.array([center_pos[0],center_pos[1],np.arctan2(rob[1,0],rob[0,0])])
            measured=np.array([vc[0],wb[2]])
            if np.all(np.isfinite(measured)):
                self.actual=.7*self.actual+.3*measured;self.odom_at=time.monotonic()
                if hasattr(self,'measured_pub'):
                    m=TwistStamped();m.header.stamp=msg.header.stamp;m.header.frame_id='wheelchair_control_center_estimated'
                    m.twist.linear.x=float(vc[0]);m.twist.angular.z=float(wb[2]);self.measured_pub.publish(m)

    def blockers(self):
        reasons=self.eye_blockers()
        if self.tail_job is not None:reasons.append('停车数据保存中')
        if self.mount is None:reasons.append('等待base_link→imu TF')
        labels={'IPAD_NOT_ARMED':'尚未授权：请点击 Arm Keyboard Control（录制按钮不等于Arm）',
                'IPAD_BLE_NOT_CONNECTED':'ESP32尚未报告BLE已连接',
                'IPAD_APP_MODE_REQUIRED':'当前不是APP锁定模式',
                'IPAD_HOMING_NOT_READY':'iPad指针归中尚未完成'}
        return [labels.get(r,r) for r in reasons]

    def draw_levels(self):
        center=np.asarray(self.full_points['steering wheel center'])
        self.canvas.create_line(10,200,410,200,fill='gray')
        for name,label in zip(TARGETS,('右','右前','前','左前','左')):
            d=np.asarray(self.full_points[name])-center
            for step in range(1,9):
                u=step/8;x=210+d[0]*25*u;y=200+d[1]*23*u
                self.canvas.create_oval(x-2,y-2,x+2,y+2,fill='steelblue')
            self.canvas.create_text(210+d[0]*24,200+d[1]*22-10,text=label)

    def on_mouse_focus_out(self,event):
        def check():
            if not self.closing and self.root.focus_displayof() is None:
                self.stop_mouse()
                if self.nav_enabled:
                    self.nav_enabled=False;self.nav_button.config(text='允许Nav2速度输入（需Arm）')
        self.root.after_idle(check)

    def press(self,event):
        if self.nav_enabled:return
        reasons=self.blockers()
        if reasons:
            self.link_status='拒绝启动: '+', '.join(reasons);self._set_display(self.link_status);return
        if self.record_enabled and not self.begin_trial('mouse'):return
        self.origin=None if self.pose is None else self.pose.copy()
        self.predicted_pose=np.zeros(3);self.predicted_path=[];self.actual_path=[];self.speed_history=[];self.mouse_history=[]
        self.canvas.delete("mouse_trace")
        self.dragging=True;self.motion(event)

    def motion(self,event):
        if self.dragging:
            self.control_xy=np.array([np.clip((event.x-210)/200,-1,1),np.clip((200-event.y)/184,0,1)])
            point,self.desired=map_input(*self.control_xy,self.full_points)
            # Apply the current pointer position immediately; plots never gate motion.
            self.publish_target(self.apply_mouse())
            self.mouse_history.append((event.x,event.y))
            if len(self.mouse_history)>1:
                self.canvas.create_line(*self.mouse_history[-2],*self.mouse_history[-1],fill='green',tags='mouse_trace')

    def stop_mouse(self):
        self.dragging=False;self.desired[:]=0
        if hasattr(self,'keys_down'):
            self.keys_down.difference_update(self.movement_keys)
            if self.click_held:self.update_keyboard_target()
        if hasattr(self,'pub'):self.publish_target(np.zeros(2))

    def publish_target(self,target):
        m=TwistStamped();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='wheelchair_control_center_estimated'
        m.twist.linear.x=float(target[0]);m.twist.angular.z=float(target[1]);self.pub.publish(m)

    def toggle_nav(self):
        self.stop_mouse()
        self.nav_enabled=not self.nav_enabled
        self.nav_received_at=0.
        self.nav_button.config(text='停止Nav2输入' if self.nav_enabled else '允许Nav2速度输入（需Arm）')

    def receive_nav_velocity(self,msg):
        if not self.nav_enabled:return
        values=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
        if (not np.all(np.isfinite(values)) or msg.linear.x<0 or
                any(abs(x)>1e-6 for x in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)) or self.blockers()):
            self.stop_mouse();self.link_status='Nav2指令拒绝：禁止后退/横移，或接口未就绪';return
        self.nav_received_at=time.monotonic()
        v=float(np.clip(msg.linear.x,0,.30));w=float(np.clip(msg.angular.z,-.40,.40))
        point,feasible=map_velocity(v,w,self.full_points)
        center=np.asarray(self.full_points['steering wheel center'])
        delta=np.asarray(point)-center
        self.control_xy=np.array([delta[0]/8,-delta[1]/8])
        self.dragging=bool(v>1e-6 or abs(w)>1e-6)
        if not self.dragging:self.stop_mouse();return
        self.desired=feasible
        self.publish_target(self.apply_mouse())

    def apply_mouse(self):
        point,prediction=map_input(*self.control_xy,self.full_points)
        self.calibration_coordinates['forward']=point
        if point!=self.full_points['steering wheel center']:self.keys_down.add('w')
        else:self.keys_down.discard('w')
        ok=self.update_keyboard_target()
        self.link_status=f'鼠标直接映射 → HID {point} → '+('发送成功' if ok else '失败')
        self.mapping_pub.publish(String(data=json.dumps(dict(input_xy=self.control_xy.tolist(), predicted_velocity=prediction.tolist(), point=list(map(int,point)), applied=bool(ok), feedback=False, reference_offset=[-.266192,.078385,0]))))
        if not ok:self.stop_mouse();return np.zeros(2)
        return prediction

    def plot_motion(self,target,dt):
        if self.origin is None:return
        v,w=target;h=self.predicted_pose[2]
        self.predicted_pose+=np.array([v*np.cos(h)*dt,v*np.sin(h)*dt,w*dt])
        self.predicted_path.append(self.predicted_pose[:2].copy())
        if self.pose is not None:
            d=self.pose[:2]-self.origin[:2];c=np.cos(self.origin[2]);s=np.sin(self.origin[2])
            self.actual_path.append(np.array([c*d[0]+s*d[1],-s*d[0]+c*d[1]]))
        self.predicted_path=self.predicted_path[-600:];self.actual_path=self.actual_path[-600:]
        pc=self.path_canvas;pc.delete('all');pc.create_text(210,12,text='本次运动轨迹（米）：蓝预测 / 红实测')
        paths=self.predicted_path+self.actual_path
        scale=85/max(.5,max((max(abs(p)) for p in paths),default=.5))
        for path,color in [(self.predicted_path,'blue'),(self.actual_path,'red')]:
            if len(path)>1:pc.create_line(*[z for p in path for z in (210-p[1]*scale,195-p[0]*scale)],fill=color)
        pc.create_text(210,210,text=f'刻度：{scale:.0f}像素/米')
        self.speed_history.append([*target,*self.actual]);self.speed_history=self.speed_history[-150:]
        sc=self.speed_canvas;sc.delete('all');sc.create_text(210,10,text='速度历史：上 v（±0.7m/s） / 下 ω（±1rad/s）')
        for col,color,base,scale_y in [(0,'blue',53,35/.7),(2,'red',53,35/.7),(1,'blue',115,30),(3,'red',115,30)]:
            if len(self.speed_history)>1:sc.create_line(*[z for i,p in enumerate(self.speed_history) for z in (10+i*400/150,base-p[col]*scale_y)],fill=color)

    def tick(self):
        now=time.monotonic();gap=now-self.last_tick;self.last_tick=now
        reasons=self.blockers()
        state=tuple(reasons)
        if state!=self.last_readiness:
            self.last_readiness=state
            row=dict(time_ns=self.get_clock().now().nanoseconds,reasons=reasons,armed=self.keyboard_armed,ble=self.bluetooth_connected,calibrating=self.calibrating,odom_receive_age_s=None if not self.odom_at else now-self.odom_at,odom_header_age_s=self.odom_stamp_age)
            with self.readiness_log.open('a') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
            self.get_logger().info('鼠标控制就绪' if not reasons else '鼠标控制未就绪：'+'；'.join(reasons))
        if self.dragging and reasons:
            self.link_status='已停止: '+', '.join(reasons);self.stop_mouse()
        target=np.zeros(2)
        if self.dragging:target=self.apply_mouse()
        self.publish_target(target)
        if self.dragging or self.writer is not None:self.plot_motion(target,min(gap,.35))
        self.info.set(f'预测v={target[0]:.3f} ω={target[1]:.3f} | 实测v={self.actual[0]:.3f} ω={self.actual[1]:.3f}\n{self.link_status}\n'+('就绪' if not reasons else ', '.join(reasons)))
        self.root.after(100,self.tick)

    def monitor_trial(self):
        if self.writer is None:return
        if self.dragging:self.root.after(50,self.monitor_trial)
        else:
            self.marker('mouse_released');self.tail_job=self.root.after(3000,self.finish_trial)

    def on_key_press(self,event):
        if event.keysym.lower()=='escape':
            self.stop_mouse()
            if self.nav_enabled:
                self.nav_enabled=False;self.nav_button.config(text='允许Nav2速度输入（需Arm）')
            return 'break'
        return 'break'

    def on_key_release(self,event):return 'break'

    def close(self):
        self.stop_mouse();super().close()


def main():
    rclpy.init();node=None
    try:
        node=MouseController()
        for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:node.root.after_idle(node.close))
        node.run()
    finally:
        if node is not None:node.writer=None;node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
