#!/usr/bin/env python3
"""Three-level manual controller using existing calibrated HID implementation."""
import json
import os
from pathlib import Path
import signal
from types import SimpleNamespace
from datetime import datetime
import tkinter as tk
import rclpy
import rosbag2_py
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from whill_controller.controller_node import WhillControllerNode
from record_five_directions import TOPICS

KEYS = {'a': 'a', 'q': 'z', 'w': 'w', 'e': 'c', 'd': 'd'}
TARGETS = ('left', 'left-forward', 'forward', 'right-forward', 'right')


def scaled(points, level):
    center = points['steering wheel center']
    result = dict(points)
    for name in TARGETS:
        result[name] = tuple(c + round((p-c)*level/3) for c,p in zip(center, points[name]))
    return result


class Recorder(WhillControllerNode):
    def __init__(self):
        self.writer = None
        self.record_enabled = False
        self.level = 1
        self.trial = 0
        self.tail_job = None
        self.option_keys = set()
        self.static_cache = {}
        self.fresh = {}
        super().__init__()
        self.full_points = dict(self.calibration_coordinates)
        self.calibration_coordinates = scaled(self.full_points, 1)
        self.output = Path(os.environ['IPAD_RECORD_OUTPUT'])
        self.output.mkdir(parents=True, exist_ok=False)
        (self.output/'calibration_original.json').write_bytes(Path(self.calibration_file).read_bytes())
        (self.output/'levels.json').write_text(json.dumps({i:scaled(self.full_points,i) for i in (1,2,3)},indent=2))
        (self.output/'notes.txt').write_text('APP speed setting=\nLoad=\nSurface=\n')
        self.record_label = tk.StringVar()
        tk.Label(self.root, textvariable=self.record_label, fg='blue').pack()
        self.record_button = tk.Button(self.root, command=self.toggle_recording, takefocus=False)
        self.record_button.pack(fill='x', padx=14, pady=8)
        self.refresh_label()
        for topic, typ in TOPICS.items():
            qos=QoSProfile(depth=getattr(self, "sensor_queue_depth", 100),reliability=ReliabilityPolicy.BEST_EFFORT)
            if topic == '/tf_static':
                qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(get_message(typ),topic,lambda m,t=topic:self.capture(t,m),qos)

    def refresh_label(self):
        self.record_label.set(f'档位 {self.level}/3 | {"录制模式" if self.record_enabled else "调整模式：不录制"} | '
                              '1/2/3选档 | 点击按钮切换录制模式 | A左 Q左前 W前 E右前 D右')

        self.record_button.config(text='停止录制模式（返回调整）' if self.record_enabled else '开启录制模式（按方向键开始一段）')

    def toggle_recording(self):
        if self.keys_down & self.movement_keys or self.click_held or self.writer is not None:
            self._set_display('请松开方向键并等待本段保存，再切换录制模式。')
            return
        self.record_enabled = not self.record_enabled
        self.refresh_label()
        self.root.focus_set()

    def capture(self, topic, msg):
        import time
        self.fresh[topic]=time.monotonic()
        if topic == '/tf_static':
            for t in msg.transforms:self.static_cache[(t.header.frame_id,t.child_frame_id)]=t
        if self.writer is not None:
            self.writer.write(topic,serialize_message(msg),self.get_clock().now().nanoseconds)

    def marker(self,event):
        if self.writer is not None:
            data=json.dumps(dict(event=event,level=self.level,trial=self.trial,
                                 coordinates=self.calibration_coordinates))
            self.writer.write('/experiment/marker',serialize_message(String(data=data)),self.get_clock().now().nanoseconds)

    def begin_trial(self, key):
        import time
        if not self.static_cache or any(t not in self.fresh for t in
                                      ['/odin1/odometry_highfreq','/wheelchair/odom']):
            self._set_display('录制未开始：等待里程计和TF。');return False
        self.trial+=1
        self.writer=rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(self.output/f'{self.trial:03d}_level{self.level}_{key}'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
        for topic,typ in {**TOPICS,'/experiment/marker':'std_msgs/msg/String'}.items():
            self.writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=typ,serialization_format='cdr'))
        msg=get_message('tf2_msgs/msg/TFMessage')();msg.transforms=list(self.static_cache.values())
        self.capture('/tf_static',msg);self.marker('key_request')
        self.root.after(50,self.monitor_trial)
        return True

    def monitor_trial(self):
        if self.writer is None:return
        if self.keys_down & self.movement_keys:
            self.root.after(50,self.monitor_trial)
        else:
            self.marker('keys_released')
            self.tail_job=self.root.after(3000,self.finish_trial)

    def finish_trial(self):
        self.marker('trial_end');self.writer=None;self.tail_job=None
        self._set_display('该段已保存；可按下一方向，或点击按钮返回调整模式。')

    def on_key_press(self,event):
        key=event.keysym.lower()
        if key == 'o':return 'break'
        if key in ('1','2','3'):
            if key in self.option_keys:return 'break'
            self.option_keys.add(key)
            if self.keys_down & self.movement_keys or self.click_held or self.writer is not None:
                self._set_display('请松开方向键并等待本段保存，再切换档位/模式。');return 'break'
            self.level=int(key);self.calibration_coordinates=scaled(self.full_points,self.level)
            self.refresh_label();return 'break'
        if key in KEYS:
            mapped=KEYS[key]
            if self.tail_job is not None:return 'break'
            held=self.keys_down & self.movement_keys
            if held and mapped not in held:return 'break'
            if self.record_enabled and self.writer is None:
                if not self.keyboard_armed or self.calibrating:return 'break'
                if not self.begin_trial(key):return 'break'
            return super().on_key_press(SimpleNamespace(keysym=mapped))
        if key in self.movement_keys:return 'break'
        return super().on_key_press(event)

    def on_key_release(self,event):
        key=event.keysym.lower()
        if key in ('1','2','3'):
            self.option_keys.discard(key);return 'break'
        return super().on_key_release(SimpleNamespace(keysym=KEYS.get(key,key)))

    def close(self):
        super().close()
        self.marker('controller_closed');self.writer=None


def main():
    rclpy.init();node=None
    try:
        node=Recorder()
        for sig in (signal.SIGINT,signal.SIGTERM):
            signal.signal(sig,lambda *_:node.root.after_idle(node.close))
        node.run()
    finally:
        if node is not None:
            node.writer=None;node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
