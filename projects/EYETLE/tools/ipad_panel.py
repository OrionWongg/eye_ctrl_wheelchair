"""Combined-window iPad controls; the serial controller owns all motion gates."""
import time
import tkinter as tk
from pathlib import Path


class IpadPanel:
    def __init__(self, parent):
        self.send = None
        self.received_at = -float('inf')
        self.feedback = {}
        self.frame = tk.Frame(parent, bg='#162235')
        self.frame.pack(fill='x', padx=20, pady=4)
        row = tk.Frame(self.frame, bg='#162235')
        row.pack(fill='x')
        self.arm = tk.Button(row, text='启用眼动', command=lambda: self.command('arm'),
                             bg='#20835d', fg='white', font=('Noto Sans CJK SC', 12),
                             state='disabled')
        self.arm.pack(side='left', padx=6)
        self.stop = tk.Button(row, text='停止 / 取消启用', command=lambda: self.command('stop'),
                              bg='#a63848', fg='white', font=('Noto Sans CJK SC', 12))
        self.stop.pack(side='left', padx=6)
        self.status = tk.StringVar(value='等待 iPad 控制器连接')
        tk.Label(row, textvariable=self.status, bg='#162235', fg='white',
                 font=('Noto Sans CJK SC', 11)).pack(side='left', padx=12)
        self.calibration = tk.StringVar(value='iPad 标定：等待控制器报告')
        tk.Label(self.frame, textvariable=self.calibration, bg='#162235', fg='#aabed6',
                 font=('Noto Sans CJK SC', 10), anchor='w').pack(fill='x', padx=6)

    def update(self, feedback=None):
        if feedback is not None:
            self.feedback = feedback
            self.received_at = time.monotonic()
        d = self.feedback
        fresh = time.monotonic() - self.received_at < .5
        blockers = d.get('blockers', [])
        can_arm = fresh and blockers == ['IPAD_NOT_ARMED'] and self.send is not None
        self.arm.configure(state='normal' if can_arm else 'disabled',
                           text='已启用' if fresh and d.get('armed') else '启用眼动')
        if not fresh:
            self.status.set('控制器反馈中断 · 禁止启用')
            return
        mode = '真实串口' if d.get('hardware_enabled') else '离线模式'
        ble = '已连接' if d.get('bluetooth_connected') is True else '未连接'
        home = '归位中' if d.get('calibrating') else '已归位'
        armed = '已启用' if d.get('armed') else '未启用'
        self.status.set(f'{mode}  |  蓝牙{ble}  |  {home}  |  {armed}')
        sequence = d.get('center_sequence', '')
        path = d.get('calibration_file', '')
        self.calibration.set(
            f'iPad 标定：{Path(path).name} · 中心左{sequence.count("a")} / 上{sequence.count("w")}步'
            f'  |  {path}')

    def command(self, action):
        self.update()
        if self.send and (action == 'stop' or str(self.arm['state']) == 'normal'):
            self.send(action)
