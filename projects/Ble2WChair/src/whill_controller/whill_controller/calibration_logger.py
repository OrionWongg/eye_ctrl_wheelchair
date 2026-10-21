#!/usr/bin/env python3
"""Interactive ROS 2 bottom-right-origin calibration logger."""

import json
import os
import signal
import tkinter as tk
from collections import Counter
from datetime import datetime

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

from whill_controller.serial_connection import open_serial


ROS_PUMP_INTERVAL_MS = 20
USER_TO_FIRMWARE = {
    'w': b'w',
    'a': b'a',
    's': b's',
    'd': b'd',
}
DIRECTION_NAMES = {
    'w': 'up',
    'a': 'left',
    's': 'down',
    'd': 'right',
}
CALIBRATION_ORIGIN = 'bottom-right screen boundary'
EXPECTED_POINT_NAMES = [
    'steering wheel center',
    'forward',
    'right-forward',
    'right',
    'right-backward',
    'backward',
    'left-backward',
    'left',
    'left-forward',
]


class CalibrationLoggerNode(Node):
    """Collect named pointer targets relative to the bottom-right origin."""

    def __init__(self):
        super().__init__('whill_calibration_logger')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('homing_wait_ms', 1200)
        self.declare_parameter(
            'log_directory',
            '~/.ros/whill_controller/calibration_logs',
        )

        self.port = str(self.get_parameter('serial_port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.homing_wait_ms = int(
            self.get_parameter('homing_wait_ms').value)
        self.log_directory = os.path.expanduser(
            str(self.get_parameter('log_directory').value))

        self.status_pub = self.create_publisher(
            String, '/whill/calibration/status', 10)
        self.event_pub = self.create_publisher(
            String, '/whill/calibration/event', 10)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.serial_device = self._open_serial()
        self.serial_text = ''
        self.bluetooth_connected = False
        self.state = 'waiting_bluetooth'
        self.closing = False
        self.keys_down = set()
        self.point_index = 1
        self.current_counts = Counter()
        self.current_sequence = []
        self.points = []
        self.naming_dialog = None

        os.makedirs(self.log_directory, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(
            self.log_directory, f'calibration_dataset_{stamp}.jsonl')
        self.summary_path = os.path.join(
            self.log_directory, f'calibration_dataset_{stamp}.json')
        self.latest_path = os.path.join(
            self.log_directory, 'latest_calibration.json')
        self.log_file = open(self.log_path, 'a', encoding='utf-8')

        self.root = tk.Tk()
        self._configure_window()
        self.root.after(ROS_PUMP_INTERVAL_MS, self._pump_ros)
        self.root.after(150, self.root.focus_force)
        self.root.after(400, self.query_bluetooth_state)
        self._record('logger_started')
        self.get_logger().info(
            f'Calibration logger started on {self.port}')

    def _open_serial(self):
        return open_serial(self.port, self.baud, 2)

    def _configure_window(self):
        self.root.title('WHILL ROS2 Multi-Point Calibration')
        self.root.geometry('820x650')
        self.root.configure(bg='#18202a')
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        tk.Label(
            self.root,
            text='WHILL Multi-Point Calibration Collector',
            font=('Sans', 20, 'bold'),
            fg='white',
            bg='#18202a',
        ).pack(pady=(20, 10))
        tk.Label(
            self.root,
            text=(
                '1. Wait for nimble Bluetooth connection and automatic '
                'bottom-right homing.\n'
                '2. Move to the target with W/A/S/D.\n'
                '3. Press Enter, name the target, then continue or finish.\n'
                'R restarts the current point from bottom-right. '
                'Esc exits safely.\n'
                'This collector never sends mouse clicks.'
            ),
            justify='center',
            font=('Sans', 12),
            fg='#d9e6f2',
            bg='#18202a',
        ).pack()

        self.status_text = tk.StringVar(
            value='Waiting for nimble Bluetooth connection')
        self.point_text = tk.StringVar(value='Current point: 1')
        self.counts_text = tk.StringVar(value='Counts: none')
        self.sequence_text = tk.StringVar(value='Sequence: none')
        self.saved_text = tk.StringVar(value='Saved points: none')

        for variable, color, size in (
                (self.status_text, '#62d6a7', 12),
                (self.point_text, '#ffffff', 12),
                (self.counts_text, '#f5c96b', 11),
                (self.sequence_text, '#b9d8ff', 11),
                (self.saved_text, '#c6a8ff', 11)):
            tk.Label(
                self.root,
                textvariable=variable,
                font=('Monospace', size, 'bold'),
                fg=color,
                bg='#18202a',
                wraplength=760,
                justify='left',
            ).pack(pady=8)

        button_frame = tk.Frame(self.root, bg='#18202a')
        button_frame.pack(pady=15)
        tk.Button(
            button_frame,
            text='Check Bluetooth / Start Homing',
            command=self.query_bluetooth_state,
            font=('Sans', 11, 'bold'),
            padx=16,
            pady=8,
        ).pack(side='left', padx=8)
        tk.Button(
            button_frame,
            text='Reset Current Point (R)',
            command=self.restart_current_point,
            font=('Sans', 11, 'bold'),
            padx=16,
            pady=8,
        ).pack(side='left', padx=8)

        self.root.bind_all('<KeyPress>', self.on_key_press)
        self.root.bind_all('<KeyRelease>', self.on_key_release)

    def _pump_ros(self):
        if self.closing:
            return
        if not rclpy.ok():
            self.close()
            return
        self._read_serial_output()
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(ROS_PUMP_INTERVAL_MS, self._pump_ros)

    def _read_serial_output(self):
        try:
            waiting = self.serial_device.in_waiting
            if not waiting:
                return
            chunk = self.serial_device.read(waiting).decode(
                'utf-8', errors='ignore')
            self.serial_text += chunk
            while '\n' in self.serial_text:
                line, self.serial_text = self.serial_text.split('\n', 1)
                self._handle_serial_line(line.strip())
        except serial.SerialException as error:
            self.state = 'serial_error'
            self.status_text.set(f'Serial read error: {error}')
            self.get_logger().error(str(error))

    def _handle_serial_line(self, line):
        if not line:
            return
        if 'BLE_CONNECTED=0' in line or 'DISCONNECT' in line:
            self.bluetooth_connected = False
            self.state = 'waiting_bluetooth'
            self.status_text.set(
                'Bluetooth disconnected · reconnect nimble to continue')
            self._record('bluetooth_disconnected', serial_line=line)
        elif 'BLE_CONNECTED=1' in line or (
                'CONNECT' in line and 'DISCONNECT' not in line):
            was_connected = self.bluetooth_connected
            self.bluetooth_connected = True
            if not was_connected and self.state == 'waiting_bluetooth':
                self._record('bluetooth_connected', serial_line=line)
                self.root.after(250, self.start_homing)
        elif 'pointer saturated at bottom-right safe origin' in line:
            if self.state == 'homing':
                self.finish_homing()

    def _write(self, command):
        try:
            self.serial_device.write(command)
            self.serial_device.flush()
            return True
        except serial.SerialException as error:
            self.state = 'serial_error'
            self.status_text.set(f'Serial write error: {error}')
            self.get_logger().error(str(error))
            return False

    def query_bluetooth_state(self):
        if self.state in {'homing', 'naming', 'finished'}:
            return
        self.status_text.set('Checking nimble Bluetooth connection...')
        if not self._write(b'?'):
            return
        self.root.after(1500, self._connection_query_timeout)

    def _connection_query_timeout(self):
        if self.state == 'waiting_bluetooth':
            self.status_text.set(
                'Waiting for nimble Bluetooth connection on the iPad')

    def start_homing(self):
        if not self.bluetooth_connected or self.state == 'homing':
            return
        self.state = 'homing'
        self.keys_down.clear()
        self.current_counts.clear()
        self.current_sequence.clear()
        self._refresh_current_display()
        self.status_text.set(
            'Bluetooth connected · moving pointer to bottom-right origin...')
        self._record('homing_started', point_index=self.point_index)
        if not self._write(b']]]j'):
            return
        self.root.after(self.homing_wait_ms, self._homing_timeout)

    def _homing_timeout(self):
        if self.state == 'homing':
            self.finish_homing()

    def finish_homing(self):
        if self.state != 'homing':
            return
        self.state = 'collecting'
        self.status_text.set(
            'Bottom-right origin ready · NOW enter movement keys, then Enter')
        self._record('homing_finished', point_index=self.point_index)
        self.root.focus_force()

    def restart_current_point(self):
        if not self.bluetooth_connected:
            self.query_bluetooth_state()
            return
        self._record(
            'current_point_discarded',
            point_index=self.point_index,
            sequence=''.join(self.current_sequence),
            counts=dict(self.current_counts),
        )
        self.start_homing()

    def _refresh_current_display(self):
        self.point_text.set(f'Current point: {self.point_index}')
        if self.current_counts:
            counts = ' '.join(
                f'{key.upper()}={count}'
                for key, count in sorted(self.current_counts.items()))
        else:
            counts = 'none'
        self.counts_text.set(f'Counts: {counts}')
        recent = ''.join(self.current_sequence[-100:]).upper() or 'none'
        self.sequence_text.set(f'Sequence (latest 100): {recent}')

    def on_key_press(self, event):
        key = event.keysym.lower()
        if key == 'escape':
            self.close()
            return 'break'
        if key in self.keys_down:
            return 'break'
        self.keys_down.add(key)

        if self.state == 'naming':
            return None
        if key == 'r':
            self.restart_current_point()
            return 'break'
        if key in {'return', 'kp_enter'}:
            if self.state == 'collecting':
                self.open_naming_dialog()
            return 'break'
        if self.state != 'collecting' or key not in USER_TO_FIRMWARE:
            return 'break'
        if not self._write(USER_TO_FIRMWARE[key]):
            return 'break'

        self.current_counts[key] += 1
        self.current_sequence.append(key)
        self._refresh_current_display()
        self.status_text.set(
            f'Recorded {key.upper()} ({DIRECTION_NAMES[key]}) · '
            f'total {len(self.current_sequence)}')
        self._record(
            'movement',
            point_index=self.point_index,
            key=key,
            firmware_command=USER_TO_FIRMWARE[key].decode('ascii'),
            direction=DIRECTION_NAMES[key],
            key_count=self.current_counts[key],
            total_count=len(self.current_sequence),
        )
        return 'break'

    def on_key_release(self, event):
        self.keys_down.discard(event.keysym.lower())
        if self.state == 'naming':
            return None
        return 'break'

    def open_naming_dialog(self):
        if self.naming_dialog is not None:
            return
        self.state = 'naming'
        dialog = tk.Toplevel(self.root)
        self.naming_dialog = dialog
        dialog.title(f'Name calibration point {self.point_index}')
        dialog.geometry('560x260')
        dialog.configure(bg='#202b38')
        dialog.transient(self.root)
        dialog.grab_set()
        dialog.protocol('WM_DELETE_WINDOW', self.cancel_naming)

        tk.Label(
            dialog,
            text=f'Pointer reached point {self.point_index}',
            font=('Sans', 16, 'bold'),
            fg='white',
            bg='#202b38',
        ).pack(pady=(22, 8))
        tk.Label(
            dialog,
            text='Enter a name for this position:',
            font=('Sans', 11),
            fg='#d9e6f2',
            bg='#202b38',
        ).pack()
        suggested_name = (
            EXPECTED_POINT_NAMES[self.point_index - 1]
            if self.point_index <= len(EXPECTED_POINT_NAMES) else '')
        name_var = tk.StringVar(value=suggested_name)
        entry = tk.Entry(
            dialog, textvariable=name_var, font=('Sans', 13), width=36)
        entry.pack(pady=12)
        entry.focus_force()
        entry.select_range(0, 'end')

        buttons = tk.Frame(dialog, bg='#202b38')
        buttons.pack(pady=12)
        tk.Button(
            buttons,
            text='Save and collect next point',
            command=lambda: self.save_named_point(
                name_var.get(), continue_collecting=True),
            font=('Sans', 10, 'bold'),
            padx=12,
            pady=7,
        ).pack(side='left', padx=8)
        tk.Button(
            buttons,
            text='Save and finish',
            command=lambda: self.save_named_point(
                name_var.get(), continue_collecting=False),
            font=('Sans', 10, 'bold'),
            padx=12,
            pady=7,
        ).pack(side='left', padx=8)

    def cancel_naming(self):
        if self.naming_dialog is not None:
            self.naming_dialog.grab_release()
            self.naming_dialog.destroy()
            self.naming_dialog = None
        self.state = 'collecting'
        self.root.focus_force()

    def save_named_point(self, name, continue_collecting):
        name = name.strip()
        if not name:
            self.status_text.set('Point name cannot be empty')
            return
        point = {
            'index': self.point_index,
            'name': name,
            'origin': CALIBRATION_ORIGIN,
            'sequence': ''.join(self.current_sequence),
            'counts': dict(sorted(self.current_counts.items())),
            'total_count': len(self.current_sequence),
            'completed_at': datetime.now().isoformat(timespec='seconds'),
        }
        self.points.append(point)
        self._record('point_saved', point=point)
        self._write_dataset()
        self.saved_text.set(
            'Saved points: ' + ', '.join(
                f'{item["index"]}:{item["name"]}' for item in self.points))

        if self.naming_dialog is not None:
            self.naming_dialog.grab_release()
            self.naming_dialog.destroy()
            self.naming_dialog = None

        if continue_collecting:
            self.point_index += 1
            self.start_homing()
        else:
            self.state = 'finished'
            self.status_text.set(
                f'Calibration saved: {len(self.points)} points · closing')
            self._record('dataset_finished', point_count=len(self.points))
            self._write_dataset()
            self.root.after(900, self.close)

    def _record(self, event, **details):
        record = {
            'time': datetime.now().isoformat(timespec='milliseconds'),
            'time_ns': self.get_clock().now().nanoseconds,
            'event': event,
        }
        record.update(details)
        self.log_file.write(json.dumps(record) + '\n')
        self.log_file.flush()
        message = String()
        message.data = json.dumps(record)
        self.event_pub.publish(message)

    def _write_dataset(self):
        dataset = {
            'created_at': datetime.now().isoformat(timespec='seconds'),
            'serial_port': self.port,
            'origin': CALIBRATION_ORIGIN,
            'point_count': len(self.points),
            'points': self.points,
            'event_log': self.log_path,
        }
        for path in (self.summary_path, self.latest_path):
            with open(path, 'w', encoding='utf-8') as output:
                json.dump(dataset, output, indent=2, ensure_ascii=False)

    def publish_status(self):
        message = String()
        message.data = json.dumps({
            'state': self.state,
            'serial_port': self.port,
            'serial_connected': bool(
                self.serial_device and self.serial_device.is_open),
            'bluetooth_connected': self.bluetooth_connected,
            'point_index': self.point_index,
            'current_sequence': ''.join(self.current_sequence),
            'current_counts': dict(self.current_counts),
            'saved_points': self.points,
            'event_log': self.log_path,
            'dataset_path': self.summary_path,
        })
        self.status_pub.publish(message)

    def close(self):
        if self.closing:
            return
        self.closing = True
        try:
            if self.naming_dialog is not None:
                self.naming_dialog.destroy()
                self.naming_dialog = None
            if self.log_file is not None:
                self._record('logger_stopped', state=self.state)
                self._write_dataset()
                self.log_file.close()
                self.log_file = None
            if self.serial_device and self.serial_device.is_open:
                self.serial_device.close()
        finally:
            self.root.quit()
            self.root.destroy()

    def run(self):
        """Run the calibration GUI and ROS callbacks."""
        self.root.mainloop()


def main(args=None):
    """Start the ROS 2 multi-point calibration collector."""
    rclpy.init(args=args)
    node = None
    try:
        node = CalibrationLoggerNode()
        signal.signal(
            signal.SIGINT,
            lambda _signum, _frame: node.root.after_idle(node.close),
        )
        signal.signal(
            signal.SIGTERM,
            lambda _signum, _frame: node.root.after_idle(node.close),
        )
        node.run()
    except (OSError, serial.SerialException, tk.TclError) as error:
        if node is not None:
            node.get_logger().error(
                f'Unable to start calibration logger: {error}')
        else:
            print(f'Unable to start calibration logger: {error}')
    except KeyboardInterrupt:
        if node is not None:
            node.close()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
