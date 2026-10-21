#!/usr/bin/env python3
"""ROS 2 keyboard and serial controller for the ESP32 BLE HID mouse."""

import json
import math
import os
import signal
import time
import tkinter as tk
from collections import Counter

import rclpy
import serial
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from whill_controller.serial_connection import open_serial

from whill_controller.motion_model import (
    BACKWARD_SPEED_MPS,
    FORWARD_SPEED_MPS,
    LEFT_TURN_RATE_RADPS,
    RIGHT_TURN_RATE_RADPS,
    DeadReckoningModel,
)


COMBO_WINDOW_MS = 35
HOLD_DELAY_MS = 250
REPEAT_INTERVAL_MS = 20
ROS_PUMP_INTERVAL_MS = 20
LOCKED_HOME_WAIT_MS = 900
BLE_QUERY_INTERVAL_MS = 1000
IPAD_SCREEN_STEP_COORDINATES = (45, 30)

STATUS_TOPIC = '/whill/controller/status'
EVENT_TOPIC = '/whill/controller/event'
COMMAND_TOPIC = '/whill/controller/command'
INPUT_TOPIC = '/whill/controller/input'
MOTION_TOPIC = '/whill/controller/motion'
ODOMETRY_TOPIC = '/whill/controller/odometry'
PATH_TOPIC = '/whill/controller/path'

CARDINAL_KEYS = {'w', 'a', 's', 'd'}
WHILL_DIAGONAL_KEYS = {'z', 'x', 'c', 'v'}
KEY_VECTORS = {
    'w': (0, -1),
    's': (0, 1),
    'a': (-1, 0),
    'd': (1, 0),
    'z': (-1, -1),
    'x': (-1, 1),
    'c': (1, -1),
    'v': (1, 1),
}
COMMANDS = {
    (0, -1): b'w',
    (0, 1): b's',
    (-1, 0): b'a',
    (1, 0): b'd',
    (-1, -1): b'q',
    (1, -1): b'e',
    (-1, 1): b'z',
    (1, 1): b'c',
}
DIRECTION_NAMES = {
    (0, 0): 'idle',
    (0, -1): 'up',
    (0, 1): 'down',
    (-1, 0): 'left',
    (1, 0): 'right',
    (-1, -1): 'up-left',
    (1, -1): 'up-right',
    (-1, 1): 'down-left',
    (1, 1): 'down-right',
}
LOCKED_TARGET_NAMES = {
    (0, 0): 'steering wheel center',
    (0, -1): 'forward',
    (1, -1): 'right-forward',
    (1, 0): 'right',
    (1, 1): 'right-backward',
    (0, 1): 'backward',
    (-1, 1): 'left-backward',
    (-1, 0): 'left',
    (-1, -1): 'left-forward',
}


class WhillControllerNode(Node):
    """Run the keyboard GUI and bridge key state to the ESP32 serial port."""

    def __init__(self):
        super().__init__('whill_controller')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('control_mode', 'free_screen')
        self.declare_parameter('whill_mode', False)
        self.declare_parameter('hardware_enabled', True)
        self.declare_parameter('forward_speed_mps', FORWARD_SPEED_MPS)
        self.declare_parameter('backward_speed_mps', BACKWARD_SPEED_MPS)
        self.declare_parameter('left_turn_rate_radps', LEFT_TURN_RATE_RADPS)
        self.declare_parameter('right_turn_rate_radps', RIGHT_TURN_RATE_RADPS)
        self.declare_parameter('odometry_requires_button_hold', True)
        self.declare_parameter('path_max_points', 3000)
        self.declare_parameter(
            'calibration_file',
            '',
        )

        self.port = str(self.get_parameter('serial_port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.hardware_enabled = bool(
            self.get_parameter('hardware_enabled').value)
        self.odometry_requires_button_hold = bool(
            self.get_parameter('odometry_requires_button_hold').value)
        self.path_max_points = max(
            100, int(self.get_parameter('path_max_points').value))
        self.motion_model = DeadReckoningModel(
            forward_speed=float(
                self.get_parameter('forward_speed_mps').value),
            backward_speed=float(
                self.get_parameter('backward_speed_mps').value),
            left_turn_rate=float(
                self.get_parameter('left_turn_rate_radps').value),
            right_turn_rate=float(
                self.get_parameter('right_turn_rate_radps').value),
        )
        configured_mode = str(self.get_parameter('control_mode').value)
        self.whill_mode = bool(self.get_parameter('whill_mode').value)
        self.whill_mode = self.whill_mode or configured_mode in {
            'whill_app', 'app_locked'}
        self.control_mode = (
            'app_locked' if self.whill_mode else 'free_screen')
        self.calibration_file = os.path.expanduser(
            str(self.get_parameter('calibration_file').value))
        if not self.calibration_file:
            self.calibration_file = os.path.join(
                get_package_share_directory('whill_controller'),
                'config',
                'default_calibration.json',
            )
        self.calibration_origin = 'unknown'
        self.calibration_sequences = {}
        self.calibration_coordinates = {}
        self._load_calibration_points()

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        log_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.status_pub = self.create_publisher(
            String, STATUS_TOPIC, status_qos)
        self.event_pub = self.create_publisher(
            String, EVENT_TOPIC, log_qos)
        self.command_pub = self.create_publisher(
            String, COMMAND_TOPIC, log_qos)
        self.motion_pub = self.create_publisher(
            String, MOTION_TOPIC, log_qos)
        self.odometry_pub = self.create_publisher(
            Odometry, ODOMETRY_TOPIC, 20)
        self.path_pub = self.create_publisher(
            Path, PATH_TOPIC, status_qos)
        self.input_sub = self.create_subscription(
            String,
            INPUT_TOPIC,
            self.on_ros_command,
            log_qos,
        )
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.motion_timer = self.create_timer(0.05, self.update_motion)

        self.keys_down = set()
        self.initial_job = None
        self.repeat_job = None
        self.hold_job = None
        self.release_jobs = {}
        self.key_press_started = {}
        self.key_press_counts = Counter()
        self.key_press_counts_by_mode = {
            'free_screen': Counter(),
            'app_locked': Counter(),
        }
        self.key_duration_totals = Counter()
        self.last_key_duration = None
        self.command_sequence = 0
        self.last_command = None
        self.click_held = False
        self.calibrating = False
        self.bluetooth_connected = None
        self.serial_text = ''
        self.pending_locked_initialization = False
        self.ble_query_job = None
        self.closing = False
        self.locked_current_coordinates = None
        self.locked_init_generation = 0
        self.motion_last_time = time.monotonic()
        self.motion_state = self.motion_model.state()
        self.motion_path = [(0.0, 0.0, 0.0)]
        self.motion_tracking_active = False
        self.keyboard_armed = False
        self.display_text = 'Starting controller'
        self.movement_keys = set(CARDINAL_KEYS)
        if self.whill_mode:
            self.movement_keys.update(WHILL_DIAGONAL_KEYS)

        self.serial_device = None
        if self.hardware_enabled:
            self._open_serial()

        self.root = tk.Tk()
        self._configure_window()
        self._publish_path()
        self.root.after(ROS_PUMP_INTERVAL_MS, self._pump_ros)
        if self.whill_mode:
            self.root.after(1200, self.request_locked_initialization)

        state = 'hardware enabled' if self.hardware_enabled else 'dry run'
        self.get_logger().info(
            f'WHILL controller started: mode={self.control_mode}, {state}, '
            f'port={self.port}')
        self.publish_event('controller_started')

    def _open_serial(self):
        self.serial_device = open_serial(self.port, self.baud, 1)

    def _load_calibration_points(self):
        try:
            with open(
                    self.calibration_file, encoding='utf-8') as input_file:
                dataset = json.load(input_file)
        except (OSError, json.JSONDecodeError) as error:
            self.get_logger().error(
                f'Unable to load calibration file '
                f'{self.calibration_file}: {error}')
            return

        self.calibration_origin = str(
            dataset.get('origin', 'top-left screen boundary'))

        for point in dataset.get('points', []):
            name = str(point.get('name', '')).strip()
            sequence = str(point.get('sequence', '')).lower()
            if not name:
                continue
            offset = point.get('offset')
            if isinstance(offset, dict):
                x = int(offset.get('x', 0))
                y = int(offset.get('y', 0))
            else:
                x = sequence.count('d') - sequence.count('a')
                y = sequence.count('s') - sequence.count('w')
            self.calibration_sequences[name] = sequence
            self.calibration_coordinates[name] = (x, y)

        missing = set(LOCKED_TARGET_NAMES.values()) - set(
            self.calibration_coordinates)
        if missing:
            self.get_logger().error(
                f'Calibration file is missing points: {sorted(missing)}')
        else:
            self.get_logger().info(
                f'Loaded {len(self.calibration_coordinates)} '
                f'calibration points from {self.calibration_origin}')

    def _configure_window(self):
        self.root.title('WHILL Control and 2D Trajectory')
        self.root.geometry('1320x820')
        self.root.minsize(1100, 700)
        self.root.configure(bg='#18202a')
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        title_text = 'WHILL APP Locked Control' if self.whill_mode else (
            'ESP32 iPad Free-Screen Control')
        self.title_label = tk.Label(
            self.root,
            text=title_text,
            font=('Sans', 20, 'bold'),
            fg='white',
            bg='#18202a',
        )
        self.title_label.pack(pady=(14, 8))

        content = tk.Frame(self.root, bg='#18202a')
        content.pack(fill='both', expand=True, padx=14, pady=(0, 14))

        trajectory_panel = tk.Frame(content, bg='#111923')
        trajectory_panel.pack(
            side='left', fill='both', expand=True, padx=(0, 12))
        tk.Label(
            trajectory_panel,
            text='Estimated Wheelchair Trajectory',
            font=('Sans', 14, 'bold'),
            fg='#d9e6f2',
            bg='#111923',
        ).pack(pady=(10, 6))
        self.trajectory_canvas = tk.Canvas(
            trajectory_panel,
            bg='#080d13',
            highlightthickness=0,
        )
        self.trajectory_canvas.pack(
            fill='both', expand=True, padx=10, pady=(0, 8))
        self.trajectory_canvas.bind(
            '<Configure>', lambda _event: self._draw_trajectory())
        trajectory_controls = tk.Frame(trajectory_panel, bg='#111923')
        trajectory_controls.pack(fill='x', padx=10, pady=(0, 10))
        self.arm_button_text = tk.StringVar(
            value='Arm Keyboard Control')
        self.arm_button = tk.Button(
            trajectory_controls,
            textvariable=self.arm_button_text,
            command=self.toggle_keyboard_arm,
            font=('Sans', 10, 'bold'),
            bg='#8b2f3c',
            fg='white',
            activebackground='#a63b4a',
            activeforeground='white',
            padx=14,
            pady=6,
        )
        self.arm_button.pack(side='left', padx=(0, 8))
        tk.Button(
            trajectory_controls,
            text='Reset Trajectory Origin',
            command=self.reset_trajectory,
            font=('Sans', 10, 'bold'),
            padx=14,
            pady=6,
        ).pack(side='left')
        tk.Label(
            trajectory_controls,
            text='Origin: startup pose (0, 0, 0°) · +X forward · +Y left',
            font=('Sans', 10),
            fg='#9fb3c8',
            bg='#111923',
        ).pack(side='right')

        information_panel = tk.Frame(content, bg='#18202a', width=410)
        information_panel.pack(side='right', fill='y')
        information_panel.pack_propagate(False)

        self.mode_text = tk.StringVar()
        self.connection_text = tk.StringVar()
        self.pose_text = tk.StringVar()
        self.velocity_text = tk.StringVar()
        self.tracking_text = tk.StringVar()
        self.counts_text = tk.StringVar()
        self.last_command_text = tk.StringVar()

        def add_information_row(title, variable, color='#d9e6f2'):
            frame = tk.Frame(information_panel, bg='#111923')
            frame.pack(fill='x', pady=(0, 8))
            tk.Label(
                frame,
                text=title,
                anchor='w',
                font=('Sans', 10, 'bold'),
                fg='#79b8ff',
                bg='#111923',
            ).pack(fill='x', padx=12, pady=(8, 2))
            tk.Label(
                frame,
                textvariable=variable,
                anchor='w',
                justify='left',
                wraplength=375,
                font=('Monospace', 10, 'bold'),
                fg=color,
                bg='#111923',
            ).pack(fill='x', padx=12, pady=(0, 8))

        add_information_row('MODE AND TRACKING', self.mode_text)
        add_information_row('CONNECTION', self.connection_text, '#62d6a7')
        add_information_row('ESTIMATED POSE', self.pose_text, '#ffffff')
        add_information_row('CALIBRATED VELOCITY', self.velocity_text)
        add_information_row('LIVE MOTION STATE', self.tracking_text, '#f5c96b')
        add_information_row('KEY PRESS TOTALS', self.counts_text)
        add_information_row('LAST ESP32 COMMAND', self.last_command_text)

        self.instructions_label = tk.Label(
            information_panel,
            text=self._instructions(),
            justify='left',
            anchor='w',
            wraplength=390,
            font=('Sans', 10),
            fg='#d9e6f2',
            bg='#18202a',
        )
        self.instructions_label.pack(fill='x', pady=(2, 8))

        initial_state = (
            f'Connected to {self.port} · pointer idle'
            if self.hardware_enabled
            else 'Dry-run mode · hardware disabled')
        self.status = tk.StringVar(value=initial_state)
        self.display_text = initial_state
        tk.Label(
            information_panel,
            textvariable=self.status,
            justify='left',
            anchor='w',
            wraplength=390,
            font=('Sans', 10, 'bold'),
            fg='#62d6a7',
            bg='#18202a',
        ).pack(fill='x', pady=(0, 8))

        self.timing = tk.StringVar(
            value='Key timing: waiting for input')
        tk.Label(
            information_panel,
            textvariable=self.timing,
            justify='left',
            anchor='w',
            wraplength=390,
            font=('Monospace', 9, 'bold'),
            fg='#f5c96b',
            bg='#18202a',
        ).pack(fill='x', pady=(0, 8))

        self.root.bind_all('<KeyPress>', self.on_key_press)
        self.root.bind_all('<KeyRelease>', self.on_key_release)
        self._refresh_dashboard()

    def _refresh_dashboard(self):
        """Refresh all dashboard text and redraw the trajectory canvas."""
        if not hasattr(self, 'mode_text'):
            return
        state = self.motion_state
        mode_name = (
            'APP LOCKED' if self.control_mode == 'app_locked'
            else 'FREE SCREEN')
        arm_state = 'ARMED' if self.keyboard_armed else 'DISARMED'
        hold_rule = (
            'requires F button hold'
            if self.odometry_requires_button_hold else 'follows direction keys')
        self.mode_text.set(
            f'{mode_name} · keyboard {arm_state}\nOdometry {hold_rule}')

        serial_state = 'connected' if (
            self.serial_device and self.serial_device.is_open) else 'offline'
        if not self.hardware_enabled:
            serial_state = 'dry run'
        ble_state = {
            True: 'connected', False: 'disconnected', None: 'unknown'
        }[self.bluetooth_connected]
        self.connection_text.set(
            f'Serial: {serial_state} ({self.port})\nBluetooth: {ble_state}')
        self.pose_text.set(
            f'X: {state.x:+.3f} m\nY: {state.y:+.3f} m\n'
            f'Yaw: {math.degrees(state.yaw):+.2f}°')
        self.velocity_text.set(
            f'W forward: {self.motion_model.forward_speed:.4f} m/s\n'
            f'S backward: {self.motion_model.backward_speed:.4f} m/s\n'
            f'A left turn: {math.degrees(self.motion_model.left_turn_rate):.3f}°/s\n'
            f'D right turn: {math.degrees(self.motion_model.right_turn_rate):.3f}°/s')
        tracking = 'ACTIVE' if self.motion_tracking_active else 'STANDBY'
        self.tracking_text.set(
            f'{tracking}\nKeys: {sorted(self.keys_down & self.movement_keys)}\n'
            f'Linear: {state.linear_velocity:+.4f} m/s\n'
            f'Angular: {math.degrees(state.angular_velocity):+.3f}°/s')
        free_total = sum(self.key_press_counts_by_mode['free_screen'].values())
        app_total = sum(self.key_press_counts_by_mode['app_locked'].values())
        cardinal_counts = '  '.join(
            f'{key.upper()}={self.key_press_counts[key]}'
            for key in ('w', 'a', 's', 'd'))
        self.counts_text.set(
            f'All modes: {sum(self.key_press_counts.values())}\n'
            f'Free screen: {free_total}  APP locked: {app_total}\n'
            f'{cardinal_counts}')
        if self.last_command:
            self.last_command_text.set(
                f'#{self.last_command["sequence"]} '
                f'{self.last_command["action"]}\n'
                f'Payload: {self.last_command["payload_ascii"]!r}')
        else:
            self.last_command_text.set('No command sent')
        self._draw_trajectory()

    def _draw_trajectory(self):
        """Render an auto-scaled XY path and the current wheelchair heading."""
        if not hasattr(self, 'trajectory_canvas'):
            return
        canvas = self.trajectory_canvas
        width = max(canvas.winfo_width(), 640)
        height = max(canvas.winfo_height(), 520)
        canvas.delete('all')

        points = self.motion_path or [(0.0, 0.0, 0.0)]
        extent = max(
            1.0,
            max(max(abs(x), abs(y)) for x, y, _yaw in points) * 1.2,
        )
        margin = 55
        scale = min(
            (width - 2 * margin) / (2 * extent),
            (height - 2 * margin) / (2 * extent),
        )
        center_x = width / 2.0
        center_y = height / 2.0

        def screen_point(x, y):
            return center_x + x * scale, center_y - y * scale

        grid_step = 0.5 if extent <= 3.0 else 1.0
        grid_limit = int(math.ceil(extent / grid_step))
        for index in range(-grid_limit, grid_limit + 1):
            value = index * grid_step
            sx, _ = screen_point(value, 0.0)
            _, sy = screen_point(0.0, value)
            canvas.create_line(
                sx, margin, sx, height - margin, fill='#162332')
            canvas.create_line(
                margin, sy, width - margin, sy, fill='#162332')

        canvas.create_line(
            margin, center_y, width - margin, center_y,
            fill='#496780', width=2, arrow='last')
        canvas.create_line(
            center_x, height - margin, center_x, margin,
            fill='#496780', width=2, arrow='last')
        canvas.create_text(
            width - margin - 10, center_y - 16, text='+X forward',
            fill='#9fb3c8', anchor='e', font=('Sans', 9, 'bold'))
        canvas.create_text(
            center_x + 12, margin + 8, text='+Y left',
            fill='#9fb3c8', anchor='nw', font=('Sans', 9, 'bold'))

        flattened = []
        for x, y, _yaw in points:
            flattened.extend(screen_point(x, y))
        if len(flattened) >= 4:
            canvas.create_line(
                *flattened, fill='#37d5ff', width=3)

        origin_x, origin_y = screen_point(0.0, 0.0)
        canvas.create_oval(
            origin_x - 6, origin_y - 6, origin_x + 6, origin_y + 6,
            fill='#62d6a7', outline='')
        state = self.motion_state
        vehicle_x, vehicle_y = screen_point(state.x, state.y)
        heading_length = 32
        heading_x = vehicle_x + heading_length * math.cos(state.yaw)
        heading_y = vehicle_y - heading_length * math.sin(state.yaw)
        canvas.create_oval(
            vehicle_x - 8, vehicle_y - 8, vehicle_x + 8, vehicle_y + 8,
            fill='#ffb347', outline='#ffffff', width=2)
        canvas.create_line(
            vehicle_x, vehicle_y, heading_x, heading_y,
            fill='#ff6b6b', width=4, arrow='last')
        canvas.create_text(
            margin, 20,
            text=f'Auto scale: ±{extent:.1f} m · path points: {len(points)}',
            fill='#9fb3c8', anchor='w', font=('Monospace', 9))

    def _instructions(self):
        if self.whill_mode:
            return (
                'APP LOCKED MODE\n'
                'W/S: forward/backward · A/D: left/right turn\n'
                'Z/X/C/V: combined diagonal targets\n'
                'F: start/stop sustained control\n'
                'R: re-center pointer · O: free-screen mode\n'
                'Click this window before using control keys.\n'
                'Trajectory updates only during active sustained control.')
        return (
            'FREE SCREEN MODE\n'
            'W/A/S/D: move the iPad pointer\n'
            'Enter: left click · F: toggle button hold\n'
            'O: APP locked mode · Esc: close safely\n'
            'Click this window before using control keys.\n'
            'Wheelchair trajectory remains stationary in this mode.')

    def _pump_ros(self):
        if self.closing:
            return
        if not rclpy.ok():
            self.close()
            return
        self._drain_serial_output()
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(ROS_PUMP_INTERVAL_MS, self._pump_ros)

    def _motion_is_enabled(self):
        """Return whether keyboard input should update wheelchair odometry."""
        if self.control_mode != 'app_locked' or self.calibrating:
            return False
        if self.odometry_requires_button_hold and not self.click_held:
            return False
        if self.hardware_enabled and self.bluetooth_connected is not True:
            return False
        return True

    @staticmethod
    def _yaw_quaternion(yaw):
        """Return a planar yaw quaternion as (x, y, z, w)."""
        return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def update_motion(self):
        """Integrate active WASD commands and publish motion telemetry."""
        if self.closing:
            return
        now = time.monotonic()
        duration = max(0.0, min(now - self.motion_last_time, 0.25))
        self.motion_last_time = now
        active_keys = self.keys_down & self.movement_keys
        enabled = self._motion_is_enabled()
        self.motion_state = self.motion_model.integrate(
            duration, active_keys, enabled=enabled)
        self.motion_tracking_active = bool(enabled and active_keys)

        state = self.motion_state
        last_point = self.motion_path[-1]
        moved = math.hypot(state.x - last_point[0], state.y - last_point[1])
        if moved >= 0.002 or abs(state.yaw - last_point[2]) >= 0.01:
            self.motion_path.append((state.x, state.y, state.yaw))
            if len(self.motion_path) > self.path_max_points:
                self.motion_path = self.motion_path[-self.path_max_points:]
            self._publish_path()
        self._publish_motion_messages()
        self._refresh_dashboard()

    def _publish_motion_messages(self):
        """Publish JSON motion telemetry and standard ROS navigation messages."""
        state = self.motion_state
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = self._yaw_quaternion(state.yaw)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'whill_origin'
        odometry.child_frame_id = 'whill_base'
        odometry.pose.pose.position.x = state.x
        odometry.pose.pose.position.y = state.y
        odometry.pose.pose.orientation.x = qx
        odometry.pose.pose.orientation.y = qy
        odometry.pose.pose.orientation.z = qz
        odometry.pose.pose.orientation.w = qw
        odometry.twist.twist.linear.x = state.linear_velocity
        odometry.twist.twist.angular.z = state.angular_velocity
        self.odometry_pub.publish(odometry)

        motion = String()
        motion.data = json.dumps({
            'time_ns': self.get_clock().now().nanoseconds,
            'control_mode': self.control_mode,
            'keyboard_armed': self.keyboard_armed,
            'tracking_active': self.motion_tracking_active,
            'x_m': round(state.x, 6),
            'y_m': round(state.y, 6),
            'yaw_rad': round(state.yaw, 6),
            'yaw_deg': round(math.degrees(state.yaw), 3),
            'linear_velocity_mps': round(state.linear_velocity, 6),
            'angular_velocity_radps': round(state.angular_velocity, 6),
            'active_keys': sorted(self.keys_down & self.movement_keys),
            'left_button_held': self.click_held,
            'odometry_requires_button_hold': (
                self.odometry_requires_button_hold),
        }, ensure_ascii=False, separators=(',', ':'))
        self.motion_pub.publish(motion)

    def _publish_path(self):
        """Publish the accumulated trajectory as a nav_msgs/Path message."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'whill_origin'
        for x, y, yaw in self.motion_path:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            qx, qy, qz, qw = self._yaw_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)
        self.path_pub.publish(path)

    def reset_trajectory(self):
        """Reset the dead-reckoning origin and start a new displayed path."""
        self.motion_model.reset()
        self.motion_state = self.motion_model.state()
        self.motion_last_time = time.monotonic()
        self.motion_path = [(0.0, 0.0, 0.0)]
        self._publish_path()
        self.publish_event('trajectory_reset', origin=[0.0, 0.0, 0.0])
        self._set_display('Trajectory origin reset to x=0.000 m, y=0.000 m')

    def toggle_keyboard_arm(self):
        """Arm keyboard control or return immediately to a safe idle state."""
        self.keyboard_armed = not self.keyboard_armed
        if self.keyboard_armed:
            self.arm_button_text.set('Disarm Keyboard Control')
            self.arm_button.configure(bg='#1f7a4d', activebackground='#299c64')
            self._set_display(
                'Keyboard control ARMED · W/A/S/D commands are active')
            self.publish_event('keyboard_control_armed', armed=True)
            self.root.focus_set()
            self._refresh_dashboard()
            return

        for job_name in ('initial_job', 'repeat_job', 'hold_job'):
            job = getattr(self, job_name)
            if job is not None:
                self.root.after_cancel(job)
                setattr(self, job_name, None)
        for key in list(self.key_press_started):
            self.finish_key_timing(key)
        self.keys_down.clear()
        if self.click_held:
            self._write_command(b']', action='keyboard_disarm_button_release')
            self.click_held = False
        self.arm_button_text.set('Arm Keyboard Control')
        self.arm_button.configure(bg='#8b2f3c', activebackground='#a63b4a')
        self._set_display(
            'Keyboard control DISARMED · movement and button hold stopped')
        self.publish_event('keyboard_control_armed', armed=False)
        self._refresh_dashboard()

    def _drain_serial_output(self):
        if not self.hardware_enabled or self.serial_device is None:
            return
        try:
            waiting = self.serial_device.in_waiting
            if waiting:
                chunk = self.serial_device.read(waiting).decode(
                    'utf-8', errors='ignore')
                self.serial_text += chunk
                while '\n' in self.serial_text:
                    line, self.serial_text = self.serial_text.split('\n', 1)
                    self._handle_serial_line(line.strip())
        except serial.SerialException as error:
            self._set_display(f'Serial read error: {error}')
            self.get_logger().error(str(error))

    def _handle_serial_line(self, line):
        """Handle BLE state feedback from the ESP32 serial console."""
        if not line:
            return
        disconnected = (
            'BLE_CONNECTED=0' in line or 'DISCONNECT' in line)
        connected = (
            'BLE_CONNECTED=1' in line or
            ('CONNECT' in line and not disconnected))
        if disconnected:
            self.bluetooth_connected = False
            self.publish_event(
                'bluetooth_state', connected=False, serial_line=line)
            if self.control_mode == 'app_locked':
                self.pending_locked_initialization = True
                self.calibrating = True
                self.locked_current_coordinates = None
                self._set_display(
                    'APP LOCKED mode · waiting for nimble Bluetooth')
                self._schedule_ble_query()
            return
        if connected:
            was_connected = self.bluetooth_connected
            self.bluetooth_connected = True
            self.publish_event(
                'bluetooth_state', connected=True, serial_line=line)
            if self.ble_query_job is not None:
                self.root.after_cancel(self.ble_query_job)
                self.ble_query_job = None
            if (self.pending_locked_initialization and
                    self.control_mode == 'app_locked'):
                self.pending_locked_initialization = False
                delay = 250 if was_connected is not True else 0
                self.root.after(delay, self.initialize_locked_mode)

    def request_locked_initialization(self):
        """Home only after the ESP32 confirms an active BLE HID link."""
        if self.control_mode != 'app_locked':
            return
        self.locked_init_generation += 1
        self.pending_locked_initialization = True
        self.calibrating = True
        self.locked_current_coordinates = None
        if not self.hardware_enabled:
            self.pending_locked_initialization = False
            self.initialize_locked_mode()
            return
        self._set_display(
            'APP LOCKED mode · checking nimble Bluetooth connection')
        self._write_command(b'?', action='bluetooth_state_query')
        self.publish_event('locked_mode_waiting_for_bluetooth')
        self._schedule_ble_query()

    def _schedule_ble_query(self):
        if self.ble_query_job is not None:
            self.root.after_cancel(self.ble_query_job)
        self.ble_query_job = self.root.after(
            BLE_QUERY_INTERVAL_MS, self._retry_ble_query)

    def _retry_ble_query(self):
        self.ble_query_job = None
        if (self.closing or self.control_mode != 'app_locked' or
                not self.pending_locked_initialization):
            return
        self._write_command(b'?', action='bluetooth_state_query')
        self._schedule_ble_query()

    def _write_command(self, command, action='serial_write', **details):
        """Write bytes to the ESP32 and publish an auditable command record."""
        self.command_sequence += 1
        command_record = {
            'sequence': self.command_sequence,
            'time_ns': self.get_clock().now().nanoseconds,
            'control_mode': self.control_mode,
            'action': action,
            'payload_ascii': command.decode('ascii', errors='backslashreplace'),
            'payload_hex': command.hex(),
            'byte_count': len(command),
            'hardware_enabled': self.hardware_enabled,
        }
        command_record.update(details)
        self.last_command = command_record
        message = String()
        message.data = json.dumps(
            command_record, ensure_ascii=False, separators=(',', ':'))
        self.command_pub.publish(message)
        if not self.hardware_enabled:
            return True
        try:
            self.serial_device.write(command)
            return True
        except serial.SerialException as error:
            self._set_display(f'Serial error: {error}')
            self.get_logger().error(str(error))
            self.publish_event(
                'serial_write_failed', sequence=self.command_sequence,
                error=str(error))
            return False

    def _set_display(self, text):
        self.display_text = text
        self.status.set(text)

    def vector(self):
        dx = sum(
            KEY_VECTORS[key][0]
            for key in self.keys_down if key in self.movement_keys)
        dy = sum(
            KEY_VECTORS[key][1]
            for key in self.keys_down if key in self.movement_keys)
        dx = int(dx > 0) - int(dx < 0)
        dy = int(dy > 0) - int(dy < 0)
        return dx, dy

    def direction_name(self):
        return DIRECTION_NAMES.get(self.vector(), 'unknown')

    def start_key_timing(self, key):
        """Start timing one logical key press."""
        if key in self.key_press_started:
            return
        self.key_press_started[key] = time.monotonic()
        self.key_press_counts[key] += 1
        self.key_press_counts_by_mode[self.control_mode][key] += 1

    def finish_key_timing(self, key):
        """Publish and display the duration of one logical key press."""
        started = self.key_press_started.pop(key, None)
        if started is None:
            return
        duration = time.monotonic() - started
        self.key_duration_totals[key] += duration
        self.last_key_duration = {
            'key': key,
            'duration_sec': round(duration, 3),
            'mode': self.control_mode,
        }
        text = (
            f'Key {key.upper()} held {duration:.3f} s · '
            f'total {self.key_duration_totals[key]:.3f} s · '
            f'key press #{self.key_press_counts[key]} · '
            f'all-mode total #{sum(self.key_press_counts.values())}')
        self.timing.set(text)
        self.get_logger().info(text)
        self.publish_event(
            'key_duration',
            key=key,
            duration_sec=round(duration, 6),
            cumulative_sec=round(self.key_duration_totals[key], 6),
            press_count=self.key_press_counts[key],
        )

    def publish_status(self):
        message = String()
        message.data = json.dumps({
            'node': self.get_name(),
            'state': 'closing' if self.closing else 'running',
            'control_mode': self.control_mode,
            'serial_port': self.port,
            'hardware_enabled': self.hardware_enabled,
            'serial_connected': bool(
                self.serial_device and self.serial_device.is_open),
            'bluetooth_connected': self.bluetooth_connected,
            'direction': self.direction_name(),
            'pointer_coordinate': self.locked_current_coordinates,
            'app_target_offsets': self.calibration_coordinates,
            'keys_down': sorted(self.keys_down),
            'left_button_held': self.click_held,
            'calibrating': self.calibrating,
            'last_key_duration': self.last_key_duration,
            'key_press_counts': dict(self.key_press_counts),
            'total_key_presses': sum(self.key_press_counts.values()),
            'key_press_counts_by_mode': {
                mode: dict(counts)
                for mode, counts in self.key_press_counts_by_mode.items()
            },
            'total_key_presses_by_mode': {
                mode: sum(counts.values())
                for mode, counts in self.key_press_counts_by_mode.items()
            },
            'wheelchair_motion': {
                'tracking_active': self.motion_tracking_active,
                'x_m': round(self.motion_state.x, 6),
                'y_m': round(self.motion_state.y, 6),
                'yaw_rad': round(self.motion_state.yaw, 6),
                'yaw_deg': round(
                    math.degrees(self.motion_state.yaw), 3),
                'linear_velocity_mps': round(
                    self.motion_state.linear_velocity, 6),
                'angular_velocity_radps': round(
                    self.motion_state.angular_velocity, 6),
                'path_point_count': len(self.motion_path),
                'requires_button_hold': (
                    self.odometry_requires_button_hold),
                'calibration': {
                    'forward_speed_mps': (
                        self.motion_model.forward_speed),
                    'backward_speed_mps': (
                        self.motion_model.backward_speed),
                    'left_turn_rate_radps': (
                        self.motion_model.left_turn_rate),
                    'right_turn_rate_radps': (
                        self.motion_model.right_turn_rate),
                },
            },
            'key_duration_totals': {
                key: round(value, 6)
                for key, value in self.key_duration_totals.items()
            },
            'command_sequence': self.command_sequence,
            'last_command': self.last_command,
            'display': self.display_text,
        }, ensure_ascii=False, separators=(',', ':'))
        self.status_pub.publish(message)

    def publish_event(self, event_name, **details):
        message = String()
        payload = {
            'event': event_name,
            'time_ns': self.get_clock().now().nanoseconds,
            'control_mode': self.control_mode,
        }
        payload.update(details)
        message.data = json.dumps(
            payload, ensure_ascii=False, separators=(',', ':'))
        self.event_pub.publish(message)

    def on_ros_command(self, message):
        """
        Accept movement-only commands for ROS integration and testing.

        The public input topic deliberately cannot click or hold the mouse
        button. A command such as ``tap:w`` emits one firmware step, while
        ``mode:toggle`` safely exercises the same mode switch as the O key.
        """
        command = message.data.strip().lower()
        self.publish_event('ros_input_received', command=command)
        if command == 'trajectory:reset':
            self.reset_trajectory()
            return
        if command == 'mode:toggle':
            self.toggle_control_mode()
            return
        if not command.startswith('tap:'):
            self.get_logger().warning(
                f'Ignored unsupported controller command: {command!r}')
            return

        key = command.removeprefix('tap:')
        if key not in self.movement_keys:
            self.get_logger().warning(
                f'Ignored unavailable movement key: {key!r}')
            return

        vector = KEY_VECTORS[key]
        firmware_command = COMMANDS.get(vector)
        if firmware_command is None or not self._write_command(
                firmware_command, action='ros_movement', key=key,
                direction=DIRECTION_NAMES[vector]):
            return

        direction = DIRECTION_NAMES[vector]
        self._set_display(
            f'ROS movement test: {direction} · one 25-unit step')
        self.publish_event(
            'ros_movement_command', key=key, direction=direction)

    def toggle_control_mode(self):
        """Toggle between free-screen and calibrated APP control."""
        for job_name in ('initial_job', 'repeat_job', 'hold_job'):
            job = getattr(self, job_name)
            if job is not None:
                self.root.after_cancel(job)
                setattr(self, job_name, None)
        for movement_key in list(self.keys_down & self.movement_keys):
            self.finish_key_timing(movement_key)
            self.keys_down.discard(movement_key)

        self.locked_init_generation += 1
        self.calibrating = False
        self.pending_locked_initialization = False
        self.locked_current_coordinates = None
        if self.ble_query_job is not None:
            self.root.after_cancel(self.ble_query_job)
            self.ble_query_job = None
        if self.click_held:
            self._write_command(b']', action='mode_toggle_button_release')
            self.click_held = False

        if self.control_mode == 'free_screen':
            self.control_mode = 'app_locked'
            self.whill_mode = True
            self.movement_keys = set(CARDINAL_KEYS | WHILL_DIAGONAL_KEYS)
            self._update_mode_ui()
            self.request_locked_initialization()
        else:
            self.control_mode = 'free_screen'
            self.whill_mode = False
            self.movement_keys = set(CARDINAL_KEYS)
            self._update_mode_ui()
            self._set_display(
                f'FREE SCREEN mode · connected to {self.port}')
            self.publish_event('mode_changed', mode=self.control_mode)

    def _update_mode_ui(self):
        if self.control_mode == 'app_locked':
            self.title_label.configure(text='WHILL APP Locked Control')
        else:
            self.title_label.configure(text='ESP32 iPad Free-Screen Control')
        self.instructions_label.configure(text=self._instructions())
        self._refresh_dashboard()

    def initialize_locked_mode(self):
        """Reach the calibrated center without a top-edge down gesture."""
        center_name = LOCKED_TARGET_NAMES[(0, 0)]
        if center_name not in self.calibration_sequences:
            self._set_display(
                f'Calibration point missing: {center_name}')
            return
        self.locked_init_generation += 1
        generation = self.locked_init_generation
        self.pending_locked_initialization = False
        self.calibrating = True
        self.locked_current_coordinates = None
        self.click_held = False
        center = self.calibration_coordinates[center_name]
        if self.calibration_origin == 'bottom-right screen boundary':
            center_sequence = self.calibration_sequences[center_name]
            key_commands = {
                'w': b'w', 'a': b'a', 's': b's', 'd': b'd',
                'z': b'q', 'x': b'z', 'c': b'e', 'v': b'c',
            }
            approach = b''.join(
                key_commands[key]
                for key in center_sequence if key in key_commands)
        else:
            # Compatibility fallback for the original top-left dataset.
            edge_x, edge_y = IPAD_SCREEN_STEP_COORDINATES
            center_x, center_y = center
            dx = center_x - edge_x
            dy = center_y - edge_y
            approach = (
                (b'd' * max(dx, 0)) +
                (b'a' * max(-dx, 0)) +
                (b's' * max(dy, 0)) +
                (b'w' * max(-dy, 0))
            )
        # Queue release, bottom-right homing, and the up-left approach as one
        # uninterrupted transaction. No downward report follows a top edge.
        if not self._write_command(
                b']]]j' + approach,
                action='locked_mode_initialize',
                home='bottom_right',
                target=center_name,
                approach_a=approach.count(b'a'),
                approach_w=approach.count(b'w')):
            return
        self._set_display(
            'APP LOCKED mode · safely approaching calibrated center')
        self.publish_event(
            'locked_mode_initializing',
            calibration=self.calibration_file,
            safe_origin='bottom-right',
            recorded_origin=self.calibration_origin,
            downward_steps=0,
        )
        settle_ms = max(
            LOCKED_HOME_WAIT_MS,
            12 * 20 + len(approach) * 20 + 250,
        )
        self.root.after(
            settle_ms,
            lambda: self._finish_locked_initialization(generation),
        )

    def _finish_locked_initialization(self, generation):
        if (generation != self.locked_init_generation or
                self.control_mode != 'app_locked'):
            return
        center_name = LOCKED_TARGET_NAMES[(0, 0)]
        self.locked_current_coordinates = (
            self.calibration_coordinates[center_name])
        self.calibrating = False
        self._set_display(
            'APP LOCKED mode ready · pointer at calibrated center')
        self.publish_event('mode_changed', mode=self.control_mode)
        self.publish_event(
            'locked_mode_ready', center=center_name,
            coordinates=self.locked_current_coordinates)

    def move_locked_to_active_target(self):
        """Move to the calibrated point selected by active direction keys."""
        if self.calibrating or self.locked_current_coordinates is None:
            return False
        vector = self.vector()
        target_name = LOCKED_TARGET_NAMES.get(vector)
        if target_name not in self.calibration_coordinates:
            return False
        target = self.calibration_coordinates[target_name]
        current_x, current_y = self.locked_current_coordinates
        target_x, target_y = target
        dx = target_x - current_x
        dy = target_y - current_y
        payload = (
            (b'd' * max(dx, 0)) +
            (b'a' * max(-dx, 0)) +
            (b's' * max(dy, 0)) +
            (b'w' * max(-dy, 0))
        )
        if payload and not self._write_command(
                payload, action='app_target_move', target=target_name,
                direction=DIRECTION_NAMES[vector], dx=dx, dy=dy):
            return False
        self.locked_current_coordinates = target
        held = ' · LEFT BUTTON HELD' if self.click_held else ''
        self._set_display(
            f'APP target: {target_name} · coordinate={target}{held}')
        self.publish_event(
            'app_target_changed',
            target=target_name,
            coordinates=target,
            direction=DIRECTION_NAMES[vector],
            left_button_held=self.click_held,
        )
        return True

    def send_current_direction(self):
        vector = self.vector()
        command = COMMANDS.get(vector)
        if command is None:
            self._set_display(
                f'Connected to {self.port} · directions cancel')
            return False
        if not self._write_command(
                command, action='free_screen_move',
                direction=DIRECTION_NAMES[vector]):
            return False

        held = ' · LEFT BUTTON HELD' if self.click_held else ''
        self._set_display(
            f'Moving {DIRECTION_NAMES[vector]} · '
            f'{sorted(self.keys_down)}{held}')
        return True

    def emit_initial(self):
        self.initial_job = None
        if self.keys_down & self.movement_keys:
            self.send_current_direction()

    def begin_repeat(self):
        self.hold_job = None
        if self.keys_down & self.movement_keys:
            self.repeat_once()

    def repeat_once(self):
        self.repeat_job = None
        if not (self.keys_down & self.movement_keys):
            return
        self.send_current_direction()
        self.repeat_job = self.root.after(
            REPEAT_INTERVAL_MS, self.repeat_once)

    def on_key_press(self, event):
        key = event.keysym.lower()
        pending_release = self.release_jobs.pop(key, None)
        if pending_release is not None:
            self.root.after_cancel(pending_release)
            return 'break'

        if key == 'escape':
            self.close()
            return 'break'
        if not self.keyboard_armed:
            return 'break'
        if key == 'o':
            if key not in self.keys_down:
                self.start_key_timing(key)
                self.toggle_control_mode()
                self.publish_event('key_press', key=key)
            self.keys_down.add(key)
            return 'break'
        if self.calibrating:
            return 'break'
        if self.control_mode == 'app_locked' and key == 'r':
            if key not in self.keys_down:
                self.start_key_timing(key)
                self.request_locked_initialization()
                self.publish_event('key_press', key=key)
            self.keys_down.add(key)
            return 'break'
        if key == 'f':
            if key not in self.keys_down:
                self.start_key_timing(key)
                self.click_held = not self.click_held
                command = b'[' if self.click_held else b']'
                self._write_command(
                    command, action='left_button_hold_toggle',
                    held=self.click_held)
                state = (
                    'HELD — movement will drag'
                    if self.click_held else 'RELEASED')
                self._set_display(f'Left button {state}')
                self.publish_event(
                    'left_button', held=self.click_held)
            self.keys_down.add(key)
            return 'break'
        if key in {'return', 'kp_enter'}:
            if key not in self.keys_down:
                self.start_key_timing(key)
                if self.click_held:
                    self._set_display(
                        'Left button is already HELD · press F to release')
                else:
                    self._write_command(b'\r', action='left_click')
                    self._set_display('Left click')
                    self.publish_event('left_click')
            self.keys_down.add(key)
            return 'break'
        if key not in self.movement_keys or key in self.keys_down:
            return 'break'

        was_idle = not (self.keys_down & self.movement_keys)
        self.keys_down.add(key)
        self.start_key_timing(key)
        self.publish_event('key_press', key=key)
        if self.control_mode == 'app_locked':
            self.move_locked_to_active_target()
            return 'break'
        if was_idle:
            self.initial_job = self.root.after(
                COMBO_WINDOW_MS, self.emit_initial)
            self.hold_job = self.root.after(
                HOLD_DELAY_MS, self.begin_repeat)
        return 'break'

    def on_key_release(self, event):
        key = event.keysym.lower()
        if not self.keyboard_armed:
            self.keys_down.discard(key)
            return 'break'
        control_keys = {'return', 'kp_enter', 'f', 'r', 'o'}
        if key not in self.movement_keys and key not in control_keys:
            return 'break'

        old_job = self.release_jobs.pop(key, None)
        if old_job is not None:
            self.root.after_cancel(old_job)
        self.release_jobs[key] = self.root.after(
            12,
            lambda released_key=key: self.finish_key_release(released_key),
        )
        return 'break'

    def finish_key_release(self, key):
        self.release_jobs.pop(key, None)
        self.finish_key_timing(key)
        if key in {'return', 'kp_enter', 'f', 'r', 'o'}:
            self.keys_down.discard(key)
            return

        if self.control_mode == 'app_locked':
            self.keys_down.discard(key)
            self.publish_event('key_release', key=key)
            self.move_locked_to_active_target()
            return

        movement_was_pending = self.initial_job is not None
        if movement_was_pending:
            self.send_current_direction()
            self.root.after_cancel(self.initial_job)
            self.initial_job = None

        self.keys_down.discard(key)
        self.publish_event('key_release', key=key)
        if not (self.keys_down & self.movement_keys):
            for job_name in ('repeat_job', 'hold_job'):
                job = getattr(self, job_name)
                if job is not None:
                    self.root.after_cancel(job)
                    setattr(self, job_name, None)
            held = ' · LEFT BUTTON HELD' if self.click_held else ''
            self._set_display(
                f'Connected to {self.port} · pointer idle{held}')

    def close(self):
        if self.closing:
            return
        self.closing = True
        if self.ble_query_job is not None:
            self.root.after_cancel(self.ble_query_job)
            self.ble_query_job = None
        for key in list(self.key_press_started):
            self.finish_key_timing(key)
        self.publish_event('controller_stopping')
        try:
            if self.click_held:
                self._write_command(b']', action='shutdown_button_release')
                self.click_held = False
            if self.serial_device and self.serial_device.is_open:
                self.serial_device.close()
        finally:
            self.root.quit()
            self.root.destroy()

    def run(self):
        """Run the Tk event loop; ROS callbacks are pumped from Tk."""
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WhillControllerNode()
        signal.signal(
            signal.SIGINT,
            lambda _signum, _frame: node.root.after_idle(node.close),
        )
        signal.signal(
            signal.SIGTERM,
            lambda _signum, _frame: node.root.after_idle(node.close),
        )
        node.run()
    except (serial.SerialException, tk.TclError) as error:
        if node is not None:
            node.get_logger().error(f'Unable to start controller: {error}')
        else:
            print(f'Unable to start controller: {error}')
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
