# ESP32 BLE HID Pointer Controller for WHILL on iPad (ROS 2)

**Orin 无硬件启动与验证：见 [OFFLINE.md](OFFLINE.md)。**

This repository contains a ROS 2 Humble workspace that turns keyboard input on
an Ubuntu computer into serial commands for an ESP32-S3 BLE HID mouse. The
ESP32 is paired with an iPad and can operate a pointer in either unrestricted
screen mode or a calibrated, application-specific control region.

The project was developed for accessibility research with the WHILL Model F
application. It includes:

- a large English-language keyboard and 2D trajectory dashboard;
- a ROS 2 serial bridge for the ESP32;
- free-screen and calibrated application modes;
- eight-direction pointer control;
- toggleable left-button hold and single-click commands;
- per-key press duration, cumulative duration, and press-count feedback;
- measured W/S linear speeds and A/D angular rates for dead reckoning;
- standard ROS 2 odometry and path topics for external visualization;
- a visual multi-point calibration collector;
- ROS 2 status, event, and command topics;
- a tested bottom-right calibration dataset.

> [!WARNING]
> This software can interact with an application that controls a powered
> mobility device. Treat it as experimental software, not a certified medical
> or safety controller. Perform initial testing with no occupant, the drive
> wheels safely unloaded or otherwise immobilized, a human operator ready to
> intervene, and the manufacturer's normal emergency-stop procedure available.
> Never use this software as the only safety layer.

## System architecture

```text
Ubuntu keyboard
      |
      v
ROS 2 whill_controller node
      |\
      | \--> 2D dead reckoning -> dashboard / ROS odometry / ROS path
      |
      | USB serial commands (/dev/ttyACM*)
      v
ESP32-S3 custom BLE HID mouse firmware
      |
      | Bluetooth Low Energy HID reports
      v
iPad pointer -> WHILL application interface
```

The iPad does not return an absolute pointer position through the BLE HID mouse
connection. All movement is relative. The calibrated mode therefore starts
from a repeatable screen boundary and replays measured movement sequences.

## Tested hardware and software

| Component | Tested configuration |
| --- | --- |
| ESP32 board | ESP32-S3-DEV-KIT-N#2R16V-M (user-reported N32 configuration) |
| Host computer | x86_64 laptop |
| Host OS | Ubuntu 22.04.5 LTS |
| ROS 2 | Humble |
| Python | 3.10.12 |
| pyserial | 3.5 |
| ESP-IDF used for the companion firmware | v5.3.2 |
| Tablet | iPad mini (6th generation) |
| iPadOS | 18.6.2 during development |
| Target application/device | WHILL application with WHILL Model F |

Other versions may work, but they have not been validated by this project.

## Repository layout

```text
whill_control_ws/
├── README.md
├── LICENSE
└── src/
    └── whill_controller/
        ├── config/
        │   ├── controller.yaml
        │   └── default_calibration.json
        ├── launch/
        │   ├── calibration.launch.py
        │   └── controller.launch.py
        ├── test/
        ├── whill_controller/
        │   ├── calibration_logger.py
        │   ├── controller_node.py
        │   └── motion_model.py
        ├── package.xml
        └── setup.py
```

Generated `build/`, `install/`, `log/`, Python cache, and local calibration-log
directories are intentionally excluded from version control.

## Important firmware prerequisite

This repository contains the ROS 2 side of the project. It does **not** contain
the complete ESP-IDF firmware project or flash the ESP32 automatically.

### Obtain the C firmware reference source

The C source starting point is available from Espressif's official ESP-IDF
repository:

- ESP-IDF repository: <https://github.com/espressif/esp-idf>
- Tested ESP-IDF v5.3.2 tag: <https://github.com/espressif/esp-idf/tree/v5.3.2>
- HID device C example: <https://github.com/espressif/esp-idf/tree/v5.3.2/examples/bluetooth/esp_hid_device>
- Bluedroid BLE HID reference README: <https://github.com/espressif/esp-idf/blob/v5.3.2/examples/bluetooth/bluedroid/ble/ble_hid_device_demo/README.md>

Download the tested source version with:

```bash
git clone --branch v5.3.2 --recursive \
  https://github.com/espressif/esp-idf.git
```

The main reference project is then located at:

```text
esp-idf/examples/bluetooth/esp_hid_device/
```

Copy that example into a separate firmware working directory before modifying
it. Keep the original ESP-IDF checkout clean so changes to the BLE HID report,
serial command parser, reconnection handling, and safe-origin commands remain
easy to review.

The tested firmware was derived from the ESP-IDF v5.3.2 example:

```text
examples/bluetooth/esp_hid_device
```

It was configured as a BLE HID mouse with the Bluetooth device name `nimble`.
The ROS 2 nodes expect the firmware to read single-byte commands from its USB
serial console at 115200 baud and implement the following protocol:

| Serial command | Firmware behavior |
| --- | --- |
| `w` | Move up by one 25-unit step |
| `a` | Move left by one 25-unit step |
| `s` | Move down by one 25-unit step |
| `d` | Move right by one 25-unit step |
| `q` | Move up-left by one 18-unit diagonal step |
| `e` | Move up-right by one 18-unit diagonal step |
| `z` | Move down-left by one 18-unit diagonal step |
| `c` | Move down-right by one 18-unit diagonal step |
| carriage return | Perform one left click |
| `[` | Explicitly hold the left mouse button |
| `]` | Explicitly release the left mouse button |
| `h` | Saturate the pointer at the top-left boundary |
| `j` | Release the button and saturate the pointer at the bottom-right boundary |
| `?` | Print the BLE HID connection state over serial |

The companion firmware must also restart its serial-to-HID input task after a
BLE reconnection. Without that behavior, the iPad can reconnect while the
firmware no longer consumes serial movement commands.

APP-mode homing is connection-gated. The ROS 2 controller sends `?` until the
ESP32 reports `BLE_CONNECTED=1`; only then does it issue the bottom-right
saturation command followed by the locked center approach. If Bluetooth drops
while APP mode is active, the controller invalidates its pointer coordinate
and repeats the same homing sequence automatically after reconnection.

Do not run an ESP-IDF serial monitor, another controller script, and this ROS 2
controller at the same time. Only one process may own the ESP32 serial port.

## Host installation

### 1. Install ROS 2 Humble

Install ROS 2 Humble on Ubuntu 22.04 using the official ROS 2 binary package
instructions. Confirm that the environment is available:

```bash
source /opt/ros/humble/setup.bash
echo "$ROS_DISTRO"
```

The expected output is:

```text
humble
```

### 2. Install workspace dependencies

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-serial \
  python3-tk \
  ros-humble-ament-flake8 \
  ros-humble-ament-pep257 \
  ros-humble-launch \
  ros-humble-launch-ros \
  ros-humble-std-msgs
```

Initialize `rosdep` if it has not already been initialized:

```bash
sudo rosdep init
rosdep update
```

If `rosdep init` reports that the source list already exists, continue with
`rosdep update`.

### 3. Clone the repository

```bash
git clone https://github.com/Yongying-Zhu/WHILL_CONTROLLER.git whill_control_ws
cd whill_control_ws
```

Install package dependencies declared by the workspace:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Configure serial permissions

Connect the ESP32-S3 and inspect the available ports:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

Add the current user to the `dialout` group if required:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and log back in before continuing. Verify access with:

```bash
groups
```

On the tested system, the ESP32-S3 appears as `/dev/ttyACM0` with USB ID
`303a:1001` and the product name `Espressif USB JTAG/serial debug unit`.
The second device, `/dev/ttyACM1`, is a QinHeng `1a86:55d3` USB serial
adapter and must not be selected for ESP32 pointer control. Device numbering
is not guaranteed to remain stable on another computer, so identify the
Espressif device with `lsusb` and the kernel log instead of guessing from the
port number. A persistent `/dev/serial/by-id/...` path is preferable when
available.

## Build and test

From the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run the package tests:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test \
  --packages-select whill_controller
colcon test-result --verbose
```

The current package contains PEP 257 and Flake8 checks. The tested result is
two tests with zero failures.

## Pair the ESP32 with the iPad

1. Flash and reset the compatible ESP32 BLE HID firmware.
2. Open **Settings -> Bluetooth** on the iPad.
3. Select the device named `nimble`.
4. Confirm that the iPad reports it as connected.
5. Move the pointer once in free-screen mode and confirm that it is visible.

If the ESP32 was reflashed, the iPad may need to reconnect. If pairing behaves
unexpectedly, remove the old `nimble` bond from the iPad, reset the ESP32, and
pair it again.

## Run the controller

Source ROS 2 and the workspace in every new terminal:

```bash
cd /path/to/whill_control_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Start with the default serial port and bundled calibration:

```bash
ros2 launch whill_controller controller.launch.py
```

Override the serial device only after confirming that it is the Espressif
ESP32 interface. For example, if the kernel assigns that interface to
`/dev/ttyACM2`:

```bash
ros2 launch whill_controller controller.launch.py \
  serial_port:=/dev/ttyACM2
```

Start directly in calibrated application mode:

```bash
ros2 launch whill_controller controller.launch.py whill_mode:=true
```

Run the GUI and ROS 2 topics without opening hardware:

```bash
ros2 launch whill_controller controller.launch.py hardware_enabled:=false
```

Keep the controller window focused while using keyboard control.

## Keyboard controls

### Controls available in both modes

| Key | Action |
| --- | --- |
| `W` | Up / forward target |
| `A` | Left target |
| `S` | Down / reverse target |
| `D` | Right target |
| `Enter` | One left click when the button is not held |
| `F` | Toggle continuous left-button hold; press again to release |
| `O` | Toggle free-screen and calibrated application modes |
| `Esc` | Safely release the button and close the controller |

### Additional calibrated-mode controls

| Key | Target direction |
| --- | --- |
| `Z` | Up-left |
| `X` | Down-left |
| `C` | Up-right |
| `V` | Down-right |
| `R` | Reinitialize from the calibrated safe boundary and return to center |

In free-screen mode, simultaneous `W/A/S/D` presses produce diagonal motion.
A tap sends one 25-unit step. Holding a key begins repeated movement after a
short delay.

In calibrated application mode, a direction key selects one of eight recorded
target positions. Releasing all direction keys returns the pointer to the
recorded center. Mode changes explicitly release the left button before moving
the pointer.

The locked APP targets use the steering-wheel center as `(x=0, y=0)`, with
positive x pointing right and positive y pointing down:

| Target | Key | Center-relative calibration |
| --- | --- | --- |
| Forward | `W` | 6 up, 1 right; `(1, -6)` |
| Backward | `S` | 6 down; `(0, 6)` |
| Left | `A` | 5 left; `(-5, 0)` |
| Right | `D` | 6 right; `(6, 0)` |
| Left-forward | `Z` | 5 up, 4 left; `(-4, -5)` |
| Left-backward | `X` | 4 left, 4 down; `(-4, 4)` |
| Right-forward | `C` | 4 up, 4 right; `(4, -4)` |
| Right-backward | `V` | 5 right, 4 down; `(5, 4)` |

## Calibrate a new iPad or application layout

Calibration is device-, orientation-, display-scale-, and application-layout
specific. Recalibrate whenever any of those conditions changes.

Stop the normal controller before calibration so the serial port is free.
Then run:

```bash
cd /path/to/whill_control_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch whill_controller calibration.launch.py \
  serial_port:=/dev/ttyACM0
```

The default output directory is:

```text
~/.ros/whill_controller/calibration_logs/
```

Calibration workflow:

1. Connect `nimble` on the iPad.
2. Wait until the GUI confirms bottom-right homing is complete.
3. Use only `W`, `A`, `S`, and `D` to reach the requested target.
4. Press `Enter` when the pointer is correctly placed.
5. Review or edit the prefilled target name.
6. Choose **Save and collect next point** or **Save and finish**.
7. Repeat until all nine points have been saved.

The collector never sends click commands. Every new point begins at the
bottom-right boundary to avoid a downward trajectory from the iPad's top edge.

Use the newly generated calibration file:

```bash
ros2 launch whill_controller controller.launch.py \
  calibration_file:="$HOME/.ros/whill_controller/calibration_logs/latest_calibration.json"
```

The repository also includes `config/default_calibration.json`, which contains
the locked center and eight center-relative APP targets.

## Two-dimensional trajectory estimate

The controller treats its startup pose as the fixed trajectory origin:

```text
x = 0 m, y = 0 m, yaw = 0 degrees
+X = initial forward direction
+Y = left side of the wheelchair
positive yaw = counterclockwise left turn
```

The current calibration was calculated from physical measurements:

| Command | Measurement | Model value |
| --- | --- | --- |
| `W` | 1.23 m in 3.309 s | 0.3717135 m/s forward |
| `S` | 1.25 m in 3.432 s | 0.3642191 m/s backward |
| `D` | `sin(angle)=0.324675`, angle below 90 degrees, in 0.820 s | 0.4032538 rad/s clockwise |
| `A` | `sin(angle)=0.4133333`, angle below 90 degrees, in 1.096 s | 0.3887880 rad/s counterclockwise |

The model integrates simultaneous linear and angular commands as unicycle
motion, producing arcs rather than independent X/Y translations. By default,
trajectory integration is enabled only in APP locked mode while the sustained
left-button control is active. Pointer-only movement does not change the
estimated wheelchair pose.

This is command-based dead reckoning, not measured odometry. Wheel slip,
acceleration, deceleration, surface conditions, battery state, load, and input
latency can create accumulated position and heading error. Future wheel,
visual, or inertial sensors should be fused with this estimate.

## ROS 2 interfaces

### Topics

| Topic | Type | Direction | Description |
| --- | --- | --- | --- |
| `/whill/controller/status` | `std_msgs/msg/String` | Published | One-Hz JSON snapshot of mode, serial state, active keys, click state, calibration state, per-key and per-mode press totals, timing totals, and the most recent ESP32 command |
| `/whill/controller/event` | `std_msgs/msg/String` | Published | JSON event stream for key presses, releases, key durations, mode changes, target changes, and clicks |
| `/whill/controller/command` | `std_msgs/msg/String` | Published | Auditable JSON record of every byte sequence sent to the ESP32, including a monotonic sequence number and semantic action |
| `/whill/controller/input` | `std_msgs/msg/String` | Subscribed | Safe remote movement-test and mode-toggle input |
| `/whill/controller/motion` | `std_msgs/msg/String` | Published | Live JSON pose, heading, velocity, active-key, and tracking-gate telemetry |
| `/whill/controller/odometry` | `nav_msgs/msg/Odometry` | Published | Standard estimated planar pose and velocity in `whill_origin` |
| `/whill/controller/path` | `nav_msgs/msg/Path` | Published | Accumulated centerline trajectory for visualization |
| `/whill/calibration/status` | `std_msgs/msg/String` | Published | Calibration session status and current point data |
| `/whill/calibration/event` | `std_msgs/msg/String` | Published | Calibration event log |

Inspect live controller status:

```bash
ros2 topic echo /whill/controller/status --full-length
```

Inspect timing and mode events:

```bash
ros2 topic echo /whill/controller/event --full-length
```

Inspect every command issued to the ESP32:

```bash
ros2 topic echo /whill/controller/command --full-length
```

Send one click-free upward test step:

```bash
ros2 topic pub --once /whill/controller/input \
  std_msgs/msg/String "{data: 'tap:w'}"
```

Toggle modes without generating a mouse click:

```bash
ros2 topic pub --once /whill/controller/input \
  std_msgs/msg/String "{data: 'mode:toggle'}"
```

Reset the displayed and published trajectory origin:

```bash
ros2 topic pub --once /whill/controller/input \
  std_msgs/msg/String "{data: 'trajectory:reset'}"
```

Inspect the live motion estimate:

```bash
ros2 topic echo /whill/controller/motion --full-length
ros2 topic echo /whill/controller/odometry --full-length
ros2 topic echo /whill/controller/path --full-length
```

If PlotJuggler is installed, start it with:

```bash
ros2 run plotjuggler plotjuggler
```

Use its ROS 2 streaming plugin to inspect the odometry pose, linear velocity,
angular velocity, and JSON motion telemetry. The built-in dashboard provides
the live XY trajectory without requiring PlotJuggler.

The input topic intentionally does not expose click or button-hold commands.
The command topic is output-only and exists so debugging does not depend on
the Tk dashboard.

### Isolated local debugging

Other Fast DDS applications on ROS domain 0 can corrupt CLI discovery and
produce `sequence size exceeds remaining buffer`. For laptop-only WHILL
testing, source the supplied environment in every terminal before launching
the controller or using `ros2 topic`:

```bash
source /home/zyy/whill_control_ws/whill_ros_env.bash
```

This selects ROS domain 57 and localhost-only discovery. All WHILL terminals
must use the same environment. Localhost-only mode must be disabled later when
the laptop and Orin need to exchange ROS 2 messages over the network.

The currently locked APP center is reached from the bottom-right boundary with
exactly 25 left steps and 14 up steps. The command topic exposes these values
as `approach_a` and `approach_w` on every initialization, so a run can be
verified without looking at the dashboard.

## Key-duration feedback

Each logical key press is timed with a monotonic clock. When the key is
released, the GUI and ROS logger print data similar to:

```text
Key W held 1.253 s · total 3.821 s · key press #2 · all-mode total #7
```

The duration event contains:

- the key name;
- the most recent hold duration;
- cumulative hold duration for that key;
- total press count;
- the active controller mode.

These values describe keyboard input timing. They are **not** measurements of
wheelchair velocity, traveled distance, or motor state.

## Troubleshooting

### The ESP32 serial port does not exist

```bash
lsusb
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
journalctl -k -n 50 --no-pager
```

Reconnect the USB cable, verify the board is powered, and press the board reset
button. Use the ESP32 port reported by the kernel.

For the tested hardware, the correct mapping is:

```text
303a:1001  Espressif USB JTAG/serial debug unit  -> controller port
1a86:55d3  QinHeng USB Single Serial             -> not the ESP32 port
```

If the kernel repeatedly reports `USB disconnect`, or the controller reports
`Write timeout`, the ROS keyboard layer may still log key events while no HID
command reaches the ESP32. Stabilize the USB connection first: reconnect or
power-cycle the hub, check the cable and connector, reset the ESP32 once, and
wait until the Espressif port remains present before launching the controller.

### Permission denied on the serial port

Add the user to `dialout`, log out, and log back in:

```bash
sudo usermod -aG dialout "$USER"
```

### The serial port is busy

```bash
fuser -v /dev/ttyACM0
```

Stop any ESP-IDF monitor, standalone Python controller, calibration collector,
or previous ROS 2 controller that owns the port.

### The iPad shows `nimble` as connected but the pointer does not move

- Confirm that the firmware is configured as a mouse, not a keyboard.
- Confirm that the HID report map uses relative X and Y mouse reports.
- Confirm that the firmware restarts its serial input task after reconnecting.
- Reset the ESP32 and reconnect `nimble`.
- Verify that another Bluetooth mouse still works with the iPad pointer
  settings.

### `nimble` is no longer visible in Bluetooth settings

Reset the ESP32. If necessary, remove the old bond on the iPad and pair again.
The firmware should resume advertising after disconnection.

### The pointer opens an iPad top-edge panel during mode initialization

Use a bottom-right-origin calibration generated by the included calibration
collector. The controller releases the button, homes to the bottom-right safe
boundary, and approaches the center using the recorded sequence. Do not use an
old top-left-origin dataset for this workflow.

### Calibrated targets are offset

- Confirm landscape/portrait orientation matches the calibration session.
- Confirm display scaling and application layout have not changed.
- Press `R` in application mode to reinitialize the pointer.
- Run a new nine-point calibration.
- Apply small visual corrections to the center only after recording the change
  in the calibration JSON.

### Direction keys do not respond

Click the Tk controller window so it has keyboard focus. The current
implementation uses desktop key events and is not a system-wide keyboard hook.

## Known limitations

- BLE HID mouse movement is relative; the ESP32 cannot read the iPad's absolute
  pointer coordinates.
- Calibration is specific to the iPad model, orientation, display settings,
  application version, and interface layout.
- The bundled calibration is an example measured on the tested system and may
  not align with another device.
- The controller GUI must have keyboard focus.
- Keyboard timing is available, but physical wheelchair velocity feedback is
  not implemented.
- This repository does not contain the complete ESP32 firmware project.

## License

The ROS 2 workspace in this repository is released under the MIT License. See
[`LICENSE`](LICENSE).

ESP-IDF and the Espressif example code used by the separate firmware project
remain subject to their own upstream licenses.
