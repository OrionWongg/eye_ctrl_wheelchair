# RViz pose goals

Start sensors as usual. Goal input is opt-in and only applies to the
`directional_intent` task manager:

```bash
./scripts/run_hardware_keyboard.sh enable_rviz_goal:=true
```

In another terminal, load the ROS environment and start the live display:

```bash
source /opt/ros/humble/setup.bash
source wheelchair_shared_ws/install/local_setup.bash
export ROS_DOMAIN_ID="${WHEELCHAIR_ROS_DOMAIN_ID:-0}"
rviz2 -d sensor_visualization/goal_navigation.rviz
```

Use RViz's **2D Goal Pose** tool (some versions label it **2D Goal**).
The tool publishes `geometry_msgs/msg/PoseStamped` on `/goal_pose`.
The existing `/move_base_simple/goal` topic is also accepted, so the original
hardware RViz display works too. Startup logs state whether the interface is
enabled; every received goal logs acceptance or the rejection reason.
Keep Fixed Frame at `odom`; drag the arrow to set the terminal heading.
The interface accepts only fresh, planar odom poses. It does not transform
map-frame goals, provide NavigateToPose actions, or explore unknown space.

Goals are accepted only when stopped, with fresh control/odometry feedback,
no active or pending keyboard task, and no rearm requirement. The existing
iPad hardware Arm is still required. After accepting a goal, the manager
renews the same finite task without requiring a held key. It uses the existing
collision checks, path service, and straight/turn follower. It stops at the
terminal pose instead of extending into cruise.

The opt-in flag also enables `/wheelchair/goal_authority` at the iPad bridge.
The manager publishes a timestamped task lease every 50 ms; its maximum age is
250 ms. This lease exempts only the active RViz task from the held-key check.
Local Arm, calibration, and the drive-command watchdog remain required.
Arm before sending a goal: a goal received while disarmed must be sent again.
Disarm, keyboard takeover, local stop, or lease expiration revokes that task;
late renewals cannot restore it. Moving focus to RViz does not cancel navigation
when no physical direction key is held. Focus loss while holding a direction
still invokes the keyboard stop behavior.

Any keyboard direction/stop event or emergency stop cancels the goal; it never
resumes automatically after keyboard takeover. New goals while moving are
rejected. Completion, blocked paths, stale feedback, clock faults, controller
restarts, or the 120-second goal timeout terminate navigation. The timeout is
the task manager's `goal_timeout` ROS parameter. No new chassis publisher is
created. Without the opt-in flag, the goal subscriber is absent.

Monitor the separate goal status (legacy keyboard navigation status remains
owned by the keyboard task manager):

```bash
ros2 topic echo /wheelchair/goal_status
```

Cancel independently of keyboard input:

```bash
ros2 topic pub --once /wheelchair/emergency_stop std_msgs/msg/Empty '{}'
```

The goal must be reachable within the current known local map and path
provider's search limits. This adds an input interface, not a different motion
model; existing turn clearance and actuator response limitations still apply.
Validate in a hardware-disabled launch before driving. Never replay recorded
goal/control topics into the live hardware ROS domain.

## One-command RViz + Smac startup

With sensors running:

```bash
./scripts/run_hardware_rviz.sh
```

This selects Smac, enables pose goals, and opens the configured RViz display
in the same ROS domain. Arm in the iPad window, then use **2D Goal Pose** in
RViz; drag to set the final heading. Closing RViz stops the hardware stack.

To compare with discrete using the same goal interface:

```bash
./scripts/run_hardware_rviz.sh path_planner:=discrete
```

The equivalent explicit keyboard entry point is:

```bash
./scripts/run_hardware_keyboard.sh path_planner:=smac_lattice \
  enable_rviz_goal:=true rviz:=true \
  rviz_config:="$PWD/sensor_visualization/goal_navigation.rviz"
```
