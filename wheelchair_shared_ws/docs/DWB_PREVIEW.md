> 当前已按用户要求删除新增的ODIN/速度接收超时及自定义新鲜度限制。以下旧版涉及这些门槛的描述不再适用；原项目既有授权与停止机制未删除。

> 后续已增加原启动流程中的 continuous_navigation 可选接管；当前集成说明见 docs/EYE_SHARED_CONTINUOUS_CONTROL.md。以下保留独立入口/旧链路背景。

# Existing DWB -> calibrated iPad topic connection

This change reuses DWB/StandardTrajectoryGenerator and the existing five-direction inverse velocity mapping. No new planner, smoother or delay-compensation algorithm is introduced.

## Start the interface (sensors already running)

Close the previous iPad controller, then from the repository root:

```bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 bash scripts/mouse_cmd_vel.sh \
  -p cmd_vel_input_topic:=/wheelchair/dwb_preview/cmd_vel
```

Default (without the parameter) remains mouse-only. The explicit input is geometry_msgs/Twist in the fitted control-center frame, not base_link. Local Arm and the additional “允许Nav2速度输入” button are both required. Mouse movement is disabled during external input. Escape, disabling the button or loss of focus revokes external input; commands absent for 0.3s stop motion. Negative v, unsupported axes and invalid numbers are rejected. Do not run the old twist_to_key/controller simultaneously.

The same process publishes corrected /wheelchair/control_center/odom and odom -> wheelchair_control_center_estimated TF using the existing fitted offset (-0.266192,+0.078385,0). The DWB profile uses that reference, including its translated footprint. Covariances are unspecified, so this output is not for localization fusion.

## Existing DWB server

Keep the existing perception/planning-map producer running. This script does not create a map or global path:

```bash
ROS_LOCALHOST_ONLY=0 WHEELCHAIR_ROS_DOMAIN_ID=0 bash scripts/run_dwb_preview.sh
```

From a ROS Humble sourced terminal:

```bash
ros2 lifecycle set /wheelchair/dwb_preview/controller_server configure
ros2 lifecycle set /wheelchair/dwb_preview/controller_server activate
```

Existing DWB limits: 0<=v<=0.30m/s, |w|<=0.40rad/s, vy=0. Initial acceleration/deceleration settings are +/-0.30m/s² and +/-0.50rad/s², not measured dynamic identification. This rectangle lies inside the initial five-ray velocity envelope. The actuator also clips to these same velocity limits and quantizes to existing HID steps. These limits do not eliminate the measured delay or quantization.

## Remaining path wiring

The current project does NOT automatically connect its global path to DWB: Nav2PathClient requests the existing ComputeDiscretePath service (discrete or Smac backend) and publishes /wheelchair/local_plan, while the current execution stack uses its own path follower. DWB accepts nav2_msgs/action/FollowPath at /wheelchair/dwb_preview/follow_path with controller_id=FollowPath and goal_checker_id=goal_checker. A bare path topic is not a FollowPath goal.

This scoped change connects cmd_vel to the actuator only. It does not replace the global planner, enable the existing RViz manager to send FollowPath actions, or claim RViz navigation is end-to-end integrated. When wiring that action, use the same fitted center for path reference, propagate cancellation, and avoid the old fixed-key executor commanding hardware concurrently.

No local tests or autonomous hardware run were performed for this change, per the user's requested scope. Existing mouse real-hardware checks predate this external input integration.
