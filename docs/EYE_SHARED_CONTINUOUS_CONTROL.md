# 目的与实际接线

眼镜/键盘是驾驶意图输入，不是速度轨迹生成器。上层眼镜协议、眼镜确认、W/A/D等价输入、direction_manager/RViz目标授权保持原实现。

空旷且完整车体停车包络安全时，前进使用标定满幅0.548m/s，左右满幅约+0.821/-0.800rad/s；原有限角度眼镜任务仍由原逻辑决定停止时刻，未擅自改成无限持续转向。障碍接近、路径需要转弯、终点需要减速时，原生Nav2 DWB跟踪已有全局路径，输出连续v、w；重新满足开放区域/直线路径条件后恢复前进满幅。禁止倒退、横移；无可行路径时停止。

## 新模式下的数据流

眼镜或键盘 → 原意图/任务管理 → 原SafetyGate与Smac/Discrete全局路径服务
→ 下游执行选择（空旷满幅 / 原生DWB FollowPath）
→ SafetyGate的连续速度与同周期DiscreteAction授权
→ 原Executor保留连续数值并检查授权有效期
→ /wheelchair/sim/drive（名字沿用，但值不再局限于四个固定值）
→ 原twist_to_key透传 → /whill/controller/navigation
→ 原whill_controller的共享速度逆映射 → HID → ESP32 → iPad → 轮椅。

/wheelchair/velocity_safe是内部配对通道，其header.frame_id携带安全会话+序号作为配对标识（不是TF帧）；Executor必须同时收到对应的DiscreteAction才执行。面向iPad的TwistStamped重新标为wheelchair_control_center_estimated。

已删除此次新增的传感器/意图/地图时间戳新鲜度门槛、连续速度额外超时、自定义DWB等待超时和模式切换等待。原项目已有授权/停止机制保留，Nav2自身参数保持原生。

DWB运行在/wheelchair/dwb_preview，但在本模式不再仅预览：由下游路径适配器发送/取消FollowPath。原眼镜/键盘停止、急停、任务取消最终都会停止授权并取消路径。没有新增规划算法、眼镜输入逻辑或自动速度增益学习。

## 启动

先关闭旧鼠标/iPad控制窗口，停止并重启原传感器入口，使修正后的odom转换生效：

```bash
cd /home/whill/eye_ctrl_wheelchair
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/start_sensors.sh
```

键盘代替眼镜测试，仍用原入口，仅增加一个下游开关：

```bash
cd /home/whill/eye_ctrl_wheelchair
ROS_DOMAIN_ID=0 WHEELCHAIR_ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 \
  bash scripts/run_hardware_keyboard.sh path_planner:=smac_lattice continuous_navigation:=true
```

等待归中后在原iPad窗口点击Arm。原来的按住W/A/D、松开停止方式不变。不要另开鼠标控制器或独立DWB server，原launch已启动所需实例。continuous_navigation默认false，旧模式仍可使用。

原眼镜启动方式只需在相同hardware.launch入口传continuous_navigation:=true；不变更眼镜协议或配置。

## 配置与反馈

- DWB避障速度初始限制0至0.30m/s、角速度±0.40rad/s，非全局满幅限速。
- DWB加减速初值0.30m/s²、0.50rad/s²，仅为试运行配置，不是精确动力学辨识。
- 地图与安全配置恢复原入口：nav2_hardware.yaml、narrow_control.yaml/control.yaml；连续模式不再替换footprint或膨胀参数。
- DWB使用完整轮廓碰撞critic，并在输出后复用原停车包络检查，不额外引入自定义响应等待预算。
- ODIN世界系线速度先旋转到车体系，再进行安装杆臂转换。sensor-side odom_relay为控制中心odom/TF唯一发布者。
- 控制中心相对base_link为拟合值(-0.266192,+0.078385,0)m，路径与footprint做同一转换；仍应现场核实几何。
- 保留停止超时、输入过期和原局部Arm/goal lease；连续指令超过0.2秒不再驱动iPad。
- 整数HID仍造成量化；不能据接线成功宣称精确动态跟踪。

## 已验证与未验证

已做隔离ROS域的原生DWB action集成：收到路径，连续输出120条非零速度，最大0.30m/s，取消后最后速度为零。
已做原Executor的ROS配对集成：完整保留(0.137,0.223)，收到42条非零输出；撤销授权续期后输出归零。
此前隔离检查使用过未经授权替换的轮廓配置；该覆盖已撤销。地图与轮廓现恢复用户原配置。
这些检查没有连接硬件。此前鼠标→iPad→轮椅真机试验验证的是底层链路，不等于本次完整避障接管已经实车验证。后续曾进行真机goal尝试并遇到INITIAL_HARD_BOUNDARY；用户随后明确确认真机goal导航已验证，此处以用户现场确认作为最新状态。

RViz目标模式：`ROS_DOMAIN_ID=0 WHEELCHAIR_ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 bash scripts/run_hardware_rviz.sh`，默认开启continuous_navigation与已有goal接口。Arm后使用2D Goal Pose，Fixed Frame为odom，发布/goal_pose。
