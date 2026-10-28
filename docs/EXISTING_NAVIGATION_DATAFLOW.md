> 后续已增加原启动流程中的 continuous_navigation 可选接管；当前集成说明见 docs/EYE_SHARED_CONTINUOUS_CONTROL.md。以下保留独立入口/旧链路背景。

# 现有导航链路与速度映射接入

原启动入口不变：scripts/run_hardware_keyboard.sh / scripts/run_hardware_rviz.sh。

现有实际数据流：

1. direction_manager / RViz目标 → SafetyGate中的导航状态机。
2. Nav2PathClient → ComputeDiscretePath服务（discrete或Smac后端）→ /wheelchair/local_plan。
3. 现有路径跟踪逻辑 → decision.action。
4. SafetyGate发布/wheelchair/action_safe（DiscreteAction）。
5. Executor执行cfg.velocity(action)，发布/wheelchair/sim/drive（Twist）。
6. twist_to_key → /whill/controller/navigation（TwistStamped）。
7. 原whill_controller → HID串口 → ESP32 → iPad WHILL App → 轮椅。

本次只更换第7步的速度执行映射，并为第6步增加连续透传选项。没有增加规划器或替换任务状态机。旧模式的Arm、按住运行、RViz goal lease、0.2秒超时保持原逻辑。

hardware.launch.py现在开启navigation_continuous_mapping：收到(v,w)后用与鼠标标定共用的velocity_mapping换算整数iPad坐标，不再以阈值只选固定w/a/d。超出实测速度包络的组合沿速度射线限制到包络。原navigation_continuous_mapping默认false，其他独立启动方式保持兼容。

默认仍接/wheelchair/sim/drive；twist_to_key保留旧Executor的纯转向ICC兼容转换，因为该话题的转向linear.x是旧模型附带的平移项。不能把它直接当作控制中心的前进请求。

已有连续速度源可通过原hardware.launch.py参数velocity_topic指定，并设置continuous_velocity_input:=true：桥接不再清零linear.x，同时将消息参考点标为wheelchair_control_center_estimated。因此输入Twist必须确实是在这个控制中心定义的速度；此参数只是指定语义，不执行坐标变换。

关键现状：当前默认Executor上游仍是DiscreteAction，只有STOP/FORWARD/LEFT/RIGHT及其固定速度。底层映射升级不会自动生成上游不存在的连续速度。独立DWB输出可作为连续速度源，但其FollowPath任务与旧SafetyGate/Executor取消、状态反馈并不自动等价，不能仅换话题就宣称完整连续导航任务已对齐。本次没有绕过或改写这部分。

部署目录为symlink install，此次Python和launch修改已位于原启动入口所引用的源文件；重启原进程加载。未运行本地或实车测试。
