> 当前已按用户要求删除新增的ODIN/速度接收超时及自定义新鲜度限制。以下旧版涉及这些门槛的描述不再适用；原项目既有授权与停止机制未删除。

# 人工输入映射与运动对照

传感器启动：

```bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/start_sensors.sh
```

关闭旧iPad控制进程后，在另一个终端启动：

```bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 bash scripts/mouse_cmd_vel.sh
```

原控制窗口负责归中、Arm和录制按钮。新增“鼠标直接操控与运动对照”窗口负责鼠标输入与图表。等待归中后点击Arm，按住鼠标左键拖动，松开停止。下沿中心为零，上为前进，左/右为转向，仅第一、第二象限。默认调整模式；开启录制模式后每次拖动单独录bag，松开后继续记录3秒。

控制链路：鼠标归一化位置→五方向标定点构成的区域→取整HID坐标→原update_keyboard_target→USB串口→ESP32 BLE HID→iPad WHILL App→轮椅。不经过速度反算，也不自动反馈修正。超出标定多边形的鼠标位置沿射线限制到边界。八个显示档位不保证八个不同HID位置；当前接口只能发整数步。

初始速度模型（拟合旋转中心处）：前进(0.548,0)，左(0,+0.821)，右(0,-0.800)，左前(0.394,+0.581)，右前(0.384,-0.590)，单位m/s、rad/s。按相邻标定方向分片线性插值预测速度，使用最终取整位置计算。任意中间方向仍待人工采样验证。

绿色轨迹为鼠标在操控平面的轨迹。蓝色预测轨迹由预测速度积分，红色实际轨迹取ODIN位姿换算至拟合控制中心，两条空间轨迹每次拖动从共同起点对齐。速度图蓝预测、红实测。显示实测速度经过轻度滤波，bag保留原始ODIN与派生未滤波速度。预测轨迹仅作模型对照，不是用户指定的导航路径。

/experiment/cmd_vel：此测试中仅发布模型预测速度供记录与观察，不订阅并反算执行。
/experiment/measured_velocity：原始ODIN速度经姿态、安装杆臂和拟合中心转换后的速度。
/experiment/velocity_mapping：鼠标输入、整数HID目标、预测速度及执行结果。

控制参考点相对base_link为(-0.266192,+0.078385,0)m，是数据拟合值，未现场测量，也未广播为TF。frame_id=wheelchair_control_center_estimated，不可直接作为base_link速度命令使用。

记录保存在bag/mouse_velocity_时间目录，每次拖动一个bag。原始标定、模型相关诊断与就绪状态一并保留。停止时鼠标释放HID；传感器接收断流、失焦或未Arm也停止。ODIN消息header时间差仅作记录，不作为启动限制；最近收到可用速度的检查仍保留。GUI内检查不是独立硬件看门狗。

后续经用户明确授权，已完成左右小幅变档及前进/左前/右前实车验证，后三段各保持1.8秒；结果见bag/live_mouse_forward_20261028_233343/RESULT.md。鼠标移动回调立即发送HID，不等待画图或轨迹结束；异常回调会停止并撤销Arm。后续根据速度幅值偏差、动态响应和空间轨迹误差，分别修正数值映射、延迟与控制参考点，再接入cmd_vel逆向控制链路。

## 显式Nav2速度入口
可通过原启动脚本追加 `-p cmd_vel_input_topic:=/wheelchair/dwb_preview/cmd_vel`，订阅Twist，局部Arm并点击允许Nav2按钮后启用。默认鼠标模式不变。外部速度按既有逆向模型转换成HID，限定v为0至0.30、w为±0.40。另发布控制中心odom/TF供DWB使用，参见wheelchair_shared_ws/docs/DWB_PREVIEW.md。现有全局路径到FollowPath action尚未自动接通。
