# costmap 启动时缺少 odom 的修复

## 本次日志原因

`host_sdk_sample_47430_1793008758759.log` 显示 Odin 在 18:02:31 收到 SIGINT 并退出；18:02:59 的硬件 launch 随后启动 cloud_to_scan、odom_relay、scan_map 和 costmap。safety_gate 日志中 scan、odom、map 均无数据，cloud_to_scan 只记录启动输入，没有后续输入证据。因此这次不是单雷达参数把 odom 坐标名改错，而是传感器端已停止。

另一次 18:02:44 启动把 lidar_mode 写为 20，launch 已明确拒绝；有效值为 both/201/202。

## 修复

仓库内 run_hardware_keyboard.sh 将 launch 参数传给 preflight，启动控制栈前检查：

1. 没有已有的派生感知/控制发布者。
2. 选定雷达的 points_filtered 已收到且时间戳新鲜；默认双雷达时要求两台。
3. /odin1/odometry_highfreq 已收到、时间戳新鲜、父坐标为 odom。
4. Odin 消息的 child frame 到 base_link、base_link 到雷达以及扫描时刻 odom 到 base_link 的 TF 可查询。

最多等待 20 秒（另有 2 秒 DDS 发现），每 5 秒打印缺失原因。超时返回非零，脚本不会启动 costmap 或物理输出。未增加任何虚构的静态 odom TF。运行中传感器退出仍由原有时效门控停车，此改动只处理启动顺序和错误提示。

## 启动顺序

终端一保持运行：

```bash
cd ~/eye_ctrl_wheelchair
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/start_sensors.sh --mid360 201
```

终端二：

```bash
cd ~/eye_ctrl_wheelchair
WHEELCHAIR_ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/run_hardware_keyboard.sh lidar_mode:=201
```

单雷达模式仍强制禁用物理轮椅输出。两终端不要共用一个前台任务，也不要启动第二个前关闭传感器终端。

## 验证

37 项硬件桥接测试通过，覆盖缺少 odom、缺少 TF、过期里程计、错误 frame、单双雷达选择。仓库内硬件桥接包构建通过。独立 ROS domain 中无传感器运行，preflight 超时并报告缺失雷达及 Odin，未启动任何控制节点。现场传感器已退出，未重新开启硬件进行运动测试。
