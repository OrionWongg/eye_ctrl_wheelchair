# 当前推荐入口（测试与正常控制分开）

正常双雷达键盘控制仍用 `scripts/start_sensors.sh` 和 `scripts/run_hardware_keyboard.sh`，后者已移除测试引入的 preflight 等待。正常调用不需要 lidar_mode。

单雷达静态 costmap 测试现在使用独立的一条命令：

```bash
cd ~/eye_ctrl_wheelchair
WHEELCHAIR_ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/test_single_lidar_costmap.sh 201
```

先退出已有传感器/控制程序并给另一台雷达断电。脚本自行启动选定 MID360、Odin、安装 TF、过滤器和 RViz，等待数据及 TF 后再启动转 2D、scan_map 和 Nav2 costmap。不会启动键盘、规划器、执行器、iPad 控制器或 twist_to_key；Ctrl+C 关闭本次测试启动的全部进程。左侧测试使用 202。测试不能与正常控制同时运行。

以下旧的 `run_hardware_keyboard.sh lidar_mode:=201` 测试说明已由这个独立入口取代；兼容参数仍保持禁止物理输出，正常入口不承担测试等待流程。

# 仓库内传感器启动配置

从原 `/home/whill/sensor_visualization` 复制运行所需文件，保留既有传感器安装 TF 和驱动设置。后续通过仓库入口启动：

```bash
cd ~/eye_ctrl_wheelchair
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/start_sensors.sh
```

车体过滤使用本目录 `body_filter.yaml`：base_link 中 X=[-0.42,0.63] m、Y=[-0.40,0.40] m，不限制 Z。范围内所有点都会删除，包含进入该范围的其他物体；这是按位置过滤，不是物体识别。点云输出仍为两台 `/mid360_20{1,2}/points_filtered`。

先退出旧传感器和控制栈，再使用此入口。原外部启动脚本仍使用旧的左右 31 cm 配置；本次没有修改它。驱动依赖仍由 livox_ws 和 odin_ws 提供。

本次只扩大左右车体过滤范围；未扩大规划 footprint，也未更改已有 TF 地面参考。规划轮廓与实际车身尺寸仍需统一复核。

## 单雷达静态互扰对照

先退出控制栈和传感器启动脚本。保留右侧 .201 并给左侧 .202 断电：

```bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/start_sensors.sh --mid360 201
```

反向测试：退出上一轮、给 .201 断电并接通 .202，再使用 `--mid360 202`。恢复双雷达使用 `--mid360 both`（默认）。驱动配置只列出选定 IP；这不等于让另一台硬件停止发射，测试互扰必须实际给另一台断电。

保持轮椅与测试物体位置不变，在 RViz 选择对应原始 `/mid360_201/points` 或 `/mid360_202/points` 对比，避免车体 mask 掩盖变化。单台、对方断电时正常，而同一台在对方通电时反复出现噪点，才能增强互扰判断；单台也异常则应继续检查地面、反射、遮挡和安装。

本模式只做静态点云检查，不启动运动控制。双传感器转换节点仍要求两台新鲜点云，不自动降级为单台行驶。Odin、安装 TF、过滤器和 RViz 仍由传感器启动入口提供。缺失雷达的 watchdog 提示在单雷达测试期间属于预期现象。

## 临时单雷达 costmap 接口

单雷达传感器启动后，可另开终端启动静态 costmap 检查（替换上一节“不能启动转换”的旧限制）：

```bash
cd ~/eye_ctrl_wheelchair
WHEELCHAIR_ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/run_hardware_keyboard.sh lidar_mode:=201
```

检查左雷达改成 `lidar_mode:=202`，与传感器 `--mid360` 选择一致。该参数让 cloud_to_scan 只订阅选定的 filtered 点云并要求其新鲜，不自动等待或回退到另一台。默认 both 保持原双传感器行为。

单雷达模式下 hardware.launch.py 强制不启动 twist_to_key 和 whill_controller，即使 hardware_output:=true 也不会开启物理轮椅输出。保留 odom、scan_map、costmap、规划/安全诊断和终端键盘。此接口用于观察 costmap，不用于单雷达行驶。按 Ctrl+C 退出，恢复双雷达时去掉 lidar_mode 参数。

检查地图可在 RViz 添加 Map，Topic 为 `/wheelchair/local_costmap/costmap`，Color Scheme 为 costmap，Fixed Frame 为 odom；原始占据图为 `/wheelchair/planning_map`。需要已有调试视图时另开终端运行 `WHEELCHAIR_ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/show_hardware_navigation.sh`。

本次 33 项硬件桥接测试通过；另行验证 both/201/202 三种模式下两个物理输出节点的启动条件，单雷达均为 false。未启动真实硬件。


地面阈值：正常硬件 YAML 与独立 single_lidar_costmap_test.launch.py 均使用 ground_tolerance=0.08，即 base_link Z=[-0.536,-0.376] m 内归为地面。退出并重启控制栈或测试脚本生效；8 cm 容差带内的实物可能被归为地面，不代表可通行高度。旧入口 `./scripts/start_sensors.sh` 已转发到仓库入口，传感器脚本本身不启动地面分离。

## TF 结构（Nav2 / REP105）

Odin 驱动的 `send_odom_baselink_tf` 已设为 0，驱动不再发布任何 TF。`odom -> base_link` 由 `odom_relay`（随 `sensors_tf.launch.py` 启动，唯一发布者）根据 `/odin1/odometry_highfreq` 发布；所有传感器静态 TF 直接挂在 `base_link` 下：

```
map -> odom -> base_link -> lidar / imu / camera_0 / base_footprint / mid360_201_local / mid360_202_local
```

`map -> odom` 留给后续 SLAM。`hardware.launch.py` 不再启动 `odom_relay`。

## 安全轮廓与规划诊断

正常 start_sensors.sh 现在同时启动只读 navigation_debug，原命令无需追加参数。
重启传感器入口后，RViz 的 Wheelchair avoidance 组显示规划图、costmap、
目标箭头、规划路径（绿色，带朝向轴）和实际轨迹（白色）。固定坐标为 odom，
相机仍跟随 base_footprint。控制栈需另外启动，传感器入口本身不产生运动命令。

sensors.rviz 只保留以下 MarkerArray（Nav2 footprint 与 local costmap 为 Nav2 自带）：
- hard：最终硬安全边界（停车保护区域），即安全门碰撞检查所用轮廓。
- navigation_goal：橙色目标朝向箭头。
- perception_body_mask_not_safety / perception_self_echo_not_safety：点云过滤与二维自身回波排除范围。
- perception_blind_band_ideal_not_safety：侧后方未知盲区清除的连续几何示意；实际操作以栅格及格中心条件为准。

避障只保留"车体 + margin"一个停车保护区域，预测只计算刹车距离；
名义车体、模型误差/采样余量、舒适边界、预测采样/包络/终点、碰撞最近点
和通道边界已从控制逻辑中删除，navigation_debug 不再发布这些话题。

只启动传感器时，轮廓采用已安装默认硬件配置并显示等待控制栈提示；
收到 decision_trace 后，车体与硬余量取实际决策数据。舒适距离与盲区配置启动时读取；
修改配置后须重新构建相关包并重启。未收到预测时不生成假预测；
旧决策超过 0.5 秒标 STALE 并淡化，历史规划路径的年龄在文字中显示。
纯转向路径位置重合，因此绿色线可能不明显，可看路径朝向轴和目标箭头。

安全图层现已拆为独立 /wheelchair/debug/<namespace> 话题，左侧 Displays
分别列出中文名称和颜色，可单独开关并展开 Topic 查看对应关系。
默认不显示画面文字；需要状态时手动启用“控制状态文字 [默认关闭]”。
原聚合 /wheelchair/debug/markers 保留供已有诊断视图使用。
