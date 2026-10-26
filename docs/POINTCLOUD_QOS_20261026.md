# 点云链路 best-effort 调整（2026-10-26）

按用户要求优化最新帧处理，保留现有 Fast DDS。本次将下列 PointCloud2 与派生 LaserScan 通道统一为 BEST_EFFORT、KEEP_LAST(1)、VOLATILE：

- 外部 Livox 驱动 PointCloud2 / CustomMsg 点云发布（双雷达、单雷达共享实现）。
- 外部 Odin 驱动 cloud_raw、cloud_slam、cloud_render 点云发布；附带 depth 点云发布及 depth/reprojection 节点的点云订阅同步适配。
- 仓库 body_cloud_filter 原始点云订阅、filtered 点云发布（原先 best-effort、depth=5，现在 depth=1）。
- cloud_to_scan 的 filtered 输入、ground_points 输出（原已 best-effort、depth=1），scan 输出（原 reliable、depth=10）。
- scan_map 扫描订阅、点云/扫描显示监控及 preflight 点云订阅只保留最新 1 帧。RViz 点云队列和扫描订阅深度同步为 1，原已使用 best-effort。

里程计、IMU、图像、TF、地图和控制命令的 QoS 不在本次修改范围；没有修改扫描时间戳、同步窗口、地面阈值和安全超时门限。depth/reprojection 节点并非正常传感器入口的一部分，此次只确保它们以后启动时能订阅 best-effort 点云；其内部图像同步队列未调整。

本次实际修改了 `/home/whill/livox_ws/src/livox_ros_driver2` 与 `/home/whill/odin_ws/src/odin_ros_driver` 的点云 QoS 源码，并重新构建安装。仅本次改动的补丁保存在 [patches/pointcloud_qos_20261026](patches/pointcloud_qos_20261026)，不包含这两个外部工作区原有的其他改动。旧传感器脚本已转发仓库入口，正常启动命令保持不变。

best-effort 允许丢帧，depth=1 限制 DDS 历史队列；它们不会取消正在执行的回调，也不会自动缩短雷达积分、双雷达等待或算法计算。必须通过同场景对照测量过期比例与尾部延迟确认收益。

## 验证

- Livox、Odin 驱动构建及安装通过；Odin 的 pcap 功能禁用提示为构建警告。
- 现有感知测试 41 项通过；Python 语法和节点本地 QoS 检查通过。
- ROS Domain 92 临时启动传感器、安装 TF、body_cloud_filter、cloud_to_scan 与显示监控，不启动 RViz、地图、规划器或任何执行器/控制节点。预热 15 秒后采样 25 秒，之后关闭本次全部子进程。
- 实际发现的全部点云与 scan 发布/订阅端均为 BEST_EFFORT，9 个检查话题全部收到数据。DDS 端点发现报告的 depth=0 为该接口未提供有效队列深度，不能据此解释为实际队列为 0；本地 Python 实体检查及 C++ 源码配置均为 KEEP_LAST(1)。
- scan 到达时的观测年龄：P50=261.2 ms、P95=271.1 ms、最大=285.9 ms；接收频率 18.64 Hz，最大接收间隔 212.5 ms。
- 此次没有完整控制栈和 RViz 的同等负载，也没有更改前同场景重测，因此不能将差异全部归因于 QoS，更不能证明完整控制栈已无过期。到达时的年龄还不包含等待下一帧期间持续增长的年龄。

原始汇总：[pointcloud_qos_trial_20261026.json](pointcloud_qos_trial_20261026.json)。
