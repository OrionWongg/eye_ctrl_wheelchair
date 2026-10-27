# 单帧近车体聚类滤波

已替代此前2/3、3/5滑动确认：删除temporal_filter及其节点接入，不再保存确认历史。scan_map原有history_window=0.25秒是地图融合，保持原样。

正常hardware_bridge.yaml启用cluster_filter_enabled=true。每台雷达分别转换到base_link，经过车体、地面与距离分类后做单帧三维欧氏聚类：连接距离5 cm，点数小于4的簇仅在水平半径1 m内被抑制。半径外保留，额外5 cm边缘支撑避免在1 m边界截断簇；链式连接允许簇整体大于5 cm。两台雷达不混合计算簇点数，无确认等待或运动历史匹配。

被拒绝点的扫描角度输出inf，不用其后方远点构造自由射线；同角度其他障碍也可能暂时不输出。原始时间戳、双雷达新鲜度门控、地面容差及未知格规则保持。真实稀疏小物体也可能被滤掉，尚未完成实物/运动验收。

## 生效

传感器命令保持原样：

```bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ./scripts/start_sensors.sh
```

本次已构建桥接包。退出并重启原正常驾驶控制栈，重启后的cloud_to_scan加载新配置；无需重启传感器。本次未自动停止或重启运行中的驾驶进程，未发送运动指令。

过滤影响/wheelchair/scan及下游地图，不改/mid360_20{1,2}/points_filtered，观看这些点云仍可看到被拒绝点。可在相同ROS环境确认：

```bash
ros2 param get /cloud_to_scan cluster_filter_enabled
```

独立单雷达静态对照可继续用scripts/test_single_lidar_costmap.sh，关闭对照的环境变量现为CLUSTER_FILTER_ENABLED=false。

## 验证

41项硬件桥接测试通过。新增检查：空输入、1–3点拒绝、4点链式连通保留、三维分离、1 m范围与边缘支撑、双雷达不凑点数、拒绝近点不暴露后方自由射线、原始最旧时间戳、不依赖历史。
