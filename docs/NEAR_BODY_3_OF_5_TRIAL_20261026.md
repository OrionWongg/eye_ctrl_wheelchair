# 3/5 帧确认试验

> 历史记录：该滑动窗口实现已被单帧聚类替代，当前入口与规则见 [聚类滤波说明](NEAR_BODY_CLUSTER_FILTER_20261026.md)。

在原静态稀疏回波试验上将默认规则由2/3改为3/5：当前必须有点，前4帧至少2帧在8 cm三维邻域内有任意障碍回波。每帧只计一次；最近5帧中总计至少3次才保留候选。只处理距车体60 cm内、8 cm邻域少于4点的稀疏回波。历史不足4帧或断帧时直接保留，不将缺测当作自由空间。10 Hz连续命中新候选约增加200 ms确认延迟，间歇命中可能400 ms。

默认规则仅在启用 sparse_persistence_enabled 的节点生效，正常驾驶仍默认关闭。匹配仍使用base_link，未加入运动补偿，不能作为已经完成运动验收的过滤版本。传感器启动脚本和正在运行的节点不自动改变。

静止对照（先退出原测试）：

```bash
cd ~/eye_ctrl_wheelchair
./scripts/test_single_lidar_costmap.sh 201
# 关闭过滤作对照
SPARSE_PERSISTENCE_ENABLED=false ./scripts/test_single_lidar_costmap.sh 201
```

43项硬件桥接测试通过，保留2/3回归，并新增3/5阈值、间歇命中、断帧重置测试。未发送运动命令，未进行现场效果验收。
