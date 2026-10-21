# Willow Garage 办公楼

## 第一人称眼动驾驶（2026-10-19）

眼动入口默认启用乘坐者相机和独立画面窗口：

```bash
cd ~/projects/EYETLE
WHEELCHAIR_SCENE=willow_office ./start_eye_wheelchair.sh
```

相机固定在轮椅基座上方约 1.2 m，朝前俯视约 4.6°，水平视野 90°，
分辨率 960×540，仿真采样 20 Hz，随轮椅位置和朝向变化。它只提供画面，
不改变眼动指令或碰撞几何。按 F 切换全屏；Esc/Q 或关闭画面窗口会关闭
本次仿真，眼动联合启动器随后清理检测器。画面超过 1 秒未更新时显示黑屏提示，
不继续将旧画面当作实时画面展示；窗口本身不发布运动指令。

只保留第一人称窗口、关闭第三人称 Gazebo 窗口：在上述命令前增加
`WHEELCHAIR_GUI=false`。关闭相机和第一人称窗口：`WHEELCHAIR_FIRST_PERSON=false`。
保留相机但不打开窗口：`WHEELCHAIR_FIRST_PERSON_VIEW=false`，适合录制或无界面测试。
纯无界面运行需要同时设置 `WHEELCHAIR_GUI=false WHEELCHAIR_FIRST_PERSON_VIEW=false`。

普通仿真入口可追加 `first_person:=true`；图像 ROS 话题为
`/wheelchair/first_person/image_raw`（`sensor_msgs/msg/Image`）。
相机需要额外 GPU 渲染资源，和双眼推理同时运行的帧率仍需佩戴测试。

验证：`wheelchair_sim` 构建成功，场景与启动器回归 12 项通过；独立仿真中
接收 100 帧，实测约 16.35 帧/秒。使用仿真位姿服务将轮椅平移 0.3 m、
旋转 0.3 rad 后，里程计和相机画面均随之变化。该测试不代表眼动真人驾驶验收。
画面及结果保存在 `artifacts/first-person-20261019/`。

启动：

```bash
~/wheelchair_shared_ws/scripts/run_willow_sim.sh gui:=true
# 等价入口：
~/wheelchair_shared_ws/scripts/run_direction_sim.sh scene:=willow_office gui:=true
```

另一个终端运行 `~/wheelchair_shared_ws/scripts/direction_keyboard.sh`，W 前进，
A/D 方向意图，U 掉头，空格/S 停车，X 退出。出生点在主通行区，初始朝向 +X。

原始建筑约 58.63 × 45.67 m，单位由原 COLLADA 的英寸转换到米，没有放大门洞或通道。
采用原始视觉网格与纹理；碰撞和仿真障碍地图从上游碰撞三角形生成，地图分辨率 5 cm，
保留原 Clearpath 适配方式的 z=0.04–1.30 m 障碍高度带和一格保守栅格扩张。
地图与模型统一平移 (-20, -28.25, 0) m，使轮椅仍从 odom 原点出生。
这是平地轮椅仿真适配，不是整栋建筑任意高度的物理仿真。

源码：<https://github.com/osrf/gazebo_models/tree/master/willowgarage>。
模型作者 Nate Koenig；上游仓库许可 CC BY 3.0，署名和许可保存在模型目录。
`SOURCE.json` 记录固定提交，`obstacle_map.json` 记录源网格 SHA256 与导入参数。
完整原始资产保存在 `vendor/willowgarage`，离线再生成入口为
`python3 scripts/import_willow_office.py`。

新场景使用现有控制器、规划服务及安全检查。切回原办公室时使用
`scene:=clearpath_office`。不要同时启动两套相同 ROS 域的仿真。

验收脚本：`python3 scripts/validate_willow_office.py`，测试 W 巡航 6 m、STOP、
停车右转和恢复前进，记录实际接触数及运行时故障。

2026-10-16 验证：场景几何回归 6/6 通过；Gazebo 巡航、停车、停车右转、
恢复前进用例通过，行驶 6.57 m，物理接触为 0，最小地图几何净距约 0.45 m。
证据：`artifacts/willow-office-20261016/cruise_turn_resume/result.json`。
`startup-probe/` 是首次静态地图尚未就绪的失败记录，不能计为通过。
