# 五方向速度响应录制

本工具只录包和离线分析，不控制轮椅、不打开串口、不修改标定或 Nav2 参数。

## 当前映射

现有底层导航接口把 Twist 按阈值变成固定 w/a/d 方向，速度大小没有映射到控制轮幅值。手动 APP LOCKED 控制支持五个前半区目标：W 正前、A 左、D 右、Z 左前、C 右前。Z/C 是专用斜向键。平板标定是鼠标步坐标，不是实测速度或最高速度证明。

## 开始

先按现有方式启动传感器；在另一个终端启动独立手动控制窗口：

```bash
cd /home/whill/eye_ctrl_wheelchair
bash scripts/record_motion_bag.sh keyboard
```

这是原有的直接手动控制入口，不带导航避障。不要与其他占用串口的控制器同时运行。等待 BLE/APP 归中就绪，操作由现场人员执行。

检查和录制终端：

```bash
cd /home/whill/eye_ctrl_wheelchair
bash scripts/record_speed_response.sh check
bash scripts/record_speed_response.sh record
```

默认 ROS domain 为 0；可用 WHEELCHAIR_ROS_DOMAIN_ID 指定，须与传感器、控制窗口一致。检查要求 /wheelchair/odom 和 /whill/controller/command 的正确类型发布者存在；存在不等于传感器质量或运动资格通过。

录制路径打印后再开始操作。先静止约 5 秒；每次只测一个方向，松开后等到完全停止，再留约 3 秒。每个方向重复至少 3 次，顺序写入打印的 operator_notes.txt，并记录乘员/负载、APP 速度设置和地面。保持条件一致。

测最大响应时需要足够空间、由操作者逐步延长保持时间到可观察稳定平台；不要把短脉冲结束时仍在加速的峰值当作最大速度。可先做短动作检查方向，再收集稳态。录包器不会自动驱动、自动延长动作或自动换方向。

全部松开并停稳后，在录制终端 Ctrl+C，等待 bag 正常结束。无需录全部雷达点云；TF、实际里程计、IMU、指令及按住/释放事件足以支持本阶段辨识。

## 分析

```bash
bash scripts/record_speed_response.sh analyze /home/whill/eye_ctrl_wheelchair/bag/speed_response_具体时间
```

默认生成相邻的 `<bag>_analysis` 新目录，已存在时拒绝覆盖；可用 --output 指定另一个新目录。

- trials.csv：方向、保持时间、是否从静止开始、是否释放并完整停稳、起动时延、松开到停稳、vx/vy/角速度峰值/P95、1 秒平台候选和采样间隙。
- summary.json：五方向结果、干净平台样本数、中位平台 vx/vy/角速度、里程计频率/P95/最大间隔及接收帧龄。
- 平台只使用按住期间的数据；松开后的惯性峰不混入按住期稳态。停稳必须有至少 0.3 秒实际低速观测。
- “平台候选”使用工程阈值检查 1 秒窗口，不能直接证明电机极限；记录不够长时不输出可信稳态值。
- 原始 bag 保留位姿与 twist，可进一步做位姿求导交叉核验。/whill/controller/odometry 是按键积分估计，不用于上述实测指标。

这里的跟踪率先指测量频率/连续性和动作响应；目前没有连续目标速度，不输出伪造的目标跟踪成功率。拟合并接入映射后，才计算目标与实际 v/omega 的跟踪误差。

本次满幅五方向用于估计速度包络和起停响应。要拟合 0 到最大速度的映射，还需后续中间幅值响应；本工具不会把五个点直接当作已验证的全区间线性关系。数据回来后再调整 Nav2 的实测可达速度、加减速度和映射，当前不提前修改。
