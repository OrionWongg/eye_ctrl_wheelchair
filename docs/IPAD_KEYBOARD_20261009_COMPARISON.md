# 10 月 9 日键盘实现核对

## 找到的历史依据

- 原目录 `/home/whill/projects/Ble2WChair` 仍保留控制器源码，文件修改时间为
  `2026-10-09 10:38:10 +0800`。
- 控制器 SHA256：`9afdf0606beae882de21cdc6bfb373a1c691f7f135ab08c2e9cb5c82933eb560`。
- 当天 10:38 的会话记录与源码差异吻合：修复串口 DTR/RTS 造成复位的问题，
  未修改拖动协议；没有烧录新固件，保留原有 nimble 固件。
- 当天 10:49 的操作记录为：原目录 `run_hardware.sh` 加 `latest_calibration.json`，
  自由指针模式启动，Arm → O → 等待中心归位 → F → 方向键。
- 当前固定标定是当天 11:05 完成的九点文件。当天还存在较早的 10:47 版本，
  仅凭操作说明无法确认用户记忆中成功测试具体用了哪次标定；本次未切换标定。
- 查到的当天记录能证明代码和操作方式，未包含 WHILL APP 成功拖动的执行回执。

## 实际指令及差异

旧版 APP 模式：F 发 `[`，W 发八个 `w`，松 W 发八个 `s` 回中心，
此时仍按住左键；之后 D、松 D 也持续按住，直到再次 F 发 `]`。
长按方向键不重复发位移。自由指针模式才在长按 250 ms 后以 20 ms 间隔重复移动。

新入口默认自动拖动：方向键自动按下，松方向键先 `]` 再回中心。
眼动模式也会在停止／方向切换时释放。旧版手动 F 模式的持续接触行为不同。

逐方法 AST 比较确认，旧目录与新目录的 `_write_command`、
`move_locked_to_active_target`、`send_current_direction`、`repeat_once`、
`request_locked_initialization` 和 `_finish_locked_initialization` 完全一致。
`serial_connection.py` 也完全相同。因此没有证据显示旧版本有额外的
“持续拖动命令”或不同的串口发送节奏。

## 直接运行旧源码进行对照

关闭当前控制窗口，避免串口冲突，然后执行：

```bash
cd /home/whill/eye_ctrl_wheelchair
./scripts/ipad_keyboard_20261009.sh
```

这个本机对照入口直接导入原目录源码，使用旧 YAML 参数、ROS domain 57，
以 FREE SCREEN 启动，固定加载 11:05 标定快照。原目录的源码与固件均未修改。
启动会打印旧源码绝对路径及 SHA256，避免误跑新控制器或其他 install 副本。
该入口依赖本机保留的旧目录，并非可独立分发的历史版本。

操作：Arm → O → 等待归位、确认中心命中 → F → W → 松 W → F 释放。
先在不驱动轮椅的条件下对照。F 后应显示 LEFT BUTTON HELD；这个显示仍然
只是主机状态，不是 iPad 回执。Esc 可释放退出。

离线启动：`./scripts/ipad_keyboard_20261009.sh --dry-run`。

如果旧源码能拖动而新入口不能，优先对比持续按住、按下至首次移动的间隔和
输入触发的释放。若两者均不能，不能靠回退主机代码认定已修复，应继续核对
中心命中、当前 iPad 设置、普通系统滑块行为及实际 HID 按钮报告。

## 本次验证

直接导入旧目录控制器，离线运行真实 Tk 按键处理，验证 Arm、O、中心归位、
F 按下、长按 W 不漂移、松 W 按住回中、D 往返、F 释放。实际捕获序列：

```text
[  wwwwwwww  ssssssss  dddddddd  aaaaaaaa  ]
```

该验证通过；未打开真实串口、未执行真实轮椅运动。
