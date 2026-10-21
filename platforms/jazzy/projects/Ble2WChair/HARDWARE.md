# 当前 ESP32 硬件配置（2026-10-09）

- 芯片实测：ESP32-S3 rev 0.2，32 MB octal 1.8 V Flash，16 MB embedded PSRAM。
- USB Serial/JTAG：`303a:1001`，USB 序列号 `90:E5:B1:CC:F0:14`。
- 当前端口 `/dev/ttyACM0`，固定别名 `/dev/ble2wchair`。
- BLE 地址 `90:E5:B1:CC:F0:16`，设备名 `nimble`。
- 蓝牙扫描确认 appearance `0x03c2`（鼠标），服务 UUID `0x1812`（HID）。
- 板上已有 WHILL BLE Mouse 固件，基于 ESP-IDF v5.3.2，编译时间 2026-07-21。
- 本次没有覆盖或烧录固件。已读取完整 32 MB Flash 备份及 SHA256：
  `/home/whill/esp32-backups/original-90e5b1ccf014-20261009.bin`。
- 固件内可确认方向、显式按下/释放、归位、`BLE_CONNECTED` 等协议字符串；
  配对前的串口查询未收到回复，发生过写超时。需在 iPad 连接后验证实际通信。

## 使用

1. 保持 ESP32 USB 连接。
2. iPad「设置 → 蓝牙」选择 `nimble`。
3. 电脑桌面终端运行：

   ```bash
   cd /home/whill/projects/Ble2WChair
   ./run_hardware.sh
   ```

4. 点击 **Arm Keyboard Control**，使用 W/A/S/D 测试指针。
5. 使用真实轮椅前先完成 iPad 布局标定。

`run_offline.sh` 仍为无硬件模式。`run_hardware.sh` 默认自由指针模式，
通过 `/dev/ble2wchair` 打开真实串口。不要同时运行两个控制器或串口监视器。

udev 规则 `/etc/udev/rules.d/70-ble2wchair-esp32.rules` 仅匹配这块板，
授予 whill 访问权限并建立固定别名；用户也已加入 dialout，组权限在重新登录后生效。

本次确认范围：芯片识别、完整备份、启动日志、鼠标广播、电脑端真实串口启动。
iPad 实际配对、指针动作、重连与实体轮椅控制仍需连接后验证。

## 串口启动导致蓝牙断开的修复

原控制器和标定工具把 DTR/RTS 均设为 False。实机复现：打开原生 USB
串口后出现 USB_UART_CHIP_RESET 启动日志。共享 serial_connection.py 现按
USB VID/PID 303a:1001 识别该接口（支持符号链接），保持 DTR/RTS 为 True，
并清除 HUPCL，避免关闭串口时降下控制线。其他接口保留原行为。
修复后连续三次打开真实串口未观察到重启；10 项包测试通过。
iPad 配对连接保持需用户重新连接后验收。

截至修复时 latest_calibration.json 为 0 个点，仍需重新采集。
路径不能在引号内部换行或加入空格。
