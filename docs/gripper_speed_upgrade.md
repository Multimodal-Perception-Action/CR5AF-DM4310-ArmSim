# DM4310夹爪网关速度升级实测

## 当前结论

- T-CAN485 的 CH9102 已准确识别为 `COM6`；U-Arm CH343 为 `COM7`，两者没有混用；
- build 1.2 将全开/全闭由旧版约10–11秒降至2.653/2.703秒；
- build 1.3 guarded-fast 已刷入，identity诊断值为`1.3`；
- build 1.3 全开/全闭实测为2.265/2.281秒，两次动作均达到端点并确认最终失能、无故障；
- `COM7`仍绝不能用作网关刷写口。

## 1.3相对旧固件的运动参数变化

`GRIPPER_MOVE_STEP_RAD` 从旧版 `0.04 rad` 经1.2的`0.08 rad`增至1.3的`0.10 rad`，节拍为10 ms；
达到目标后的保持阶段从固定最多20次改为最少3次、最多10次，并在位置误差不超过0.06 rad且速度不超过0.20 rad/s时提前完成。以下保护保持不变：

- 每一个位置步进均等待DM4310反馈；
- 每步仍有节拍间隔；
- 实测速度绝对值不得超过3 rad/s；
- MOS和转子温度必须低于60°C；
- 运动范围受实测硬端点及额外0.10 rad检查范围约束；
- 10秒动作超时；
- RS485 STOP可中断运动；
- 反馈丢失、故障、越界、超速或过温立即中止；
- 所有退出路径最后发送失能，并读取状态确认失能；
- 网络协议、命令白名单和guard byte不变，不开放任意CAN透传或任意位置目标。

实际循环包含CAN发送、逐步反馈等待和USB诊断输出，且实测速度仍受3 rad/s硬检查限制。最终1.3双向全行程均约2.3秒；没有取消任何反馈或最终失能校验。

## 构建产物

PlatformIO平台：`espressif32@6.11.0`，板型：`esp32dev`，构建日期：2026-11-10。

快速版：

- 环境：`esp32dev`
- 文件：`firmware/tcan485_dm4310_read_probe/.pio/build/esp32dev/firmware.bin`
- 大小：286128 bytes
- SHA-256：`747CADB19C1E684904B20B001CE12E11ADD643FCEE5EBE37F70EFE924CE2BE4B`
- 内嵌标识：`T-CAN485 DM4310 guarded gateway build 1.3 guarded-fast`

回退版：

- 环境：`esp32dev_legacy`
- 文件：`firmware/tcan485_dm4310_read_probe/.pio/build/esp32dev_legacy/firmware.bin`
- 大小：286128 bytes
- SHA-256：`A123445B0954E3E181A04D9059F25D432F6BA7C0F643B2A62762648B283B44B8`
- 内嵌标识：`T-CAN485 DM4310 guarded gateway build 1.1 rollback`

两种环境均构建成功。默认环境仅为 `esp32dev`，避免不带 `-e` 时误上传回退版。

重新构建命令：

```powershell
.\.venv-platformio\Scripts\pio.exe run `
  -d .\firmware\tcan485_dm4310_read_probe -e esp32dev

.\.venv-platformio\Scripts\pio.exe run `
  -d .\firmware\tcan485_dm4310_read_probe -e esp32dev_legacy
```

## 已执行的受保护上传流程

1. 用 `python -m serial.tools.list_ports -v` 确认网关CH9102串口；不要根据旧端口号盲刷。
2. 确认该端口不是 U-Arm 使用的 VID:PID `1A86:55D3` CH343 适配器。
3. 让夹爪周围空置，先用TCP60000读取DM4310状态并确认失能、无故障、静止、温度正常。
4. 明确指定快速环境和实际网关串口上传：

```powershell
.\.venv-platformio\Scripts\pio.exe run `
  -d .\firmware\tcan485_dm4310_read_probe `
  -e esp32dev -t upload --upload-port COM6
```

5. 通过网络identity诊断字段确认`1.3`；随后先做TCP60000只读状态查询。
6. 空载执行一次全开和一次全闭，记录时间、最大反馈速度、温度、端点误差和最终失能状态。
7. 任一检查失败立即刷回 `esp32dev_legacy` 环境，不提高步长或取消保护。

本流程已完成。刷写日志：`logs/gripper_gateway_flash_20261110_234142.json`；动作日志：`logs/gripper_open_20261110_234153.json`、`logs/gripper_close_20261110_234201.json`。
