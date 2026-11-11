# T-CAN485 / DM-J4310-2EC V1.2 gateway

This first-stage firmware is intentionally passive:

- CAN: 1 Mbit/s, standard/extended frames accepted, ESP32 TWAI listen-only mode.
- CAN transmit queue: disabled; no call to `twai_transmit` exists.
- RS485: 115200 8N1, received bytes are printed to USB serial.
- USB log: 115200 baud on COM6.

It replaces the LILYGO factory test firmware, which initializes CAN at 500 kbit/s
and includes a periodic test transmitter. This build verifies wiring and the
CR5AF-to-RS485 receive path before CAN transmission is enabled.

Build and upload from the repository root:

```powershell
.\.venv-platformio\Scripts\pio.exe run -d firmware\tcan485_dm4310_gateway
.\.venv-platformio\Scripts\pio.exe run -d firmware\tcan485_dm4310_gateway -t upload
```
