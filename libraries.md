# Libraries for `self_balancing_robot.ino`

## Summary

**No external libraries need to be installed.** Every library the sketch uses ships with the ESP32 board package. The MPU6050 is read directly over I2C, so no MPU6050 library is needed.

## Libraries used

| Library | Used for | Install |
|---|---|---|
| `Wire.h` | I2C communication with the MPU6050 (SDA 21, SCL 22) | Built into the ESP32 core |
| `WiFi.h` | Connecting the ESP32 to your 2.4 GHz WiFi | Built into the ESP32 core |
| `WebServer.h` | Hosting the PID tuning page in the browser | Built into the ESP32 core |
| `Preferences.h` | "Save to flash" (stores PID values in permanent memory) | Built into the ESP32 core |

PWM for the L293D enable pins uses the core's built-in `ledcAttach()` / `ledcWrite()`. No library is needed for that either.

## What you do need

### 1. Arduino IDE
Version 2.x is recommended: https://www.arduino.cc/en/software

### 2. ESP32 board package (by Espressif Systems), version 3.x
1. Open **File → Preferences**.
2. In **Additional boards manager URLs**, add:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. Open **Tools → Board → Boards Manager**, search for **esp32**, and install **esp32 by Espressif Systems**.
4. Make sure the installed version is **3.0.0 or newer**. The sketch uses `ledcAttach(pin, freq, resolution)`, which only exists in core 3.x.

### 3. Board settings
| Setting | Value |
|---|---|
| Board | **ESP32 Dev Module** |
| Upload speed | 115200 (use this if uploads fail at higher speeds) |
| Port | The COM port that appears when the ESP32 is plugged in |
| Serial Monitor baud | **115200** |

## Troubleshooting

- **Error: `'ledcAttach' was not declared`**: your ESP32 core is version 2.x. Update it to 3.x in Boards Manager.
- **Upload stuck at "Connecting..."**: hold the **BOOT** button on the ESP32 while it connects.
- **No COM port appears**: use a USB data cable (some cables only charge), and install the CP210x or CH340 driver if your board needs one.
- **WiFi will not connect**: the ESP32 supports only **2.4 GHz** WiFi. Check `WIFI_SSID` and `WIFI_PASS` at the top of the sketch.
