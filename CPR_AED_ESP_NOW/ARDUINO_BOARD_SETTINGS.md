# Arduino IDE Board Settings — CPR AED ESP-NOW

## Board Package
Install via **Boards Manager**: `esp32` by Espressif Systems (≥ v2.0.11)

## Board Settings

| Setting              | Value                       |
|----------------------|-----------------------------|
| Board                | **ESP32C3 Dev Module**       |
| USB CDC On Boot      | **Enabled**                 |
| CPU Frequency        | 160 MHz                     |
| Flash Frequency      | 80 MHz                      |
| Flash Mode           | QIO                         |
| Flash Size           | **4MB (32Mb)**              |
| Partition Scheme     | Default 4MB with spiffs     |
| Upload Speed         | 921600                      |
| Port                 | COM8 (or whichever shows)   |

> **Note:** "USB CDC On Boot → Enabled" is required for `Serial.println()` to appear
> over the USB-C port on the ESP32-C3 Super Mini without a separate USB-to-UART chip.

## Libraries Required
No external libraries needed — `esp_now.h` and `WiFi.h` are bundled with the esp32 package.

## Flashing Steps
1. Open `CPR_AED_ESP_NOW.ino` in Arduino IDE 2.x
2. Set board to **ESP32C3 Dev Module** and correct COM port
3. Click **Upload** (Ctrl+U)
4. Open **Serial Monitor** at **115200 baud** to view logs

## Serial Log to Verify
On first boot you should see:
```
══════════════════════════════════════════════════════
 CPR External AED Unit v2.0.0 [Arduino / ESP-NOW]
 Target: ESP32-C3 Super Mini
 Protocol: ESP-NOW ONLY (No BLE)
══════════════════════════════════════════════════════
[MCU] AED Wi-Fi STA MAC : XX:XX:XX:XX:XX:XX
[MCU] Target Manikin MAC: E8:3D:C1:82:1B:F0
[MCU] ESP-NOW Channel   : 1
[ESP-NOW] Initialized. Peer registered.
[AED] Boot complete. Sending AED_ON. Waiting for shock button...
```
