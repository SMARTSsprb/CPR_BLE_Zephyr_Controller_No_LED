# CPR BLE Zephyr RTOS Controller (No LED Build)

Ultra-low-latency, deterministic CPR Training Monitor firmware running on **Zephyr RTOS v4.x** for the ESP32-C3 Super Mini.

---

## 1. Hardware Pinout (ESP32-C3 Super Mini)

| Pin | Devicetree Node | Type | Hardware Mapping |
| :--- | :--- | :--- | :--- |
| **GPIO 0** | `adc0` (channel@0) | `ADC1_CH0` | 49E Linear Hall Sensor: **Chest AED Pad** (`A2`) |
| **GPIO 1** | `adc0` (channel@1) | `ADC1_CH1` | 49E Linear Hall Sensor: **Hip AED Pad** (`A1`) |
| **GPIO 2** | `adc0` (channel@2) | `ADC1_CH2` | Battery Voltage Divider (2:1, 100kΩ/100kΩ) |
| **GPIO 3** | `ir1_pin` | `gpio-keys` | Optical IR Shallow Sensor (Active-LOW) |
| **GPIO 4** | `ir2_pin` | `gpio-keys` | Optical IR Good Depth Sensor (Active-LOW) |
| **GPIO 6** | `motor_pwm_0` | `ledc0` PWM | 200 Hz Haptic Motor PWM for Lub-Dub heartbeats |
| **GPIO 8** | `i2c0` (SDA) | I2C Bus | Mouth Airway Pressure / Breath Sensor (BMP280 / HW-611) |
| **GPIO 9** | `i2c0` (SCL) | I2C Bus | Mouth Airway Pressure / Breath Sensor (BMP280 / HW-611) |

---

## 2. Dedicated Sensor Channels

- **Chest AED (`A2`)**: Monitored strictly on **GPIO 0 (`ADC1_CH0`)**.
- **Hip AED (`A1`)**: Monitored strictly on **GPIO 1 (`ADC1_CH1`)**.
- **Breath Detection (`M1`/`M0`)**: Monitored on **I2C0 (GPIO 8 SDA, GPIO 9 SCL)** via BMP280 barometric pressure sensor.

---

## 3. Flashing Precompiled Zephyr Binary

The precompiled, verified binary is located at `bin/zephyr.bin`:

```powershell
# Flash to COM7 (or your active ESP32-C3 port)
esptool --chip esp32c3 -p COM7 -b 921600 write_flash 0x0 bin\zephyr.bin

# Monitor serial output
python -m serial.tools.miniterm COM7 115200
```
