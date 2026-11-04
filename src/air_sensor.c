/**
 * @file    air_sensor.c
 * @brief   BMP280 Airway Barometric Pressure Sensor Driver Implementation (Zephyr RTOS)
 *          Operates over I2C0 (SDA: GPIO 8, SCL: GPIO 9).
 * @version 9.3.0
 */

#include "air_sensor.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(air_sensor, LOG_LEVEL_INF);

static const struct device *i2c_dev = NULL;
static uint16_t s_bmp_addr          = 0x76;
static bool     s_bmp_initialized   = false;
static bool     s_air_active        = false;

/* Calibration Parameters */
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t  t_fine;

/* Air Detection State */
static float   s_baseline_pressure = 0.0f;
static float   s_filtered_diff     = 0.0f;
static bool    s_first_reading     = true;
static bool    s_air_detected      = false;
static uint8_t s_on_count          = 0;
static uint8_t s_off_count         = 0;

/* BMP280 Registers */
#define BMP280_REG_CHIPID       0xD0
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_CALIB_START  0x88

static int bmp280_read_calib(void)
{
    uint8_t calib[24];
    int ret = i2c_burst_read(i2c_dev, s_bmp_addr, BMP280_REG_CALIB_START, calib, 24);
    if (ret) return ret;

    dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);

    dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);

    return 0;
}

static float bmp280_read_pressure_pa(void)
{
    if (!s_bmp_initialized) return 101325.0f;

    uint8_t data[6];
    int ret = i2c_burst_read(i2c_dev, s_bmp_addr, BMP280_REG_PRESS_MSB, data, 6);
    if (ret) return 101325.0f;

    int32_t adc_p = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    int32_t adc_t = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);

    /* Temperature compensation */
    int32_t var1 = ((((adc_t >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_t >> 4) - ((int32_t)dig_T1)) * ((adc_t >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;

    /* Pressure compensation */
    int64_t p_var1 = ((int64_t)t_fine) - 128000;
    int64_t p_var2 = p_var1 * p_var1 * (int64_t)dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3) >> 8) + ((p_var1 * (int64_t)dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)dig_P1) >> 33;

    if (p_var1 == 0) return 0.0f;

    int64_t p = 1048576 - adc_p;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dig_P7) << 4);

    return (float)p / 256.0f;
}

int air_sensor_init(void)
{
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(i2c_dev)) {
        LOG_WRN("I2C0 not ready");
        return -ENODEV;
    }

    uint8_t chip_id = 0;
    /* Probe 0x76 first, then 0x77 */
    s_bmp_addr = 0x76;
    if (i2c_reg_read_byte(i2c_dev, s_bmp_addr, BMP280_REG_CHIPID, &chip_id) != 0 || (chip_id != 0x58 && chip_id != 0x60)) {
        s_bmp_addr = 0x77;
        if (i2c_reg_read_byte(i2c_dev, s_bmp_addr, BMP280_REG_CHIPID, &chip_id) != 0 || (chip_id != 0x58 && chip_id != 0x60)) {
            LOG_WRN("BMP280 not found on I2C (0x76 / 0x77)");
            s_bmp_initialized = false;
            return -ENODEV;
        }
    }

    bmp280_read_calib();

    /* Put in sleep mode initially */
    i2c_reg_write_byte(i2c_dev, s_bmp_addr, BMP280_REG_CTRL_MEAS, 0x00);
    s_bmp_initialized = true;
    LOG_INF("BMP280 airway sensor initialized at 0x%02x on I2C0 (Sleep Mode)", s_bmp_addr);
    return 0;
}

bool air_sensor_calibrate(void)
{
    if (!s_bmp_initialized) return false;

    /* Configure normal mode, x2 temp oversample, x16 pressure oversample */
    i2c_reg_write_byte(i2c_dev, s_bmp_addr, BMP280_REG_CTRL_MEAS, 0x57);
    i2c_reg_write_byte(i2c_dev, s_bmp_addr, BMP280_REG_CONFIG, 0x10); /* Filter x16 */

    k_msleep(100);

    float sum = 0.0f;
    for (int i = 0; i < AIR_CALIBRATION_SAMPLES; ++i) {
        sum += bmp280_read_pressure_pa();
        k_msleep(20);
    }
    s_baseline_pressure = sum / (float)AIR_CALIBRATION_SAMPLES;
    s_first_reading     = true;
    s_air_detected      = false;
    s_on_count          = 0;
    s_off_count         = 0;

    LOG_INF("BMP280 Calibrated: Baseline Pressure = %.2f Pa", (double)s_baseline_pressure);
    return true;
}

void air_sensor_set_active(bool active)
{
    s_air_active = active;
    if (active) {
        air_sensor_calibrate();
        LOG_INF(">>> Mouth Airway Sensor ACTIVATED (Cmd 'M1')");
    } else {
        if (s_bmp_initialized) {
            i2c_reg_write_byte(i2c_dev, s_bmp_addr, BMP280_REG_CTRL_MEAS, 0x00); /* Sleep */
        }
        LOG_INF("<<< Mouth Airway Sensor DEACTIVATED (Cmd 'M0')");
    }
}

bool air_sensor_is_active(void)
{
    return s_air_active;
}

bool air_sensor_update(bool *out_air_detected)
{
    if (!s_air_active || !s_bmp_initialized) return false;

    float pressure = bmp280_read_pressure_pa();
    float diff = pressure - s_baseline_pressure;

    if (s_first_reading) {
        s_filtered_diff = diff;
        s_first_reading = false;
    } else {
        s_filtered_diff = AIR_FILTER_ALPHA * diff + (1.0f - AIR_FILTER_ALPHA) * s_filtered_diff;
    }

    float magnitude = fabsf(s_filtered_diff);
    bool state_changed = false;

    if (!s_air_detected) {
        if (magnitude >= AIR_ON_THRESHOLD_PA) {
            s_on_count++;
        } else {
            s_on_count = 0;
        }

        if (s_on_count >= AIR_ON_CONFIRM_COUNT) {
            s_air_detected = true;
            s_on_count     = 0;
            s_off_count    = 0;
            state_changed  = true;
            LOG_INF("[AIR EVENT] >>> BREATH DETECTED! (Delta: %.1f Pa)", (double)magnitude);
        }
    } else {
        if (magnitude <= AIR_OFF_THRESHOLD_PA) {
            s_off_count++;
        } else {
            s_off_count = 0;
        }

        if (s_off_count >= AIR_OFF_CONFIRM_COUNT) {
            s_air_detected = false;
            s_off_count    = 0;
            s_on_count     = 0;
            state_changed  = true;
            LOG_INF("[AIR EVENT] <<< BREATH ENDED");
        }
    }

    *out_air_detected = s_air_detected;
    return state_changed;
}
