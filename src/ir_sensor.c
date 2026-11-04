/**
 * @file    ir_sensor.c
 * @brief   Optical IR Sensors & Sequence Tracker Implementation (Zephyr RTOS)
 * @version 9.0.0
 */

#include "ir_sensor.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ir_sensor, LOG_LEVEL_INF);

static const struct device *gpio_dev = NULL;

static compression_callback_t s_comp_cb   = NULL;
static recoil_callback_t      s_recoil_cb = NULL;

/* Sequence Tracker State */
static seq_phase_t  s_phase           = PHASE_IDLE;
static peak_level_t s_peak            = PEAK_NONE;
static uint32_t     s_idle_time_ms    = 0;
static uint32_t     s_ascent_time_ms  = 0;
static bool         s_ir2_was_on      = false;
static bool         s_ir2_went_off    = false;
static bool         s_instant_tripped = false;

/* CPM Tracker Rolling Window */
static uint32_t s_cpm_timestamps[CPM_SAMPLE_COUNT];
static uint8_t  s_cpm_count = 0;
static uint8_t  s_cpm_head  = 0;
static uint16_t s_current_cpm = 0;

int ir_sensor_init(void)
{
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO controller not ready");
        return -ENODEV;
    }

    /* Configure IR1 (GPIO 3) and IR2 (GPIO 4) as inputs with pullup */
    int ret;
    ret = gpio_pin_configure(gpio_dev, HAL_PIN_IR1, GPIO_INPUT | GPIO_PULL_UP);
    if (ret) {
        LOG_ERR("Failed to configure IR1 pin (err %d)", ret);
        return ret;
    }

    ret = gpio_pin_configure(gpio_dev, HAL_PIN_IR2, GPIO_INPUT | GPIO_PULL_UP);
    if (ret) {
        LOG_ERR("Failed to configure IR2 pin (err %d)", ret);
        return ret;
    }

    LOG_INF("IR Sensors Initialized: IR1 (GPIO %d), IR2 (GPIO %d)", HAL_PIN_IR1, HAL_PIN_IR2);
    return 0;
}

ir_snapshot_t ir_sensor_read_debounced(void)
{
    ir_snapshot_t snap;

    /* Read active-low sensors: 0 = beam broken (object present) */
    int r1_a = gpio_pin_get_raw(gpio_dev, HAL_PIN_IR1);
    int r2_a = gpio_pin_get_raw(gpio_dev, HAL_PIN_IR2);

    k_busy_wait(TIMING_DEBOUNCE_US);

    int r1_b = gpio_pin_get_raw(gpio_dev, HAL_PIN_IR1);
    int r2_b = gpio_pin_get_raw(gpio_dev, HAL_PIN_IR2);

    snap.ir1 = ((r1_a == 0) && (r1_b == 0));
    snap.ir2 = ((r2_a == 0) && (r2_b == 0));

    return snap;
}

void ir_sensor_register_callbacks(compression_callback_t comp_cb, recoil_callback_t recoil_cb)
{
    s_comp_cb   = comp_cb;
    s_recoil_cb = recoil_cb;
}

static void cpm_tracker_record(uint32_t now_ms)
{
    s_cpm_timestamps[s_cpm_head] = now_ms;
    s_cpm_head = (s_cpm_head + 1) % CPM_SAMPLE_COUNT;
    if (s_cpm_count < CPM_SAMPLE_COUNT) {
        s_cpm_count++;
    }

    if (s_cpm_count >= 2) {
        uint8_t oldest_idx = (s_cpm_head + CPM_SAMPLE_COUNT - s_cpm_count) % CPM_SAMPLE_COUNT;
        uint32_t dt = now_ms - s_cpm_timestamps[oldest_idx];
        if (dt > 0) {
            s_current_cpm = (uint16_t)(((uint64_t)(s_cpm_count - 1) * 60000ULL) / dt);
        }
    }
}

uint16_t cpm_tracker_get_current_rate(void)
{
    return s_current_cpm;
}

void ir_sensor_update_tick(uint32_t dt_ms)
{
    ir_snapshot_t snap = ir_sensor_read_debounced();
    uint32_t now_ms = k_uptime_get_32();

    switch (s_phase) {
        case PHASE_IDLE:
            if (snap.ir1) {
                s_phase           = PHASE_DESCENT;
                s_peak            = PEAK_SHALLOW;
                s_ir2_was_on      = false;
                s_ir2_went_off    = false;
                s_instant_tripped = false;
                s_idle_time_ms    = 0;
            }
            break;

        case PHASE_DESCENT:
            if (snap.ir2) {
                s_peak = PEAK_GOOD;
                s_ir2_was_on = true;

                /* INSTANT TRIGGER: Zero-delay BLE notification on IR2 trip (<0.05ms) */
                if (!s_instant_tripped) {
                    s_instant_tripped = true;
                    cpm_tracker_record(now_ms);
                    if (s_comp_cb) {
                        s_comp_cb(DEPTH_GOOD);
                    }
                }
            }

            /* Detect start of chest rise */
            bool start_ascent = false;
            if (s_peak == PEAK_GOOD && !snap.ir2) {
                start_ascent = true;
                s_ir2_went_off = true;
            } else if (s_peak == PEAK_SHALLOW && !snap.ir1) {
                start_ascent = true;
            }

            if (start_ascent) {
                s_phase          = PHASE_ASCENT;
                s_ascent_time_ms = 0;
            }
            break;

        case PHASE_ASCENT:
            s_ascent_time_ms += dt_ms;

            if (s_peak == PEAK_GOOD && snap.ir2) {
                s_ir2_went_off = false;
            }

            /* Full return to neutral rest position */
            if (!snap.ir1 && !snap.ir2) {
                s_idle_time_ms += dt_ms;
                if (s_idle_time_ms >= TIMING_IDLE_CONFIRM_MS) {
                    /* Compression cycle complete */
                    if (s_peak == PEAK_GOOD) {
                        if (s_ir2_went_off) {
                            if (s_recoil_cb) s_recoil_cb(DEPTH_IDLE);
                        } else {
                            if (s_recoil_cb) s_recoil_cb(DEPTH_FAILED_RECOIL);
                        }
                    } else if (s_peak == PEAK_SHALLOW) {
                        cpm_tracker_record(now_ms);
                        if (s_comp_cb) s_comp_cb(DEPTH_SHALLOW);
                        if (s_recoil_cb) s_recoil_cb(DEPTH_IDLE);
                    }

                    s_phase = PHASE_IDLE;
                    s_peak = PEAK_NONE;
                    s_idle_time_ms = 0;
                }
            } else {
                s_idle_time_ms = 0;

                /* Re-compression before full recoil */
                if (s_ascent_time_ms > 200 && snap.ir1) {
                    if (s_recoil_cb) {
                        s_recoil_cb(DEPTH_FAILED_RECOIL);
                    }
                    s_phase = PHASE_DESCENT;
                    s_peak = snap.ir2 ? PEAK_GOOD : PEAK_SHALLOW;
                    s_instant_tripped = snap.ir2;
                }
            }
            break;

        case PHASE_FAULT:
            s_phase = PHASE_IDLE;
            s_peak = PEAK_NONE;
            break;
    }
}

