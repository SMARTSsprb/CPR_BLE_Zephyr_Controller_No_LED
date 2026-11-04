/**
 * @file    haptic.c
 * @brief   Haptic Motor Driver Implementation with Zephyr PWM (Lub-Dub Engine)
 * @version 9.0.0
 */

#include "haptic.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(haptic, LOG_LEVEL_INF);

static const struct device *gpio_dev = NULL;
static const struct pwm_dt_spec pwm_motor = PWM_DT_SPEC_GET_OR(DT_ALIAS(pwm_motor), {});
static bool s_use_pwm = false;

static struct k_mutex s_haptic_mutex;
static volatile heart_mode_t s_heart_mode = HEART_STOPPED;

int haptic_init(void)
{
    k_mutex_init(&s_haptic_mutex);

    if (pwm_is_ready_dt(&pwm_motor)) {
        s_use_pwm = true;
        LOG_INF("Haptic motor configured via PWM DT spec (200 Hz)");
    } else {
        /* Fallback: Direct GPIO output on GPIO 6 */
        gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
        if (device_is_ready(gpio_dev)) {
            gpio_pin_configure(gpio_dev, HAL_PIN_MOTOR, GPIO_OUTPUT_INACTIVE);
            LOG_INF("Haptic motor fallback initialized on GPIO %d", HAL_PIN_MOTOR);
        } else {
            LOG_ERR("No valid device for haptic motor");
            return -ENODEV;
        }
    }

    haptic_stop();
    return 0;
}

void haptic_set_duty(uint8_t percent)
{
    if (percent > 100) percent = 100;

    if (s_use_pwm) {
        uint32_t period_ns = pwm_motor.period;
        uint32_t pulse_ns = (period_ns * percent) / 100;
        pwm_set_dt(&pwm_motor, period_ns, pulse_ns);
    } else if (gpio_dev) {
        gpio_pin_set_raw(gpio_dev, HAL_PIN_MOTOR, (percent > 0) ? 1 : 0);
    }
}

void haptic_stop(void)
{
    haptic_set_duty(0);
}

void haptic_set_mode(heart_mode_t mode)
{
    k_mutex_lock(&s_haptic_mutex, K_FOREVER);
    s_heart_mode = mode;
    k_mutex_unlock(&s_haptic_mutex);
}

heart_mode_t haptic_get_mode(void)
{
    heart_mode_t m;
    k_mutex_lock(&s_haptic_mutex, K_FOREVER);
    m = s_heart_mode;
    k_mutex_unlock(&s_haptic_mutex);
    return m;
}

void haptic_run_cycle(void)
{
    heart_mode_t mode = haptic_get_mode();

    if (mode == HEART_STOPPED) {
        haptic_stop();
        k_msleep(100);
        return;
    }

    if (mode == HEART_CONTINUOUS) {
        haptic_set_duty(MOTOR_DUTY_MAX_PERCENT);
        k_msleep(50);
        return;
    }

    if (mode == HEART_SINGLE_BEAT) {
        /* Exact 1-shot Lub-Dub pulse */
        haptic_set_duty(MOTOR_DUTY_MAX_PERCENT);
        k_msleep(TIMING_LUB_ON_MS);

        haptic_stop();
        k_msleep(TIMING_LUB_OFF_MS);

        haptic_set_duty(MOTOR_DUTY_MAX_PERCENT);
        k_msleep(TIMING_DUB_ON_MS);

        haptic_stop();
        haptic_set_mode(HEART_STOPPED);
        return;
    }

    /* HEART_NORMAL: 72 BPM repeating rhythm */
    const uint32_t interval_ms = (60000UL / TIMING_HEART_BPM_NORMAL);

    haptic_set_duty(MOTOR_DUTY_MAX_PERCENT);
    k_msleep(TIMING_LUB_ON_MS);

    haptic_stop();
    k_msleep(TIMING_LUB_OFF_MS);

    haptic_set_duty(MOTOR_DUTY_MAX_PERCENT);
    k_msleep(TIMING_DUB_ON_MS);

    haptic_stop();

    uint32_t rest_ms = (interval_ms > TIMING_BEAT_TOTAL_MS) ?
                       (interval_ms - TIMING_BEAT_TOTAL_MS) : 0;
    if (rest_ms > 0) {
        k_msleep(rest_ms);
    }
}

