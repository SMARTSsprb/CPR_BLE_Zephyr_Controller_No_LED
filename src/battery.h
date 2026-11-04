/**
 * @file    battery.h
 * @brief   Battery ADC Voltage & Percentage Monitor (Zephyr RTOS)
 * @version 9.0.0
 */

#pragma once

#include <stdint.h>
#include "cpr_config.h"

#ifdef __cplusplus
extern "C" {
#endif

int battery_init(void);

uint16_t battery_get_voltage_mv(void);

uint8_t battery_get_percentage(void);

#ifdef __cplusplus
}
#endif

