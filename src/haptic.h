/**
 * @file    haptic.h
 * @brief   Haptic Motor Driver & Cardiac Rhythmic Pulse Engine (Zephyr RTOS)
 * @version 9.0.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "cpr_config.h"

#ifdef __cplusplus
extern "C" {
#endif

int haptic_init(void);

void haptic_set_mode(heart_mode_t mode);

heart_mode_t haptic_get_mode(void);

void haptic_set_duty(uint8_t percent);

void haptic_stop(void);

void haptic_run_cycle(void);

#ifdef __cplusplus
}
#endif

