/**
 * @file    ble_service.h
 * @brief   Zephyr BLE GATT Nordic UART Service (NUS) & Battery Service (BAS)
 * @version 9.0.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "cpr_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ble_heart_mode_callback_t)(heart_mode_t mode);
typedef void (*ble_announce_request_callback_t)(void);
typedef void (*ble_aed_mode_callback_t)(aed_mode_t mode);
typedef void (*ble_air_mode_callback_t)(bool active);

void ble_service_register_callbacks(ble_heart_mode_callback_t heart_cb,
                                    ble_announce_request_callback_t announce_cb,
                                    ble_aed_mode_callback_t aed_cb,
                                    ble_air_mode_callback_t air_cb);

int ble_service_init(void);

bool ble_service_is_connected(void);
bool ble_service_is_advertising(void);
int ble_service_start_advertising(void);
void ble_service_ensure_advertising(void);

int ble_service_send_announce(const AnnouncePacket *pkt);

int ble_service_send_compression(const CprPacket *pkt);

int ble_service_send_aed(const CprPacket *pkt);

int ble_service_send_air(const CprPacket *pkt);

void ble_service_set_battery_level(uint8_t percentage);

#ifdef __cplusplus
}
#endif

