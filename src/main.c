/**
 * @file    main.c
 * @brief   CPR Training Monitor — Main Application Entry Point (Zephyr RTOS, No LED)
 * @version 9.0.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>

#include "cpr_config.h"
#include "cpr_protocol.h"
#include "ble_service.h"
#include "ir_sensor.h"
#include "haptic.h"
#include "aed_sensor.h"
#include "air_sensor.h"
#include "battery.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Thread Stack Sizes & Priorities */
#define STACK_SIZE_SENSOR   2048
#define STACK_SIZE_HEART    1024
#define STACK_SIZE_AED      1536
#define STACK_SIZE_AIR      2048

#define PRIORITY_SENSOR     2   /* High priority for deterministic 1 kHz optical sampling */
#define PRIORITY_HEART      5   /* Background rhythmic haptic driver */
#define PRIORITY_AED        6   /* 10 Hz telemetry */
#define PRIORITY_AIR        6   /* 50 Hz pressure telemetry */

static uint8_t  s_device_id[3]   = {0x82, 0x30, 0x9A}; /* Derived from BT MAC */
static uint16_t s_packet_counter = 0;
static int8_t   s_last_aed_state = -1;
static volatile bool s_announce_pending = false;

/* =============================================================================
 * SENSOR & BLE EVENT HANDLERS
 * ============================================================================= */
static void on_compression_event(depth_result_t result)
{
    if (ble_service_is_connected()) {
        CprPacket cp = cpr_protocol_build_packet(
            PTYPE_COMPRESSION,
            (int8_t)result,
            s_packet_counter++,
            s_device_id
        );
        ble_service_send_compression(&cp);
    }
}

static void on_recoil_event(depth_result_t result)
{
    if (ble_service_is_connected()) {
        CprPacket cp = cpr_protocol_build_packet(
            PTYPE_COMPRESSION,
            (int8_t)result,
            s_packet_counter++,
            s_device_id
        );
        ble_service_send_compression(&cp);
    }
}

static void on_ble_heart_mode_cmd(heart_mode_t mode)
{
    haptic_set_mode(mode);
}

static void on_ble_announce_cmd(void)
{
    s_announce_pending = true;
}

static void on_ble_aed_mode_cmd(aed_mode_t mode)
{
    aed_sensor_set_mode(mode);
    s_last_aed_state = -1;
}

static void on_ble_air_mode_cmd(bool active)
{
    air_sensor_set_active(active);
}

/* =============================================================================
 * ZEPHYR RTOS DEDICATED THREADS
 * ============================================================================= */

/* 1. Sensor Thread: Deterministic 1 kHz Sampling (Zero-Delay Optical Trigger) */
static void sensor_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        ir_sensor_update_tick(TIMING_SENSOR_POLL_MS);
        k_msleep(TIMING_SENSOR_POLL_MS);
    }
}
K_THREAD_DEFINE(sensor_tid, STACK_SIZE_SENSOR, sensor_thread_entry, NULL, NULL, NULL,
                PRIORITY_SENSOR, 0, 0);

/* 2. Heartbeat Thread: Precise Lub-Dub Cardiac Motor Rhythms */
static void heart_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        haptic_run_cycle();
    }
}
K_THREAD_DEFINE(heart_tid, STACK_SIZE_HEART, heart_thread_entry, NULL, NULL, NULL,
                PRIORITY_HEART, 0, 0);

/* 3. AED Thread: 10 Hz 49E Hall Effect Pad Detection (Chest A2 & Hip A1) */
static void aed_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        if (aed_sensor_is_active()) {
            aed_snapshot_t aed = aed_sensor_read();
            int8_t state = aed.is_attached ? 1 : 0;
            if (state != s_last_aed_state) {
                s_last_aed_state = state;
                if (ble_service_is_connected()) {
                    CprPacket p = cpr_protocol_build_packet(
                        PTYPE_AED,
                        state,
                        s_packet_counter++,
                        s_device_id
                    );
                    ble_service_send_aed(&p);
                    LOG_INF("[AED Telemetry] Mode=%s, Attached=%s (ADC=%d, Baseline=%d, Diff=%d)",
                            (aed.mode == AED_MODE_CHEST) ? "CHEST (A2)" : "HIP (A1)",
                            aed.is_attached ? "YES" : "NO",
                            aed.raw_adc,
                            aed.baseline,
                            aed.diff);
                }
            }
            k_msleep(AED_SAMPLE_INTERVAL_MS);
        } else {
            k_msleep(100);
        }
    }
}
K_THREAD_DEFINE(aed_tid, STACK_SIZE_AED, aed_thread_entry, NULL, NULL, NULL,
                PRIORITY_AED, 0, 0);

/* 4. Air Thread: 50 Hz Digital Breath Detection on GPIO 1 */
static void air_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        if (air_sensor_is_active()) {
            bool is_air = false;
            if (air_sensor_update(&is_air)) {
                if (ble_service_is_connected()) {
                    CprPacket p = cpr_protocol_build_packet(
                        PTYPE_AIR,
                        is_air ? 1 : 0,
                        s_packet_counter++,
                        s_device_id
                    );
                    ble_service_send_air(&p);
                    LOG_INF("[Breath Telemetry] Breath=%s", is_air ? "DETECTED (ON)" : "RELEASED (OFF)");
                }
            }
            k_msleep(AIR_SAMPLE_INTERVAL_MS);
        } else {
            k_msleep(100);
        }
    }
}
K_THREAD_DEFINE(air_tid, STACK_SIZE_AIR, air_thread_entry, NULL, NULL, NULL,
                PRIORITY_AIR, 0, 0);

/* =============================================================================
 * MAIN ENTRY POINT
 * ============================================================================= */
int main(void)
{
    LOG_INF("==================================================");
    LOG_INF(" CPR Monitor v%s [Zephyr RTOS No-LED]", FW_VERSION_STR);
    LOG_INF("==================================================");

    /* 1. Initialize Bare-Metal Drivers */
    ir_sensor_init();
    haptic_init();
    aed_sensor_init();
    air_sensor_init();
    battery_init();

    /* 2. Register Callback Handlers */
    ir_sensor_register_callbacks(on_compression_event, on_recoil_event);
    ble_service_register_callbacks(
        on_ble_heart_mode_cmd,
        on_ble_announce_cmd,
        on_ble_aed_mode_cmd,
        on_ble_air_mode_cmd
    );

    /* 3. Initialize BLE GATT & Advertising */
    int ret = ble_service_init();
    if (ret) {
        LOG_ERR("BLE initialization failed (err %d)", ret);
    }

    LOG_INF("System boot sequence complete. Active threads: Sensor, Heart, AED, Air.");

    /* 4. Main Telemetry & Watchdog Loop (Every 5 seconds) */
    while (1) {
        uint8_t batt_pct = battery_get_percentage();
        ble_service_set_battery_level(batt_pct);

        if (s_announce_pending) {
            s_announce_pending = false;
            if (ble_service_is_connected()) {
                AnnouncePacket ap = cpr_protocol_build_announce(
                    s_packet_counter++,
                    s_device_id,
                    batt_pct
                );
                ble_service_send_announce(&ap);
            }
        }

        /* Ensure advertising is active whenever disconnected */
        if (!ble_service_is_connected()) {
            ble_service_ensure_advertising();
        }

        LOG_INF("[HEALTH] State: %s | Battery: %u%% | CPM: %u",
                ble_service_is_connected() ? "CONNECTED" : 
                (ble_service_is_advertising() ? "ADVERTISING" : "IDLE"),
                batt_pct,
                cpm_tracker_get_current_rate());

        k_msleep(5000);
    }

    return 0;
}

