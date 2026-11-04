/**
 * @file    cpr_config.h
 * @brief   System & Hardware Configuration for CPR BLE Monitor (Zephyr RTOS, No LED)
 * @version 9.0.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define FW_VERSION_STR       "9.0.0-ZEPHYR-NO-LED"
#define FW_PROJECT_NAME      "CPR_BLE_Zephyr"

/* =============================================================================
 * HARDWARE GPIO ASSIGNMENTS (ESP32-C3 Super Mini)
 * ============================================================================= */
#define HAL_PIN_AED_CHEST       0   /* ADC1_CH0 (49E Hall Sensor: Chest A2) */
#define HAL_PIN_AED_HIP         1   /* ADC1_CH1 (49E Hall Sensor: Hip A1) */
#define HAL_PIN_BATTERY_ADC     2   /* ADC1_CH2 (Battery 2:1 Divider) */
#define HAL_PIN_IR1             3   /* Optical IR Shallow Sensor (Active-LOW, Pull-up) */
#define HAL_PIN_IR2             4   /* Optical IR Deep Sensor (Active-LOW, Pull-up) */
#define HAL_PIN_MOTOR           6   /* Haptic PWM (200 Hz LEDC PWM) */
#define HAL_PIN_I2C_SDA         8   /* I2C SDA for BMP280 Airway Breath Sensor */
#define HAL_PIN_I2C_SCL         9   /* I2C SCL for BMP280 Airway Breath Sensor */

#define MOTOR_PWM_FREQ_HZ       200
#define MOTOR_DUTY_MAX_PERCENT  100

/* =============================================================================
 * BATTERY MONITORING CONFIGURATION
 * ============================================================================= */
#define BATT_DIVIDER_RATIO      2.0f
#define BATT_MIN_VOLTAGE_MV     3300   /* 3.3V = 0% */
#define BATT_MAX_VOLTAGE_MV     4200   /* 4.2V = 100% */
#define BATT_LOW_THRESHOLD_PCT  30

/* =============================================================================
 * BLE GATT & CHARACTERISTIC UUIDS (Nordic UART Service - NUS)
 * ============================================================================= */
#define BLE_DEVICE_NAME         "CPR_106"
#define MANIKIN_NAME_STR        "106"
#define MANIKIN_TYPE_DEFAULT    1

/* Nordic UART Service 128-bit UUIDs */
/* 6E400001-B5A3-F393-E0A9-E50E24DCCA9E */
#define BT_UUID_NUS_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x6e400001, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)

/* 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (RX: Write) */
#define BT_UUID_NUS_RX_VAL \
    BT_UUID_128_ENCODE(0x6e400002, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)

/* 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (TX: Notify/Read) */
#define BT_UUID_NUS_TX_VAL \
    BT_UUID_128_ENCODE(0x6e400003, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)

/* =============================================================================
 * TIMING CONSTANTS (Milliseconds)
 * ============================================================================= */
#define TIMING_SENSOR_POLL_MS    1      /* 1 kHz sampling for zero-delay response */
#define TIMING_DEBOUNCE_US       20     /* 20us optical glitch filter */
#define TIMING_IDLE_CONFIRM_MS   2
#define TIMING_STABLE_OFF_MS     2
#define TIMING_LUB_ON_MS         140    /* Systole pulse */
#define TIMING_LUB_OFF_MS        100    /* Diastolic gap */
#define TIMING_DUB_ON_MS         90     /* Diastole pulse */
#define TIMING_BEAT_TOTAL_MS     (TIMING_LUB_ON_MS + TIMING_LUB_OFF_MS + TIMING_DUB_ON_MS) /* 330ms */
#define TIMING_HEART_BPM_NORMAL  72

/* =============================================================================
 * AED PAD DETECTION CONFIGURATION (49E Linear Hall Sensors)
 * ============================================================================= */
#define AED_ADC_CH_CHEST         0      /* ADC1_CH0 (GPIO 0 - Chest Pad A2) */
#define AED_ADC_CH_HIP           1      /* ADC1_CH1 (GPIO 1 - Hip Pad A1) */
#define AED_ADC_RESOLUTION_BITS  12     /* 0..4095 */
#define AED_CALIBRATION_SAMPLES  50     /* Baseline sample count (precise zero reference) */
#define AED_SAMPLE_INTERVAL_MS   100    /* 10 Hz sampling when active */
#define AED_DIFF_THRESHOLD       6      /* High-sensitivity threshold for thin/small magnets */
#define AED_HYSTERESIS_THRESHOLD 3      /* Release threshold to prevent flapping */
#define AED_OVERSAMPLE_COUNT     16     /* 16-sample averaging filter for low-noise ADC */

/* =============================================================================
 * MOUTH AIR / BREATH DETECTION CONFIGURATION (BMP280 I2C on GPIO 8/9)
 * ============================================================================= */
#define AIR_ON_THRESHOLD_PA      15.0f  /* 15.0 Pa threshold for breath detection */
#define AIR_OFF_THRESHOLD_PA     8.0f   /* 8.0 Pa release threshold (hysteresis) */
#define AIR_FILTER_ALPHA         0.35f  /* Exponential moving average alpha */
#define AIR_ON_CONFIRM_COUNT     5      /* Consecutive samples required for ON */
#define AIR_OFF_CONFIRM_COUNT    6      /* Consecutive samples required for OFF */
#define AIR_CALIBRATION_SAMPLES  50     /* Baseline calibration sample count */
#define AIR_SAMPLE_INTERVAL_MS   20     /* 50 Hz sampling task period (20 ms) */

/* =============================================================================
 * CPM (COMPRESSIONS PER MINUTE) RATE ESTIMATION
 * ============================================================================= */
#define CPM_SAMPLE_COUNT         6
#define CPM_LOW_THRESHOLD        100
#define CPM_HIGH_THRESHOLD       120

/* =============================================================================
 * PROTOCOL CONSTANTS & ENUMS
 * ============================================================================= */
#define PACKET_MAGIC_0           0xC9
#define PACKET_MAGIC_1           0xC2
#define PROTOCOL_VER             0x06
#define PTYPE_ANNOUNCE           0x01
#define PTYPE_COMPRESSION        0x02
#define PTYPE_AED                0x04
#define PTYPE_AIR                0x05

typedef enum {
    DEPTH_FAILED_RECOIL = -1,
    DEPTH_IDLE          =  0,
    DEPTH_SHALLOW       =  1,
    DEPTH_GOOD          =  2
} depth_result_t;

typedef enum {
    AED_MODE_DISABLED = 0, /* A0: Deactivate AED */
    AED_MODE_HIP      = 1, /* A1: Hip AED Pad */
    AED_MODE_CHEST    = 2  /* A2: Chest AED Pad */
} aed_mode_t;

typedef enum {
    HEART_STOPPED     = 0,
    HEART_NORMAL      = 1,
    HEART_CONTINUOUS  = 2,
    HEART_SINGLE_BEAT = 3,
    HEART_MODE_MAX
} heart_mode_t;

typedef enum {
    PHASE_IDLE    = 0,
    PHASE_DESCENT = 1,
    PHASE_ASCENT  = 2,
    PHASE_FAULT   = 3
} seq_phase_t;

typedef enum {
    PEAK_NONE    = 0,
    PEAK_SHALLOW = 1,
    PEAK_GOOD    = 2
} peak_level_t;

/* =============================================================================
 * BINARY PACKET DEFINITIONS (Standard 15 & 20 Bytes)
 * ============================================================================= */
#pragma pack(push, 1)

typedef struct {
    uint8_t  magic[2];       /* [0..1]   0xC9, 0xC2 */
    uint8_t  version;        /* [2]      0x06 */
    uint8_t  type;           /* [3]      0x02 (COMPRESSION), 0x04 (AED), 0x05 (AIR) */
    uint32_t timestamp;      /* [4..7]   Little-endian milliseconds */
    uint16_t id;             /* [8..9]   Rolling packet index */
    int8_t   depth;          /* [10]     depth result or status code */
    uint8_t  deviceID[3];    /* [11..13] MAC bytes [3..5] */
    uint8_t  checksum;       /* [14]     XOR of bytes [0..13] */
} CprPacket;

typedef struct {
    uint8_t  magic[2];       /* [0..1]   0xC9, 0xC2 */
    uint8_t  version;        /* [2]      0x06 */
    uint8_t  type;           /* [3]      0x01 (ANNOUNCE) */
    uint32_t timestamp;      /* [4..7]   Little-endian milliseconds */
    uint16_t id;             /* [8..9]   Rolling packet index */
    int8_t   depth;          /* [10]     Battery Percentage */
    uint8_t  deviceID[3];    /* [11..13] MAC bytes [3..5] */
    char     manikinName[4]; /* [14..17] "106\0" */
    uint8_t  manikinType;    /* [18]     1 */
    uint8_t  checksum;       /* [19]     XOR of bytes [0..18] */
} AnnouncePacket;

#pragma pack(pop)

