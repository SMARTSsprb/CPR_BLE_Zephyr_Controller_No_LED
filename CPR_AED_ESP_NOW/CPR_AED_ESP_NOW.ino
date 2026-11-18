/**
 * @file    CPR_AED_ESP_NOW.ino
 * @brief   CPR External AED Shock Unit — Arduino Firmware (ESP-NOW Protocol)
 * @version 2.1.0
 * @target  ESP32-C3 (Arduino IDE, esp32 by Espressif board package)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  PROTOCOL: ESP-NOW ONLY (NO BLE)
 *  Sends 4-command packets to Manikin MCU:
 *    CMD 0 = AED_OFF   — AED unit powered off / deactivated
 *    CMD 1 = AED_ON    — AED unit powered on / activated
 *    CMD 2 = SHOCK_ON  — Shock sequence active (stage 1=alert, 2=delivering)
 *    CMD 3 = SHOCK_OFF — Shock sequence complete
 *
 *  RGB LED INDICATION SYSTEM (Visual Feedback Modes & Waveforms):
 *    MODE 01: WAITING FOR PEER  — Deep Blue breathing (1.8s breath cycle, 5% - 100%)
 *    MODE 02: PEER ACTIVE&READY — Emerald Green continuous steady glow (100% Constant Solid ON)
 *    MODE 03: SHOCK ON (ALERT)  — Warm Amber/Yellow one-time triple strobe burst (3x 60ms/60ms)
 *    MODE 04: SHOCK GIVING(ARM) — Urgent Red rapid hazard strobe (5 Hz / 100ms ON / 100ms OFF)
 *    MODE 05: SHOCK OFF (DELIV) — Pure Green 1.0s confirmation latch -> returns to Peer Active Glow
 *
 *  HARDWARE GPIO ASSIGNMENTS (ESP32-C3):
 *    Red: GPIO 3 | Green: GPIO 4 | Blue: GPIO 5 | Sync: GPIO 7 | Shock: GPIO 9
 *
 *  TARGET MANIKIN MAC: E8:3D:C1:82:1B:F0 (Wi-Fi STA MAC for ESP-NOW)
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ─────────────────────────────────────────────────────────────────────────────
// CONFIGURATION — Hardware pins & timing
// ─────────────────────────────────────────────────────────────────────────────
#define PIN_SHOCK_BUTTON       9    // Active-LOW shock button (BOOT pin or external)
#define PIN_POWER_SWITCH       6    // Power switch (Option 1: hardware cut)
#define PIN_STATUS_LED         8    // Active-LOW onboard blue LED
#define PIN_SYNC_OUT           7    // Hardware sync pulse output to Manikin

#define PIN_RGB_RED            3    // RGB Red channel
#define PIN_RGB_GREEN          4    // RGB Green channel
#define PIN_RGB_BLUE           5    // RGB Blue channel

// RGB polarity: 0 = Common Cathode (Active-HIGH), 1 = Common Anode (Active-LOW)
#define RGB_LED_ACTIVE_LOW     0

// Timing constants (ms)
#define T_BTN_DEBOUNCE_MS      50
#define T_SHOCK_COOLDOWN_MS    11000  // 11s lockout after full sequence
#define T_SHOCK_ALERT_STROBE_MS 60   // 60ms ON / 60ms OFF triple strobe
#define T_SHOCK_GIVING_MS      5000  // 5s Red hazard stage
#define T_SHOCK_OFF_LATCH_MS   1000  // 1s Green latch after shock done
#define T_SYNC_PULSE_US        1000  // 1ms sync pulse (micro-seconds)

// ESP-NOW target Manikin Wi-Fi STA MAC address
static const uint8_t MANIKIN_MAC[6] = { 0xE8, 0x3D, 0xC1, 0x82, 0x1B, 0xF0 };
#define ESPNOW_CHANNEL         1    // Wi-Fi channel (must match Manikin)

// ─────────────────────────────────────────────────────────────────────────────
// ESP-NOW Packet Structure
// ─────────────────────────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  cmd;    // 0=AED_OFF, 1=AED_ON, 2=SHOCK_ON, 3=SHOCK_OFF
    uint8_t  stage;  // 0=idle, 1=alert burst, 2=delivering 5s, 3=done
    uint32_t ts;     // millis() timestamp
} aed_espnow_pkt_t;

typedef enum {
    CMD_AED_OFF   = 0,
    CMD_AED_ON    = 1,
    CMD_SHOCK_ON  = 2,
    CMD_SHOCK_OFF = 3
} aed_cmd_t;

// ─────────────────────────────────────────────────────────────────────────────
// AED State Machine
// ─────────────────────────────────────────────────────────────────────────────
typedef enum {
    AED_STATE_STANDBY = 0,
    AED_STATE_STAGE1  = 1,  // Alert Strobe (~360ms)
    AED_STATE_STAGE2  = 2,  // Delivering Red 5s (sync pulse)
    AED_STATE_STAGE3  = 3   // Green Latch 1s -> Standby
} aed_state_t;

static volatile aed_state_t s_state = AED_STATE_STANDBY;
static unsigned long s_stage_start_ms  = 0;
static unsigned long s_last_btn_ms     = 0;
static volatile bool s_btn_pressed     = false;
static bool s_espnow_ok                = false;

// ─────────────────────────────────────────────────────────────────────────────
// LED Mode State
// ─────────────────────────────────────────────────────────────────────────────
typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_WAITING_FOR_PEER,  // Mode 01: Deep Blue breathing (1.8s breath cycle, 5% - 100%)
    LED_MODE_PEER_ACTIVE,       // Mode 02: Emerald Green continuous steady glow
    LED_MODE_SHOCK_ON_ALERT,    // Mode 03: Warm Amber/Yellow triple strobe (3x 60ms ON/60ms OFF)
    LED_MODE_SHOCK_GIVING,      // Mode 04: Urgent Red rapid hazard strobe (5 Hz / 100ms ON/100ms OFF)
    LED_MODE_SHOCK_OFF_LATCH    // Mode 05: Pure Green 1.0s confirmation latch -> Peer Active
} led_mode_t;

static volatile led_mode_t s_led_mode = LED_MODE_WAITING_FOR_PEER;

// ─── PWM Breathing engine: 150 steps @ 20ms = 3000ms (3.0s organic breath cycle)
static const uint8_t BREATH_TABLE[150] = {
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   2,
      2,   3,   3,   4,   5,   6,   7,   9,  10,  12,  14,  16,  18,  20,  22,
     25,  28,  30,  33,  36,  40,  43,  46,  49,  53,  56,  60,  63,  66,  70,
     73,  76,  79,  82,  85,  87,  89,  92,  93,  95,  97,  98,  99,  99, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  99,  99,  98,
     97,  95,  93,  92,  89,  87,  85,  82,  79,  76,  73,  70,  66,  63,  60,
     56,  53,  49,  46,  43,  40,  36,  33,  30,  28,  25,  22,  20,  18,  16,
     14,  12,  10,   9,   7,   6,   5,   4,   3,   3,   2,   2,   1,   1,   1,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
};
static uint8_t s_breath_step = 0;
static unsigned long s_breath_tick_ms = 0;

// ─── Alert Strobe engine (Mode 03: 3x 60ms ON / 60ms OFF = 360ms)
static uint8_t s_alert_step = 0;
static unsigned long s_alert_tick_ms = 0;

// ─── Hazard Strobe engine (Mode 04: 5 Hz = 100ms ON / 100ms OFF)
static bool s_hazard_state = false;
static unsigned long s_hazard_tick_ms = 0;
#define HAZARD_INTERVAL_MS 100

// ─── Latch timer (Mode 05: 1.0s confirmation latch)
static unsigned long s_latch_start_ms = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Low-level RGB output helpers
// ─────────────────────────────────────────────────────────────────────────────
static void rgb_set(bool r, bool g, bool b)
{
#if RGB_LED_ACTIVE_LOW
    digitalWrite(PIN_RGB_RED,   r ? LOW  : HIGH);
    digitalWrite(PIN_RGB_GREEN, g ? LOW  : HIGH);
    digitalWrite(PIN_RGB_BLUE,  b ? LOW  : HIGH);
#else
    digitalWrite(PIN_RGB_RED,   r ? HIGH : LOW);
    digitalWrite(PIN_RGB_GREEN, g ? HIGH : LOW);
    digitalWrite(PIN_RGB_BLUE,  b ? HIGH : LOW);
#endif
}

static void onboard_led_set(bool on)
{
    // Active-LOW on ESP32-C3 Super Mini onboard LED
    digitalWrite(PIN_STATUS_LED, on ? LOW : HIGH);
}

static void led_set_mode(led_mode_t mode)
{
    s_led_mode = mode;
    switch (mode) {
        case LED_MODE_WAITING_FOR_PEER:
            s_breath_step = 0;
            break;
        case LED_MODE_SHOCK_ON_ALERT:
            s_alert_step = 0;
            s_alert_tick_ms = millis();
            break;
        case LED_MODE_SHOCK_GIVING:
            s_hazard_state = false;
            s_hazard_tick_ms = millis();
            break;
        case LED_MODE_SHOCK_OFF_LATCH:
            s_latch_start_ms = millis();
            break;
        default:
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Hardware Sync Pulse (1 ms pulse on GPIO 7 to Manikin)
// ─────────────────────────────────────────────────────────────────────────────
static void fire_sync_pulse(void)
{
    digitalWrite(PIN_SYNC_OUT, HIGH);
    delayMicroseconds(T_SYNC_PULSE_US);
    digitalWrite(PIN_SYNC_OUT, LOW);
    Serial.println("[SYNC] 1ms pulse fired on GPIO 7");
}

// ─────────────────────────────────────────────────────────────────────────────
// ESP-NOW Send Helper
// ─────────────────────────────────────────────────────────────────────────────
static void espnow_send(uint8_t cmd, uint8_t stage)
{
    if (!s_espnow_ok) {
        Serial.printf("[ESP-NOW] TX dropped (not ready): cmd=%d stage=%d\n", cmd, stage);
        return;
    }
    aed_espnow_pkt_t pkt = {
        .cmd   = cmd,
        .stage = stage,
        .ts    = (uint32_t)millis()
    };
    esp_err_t err = esp_now_send(MANIKIN_MAC, (const uint8_t *)&pkt, sizeof(pkt));
    Serial.printf("[ESP-NOW] TX cmd=%d stage=%d ts=%u ms -> %s\n",
                  cmd, stage, pkt.ts,
                  (err == ESP_OK) ? "QUEUED" : "ERROR");
}

// ─────────────────────────────────────────────────────────────────────────────
// Shock Button ISR (Falling edge on GPIO 9)
// ─────────────────────────────────────────────────────────────────────────────
static void IRAM_ATTR shock_button_isr(void)
{
    s_btn_pressed = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// LED Engine Update Loop
// ─────────────────────────────────────────────────────────────────────────────
static void led_update(void)
{
    unsigned long now = millis();

    switch (s_led_mode) {
        case LED_MODE_OFF:
            rgb_set(false, false, false);
            onboard_led_set(false);
            break;

        case LED_MODE_WAITING_FOR_PEER:
            // Mode 01: Deep Blue Breathing (3.0s organic cycle)
            if ((now - s_breath_tick_ms) >= 20) {
                s_breath_tick_ms = now;
                uint8_t v = BREATH_TABLE[s_breath_step];
                s_breath_step = (s_breath_step + 1) % 150;
                // Duty threshold: simulate Blue channel duty
                rgb_set(false, false, (s_breath_step % 4 != 0) || (v > 20));
                onboard_led_set(v > 45);
            }
            break;

        case LED_MODE_PEER_ACTIVE:
            // Mode 02: Emerald Green Continuous Steady Glow
            rgb_set(false, true, false);
            onboard_led_set(true);
            break;

        case LED_MODE_SHOCK_ON_ALERT:
            // Mode 03: Warm Amber/Yellow Triple Strobe Burst (3x 60ms ON / 60ms OFF)
            if ((now - s_alert_tick_ms) >= T_SHOCK_ALERT_STROBE_MS) {
                s_alert_tick_ms = now;
                if (s_alert_step < 6) {
                    bool on = ((s_alert_step % 2) == 0);
                    rgb_set(on, on, false); // Amber / Yellow (R + G)
                    onboard_led_set(on);
                    s_alert_step++;
                } else {
                    // Auto-transitions to Mode 04
                    led_set_mode(LED_MODE_SHOCK_GIVING);
                }
            }
            break;

        case LED_MODE_SHOCK_GIVING:
            // Mode 04: Urgent Red Rapid Hazard Strobe (5 Hz: 100ms ON / 100ms OFF)
            if ((now - s_hazard_tick_ms) >= HAZARD_INTERVAL_MS) {
                s_hazard_tick_ms = now;
                s_hazard_state = !s_hazard_state;
                rgb_set(s_hazard_state, false, false);
                onboard_led_set(s_hazard_state);
            }
            break;

        case LED_MODE_SHOCK_OFF_LATCH:
            // Mode 05: Pure Green 1.0s confirmation latch -> returns to Peer Active
            rgb_set(false, true, false);
            onboard_led_set(true);
            if ((now - s_latch_start_ms) >= T_SHOCK_OFF_LATCH_MS) {
                led_set_mode(LED_MODE_PEER_ACTIVE);
            }
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Shock Sequence State Machine Update
// ─────────────────────────────────────────────────────────────────────────────
static void shock_sm_update(void)
{
    unsigned long now = millis();

    // Check button press
    if (s_btn_pressed) {
        s_btn_pressed = false;

        // Enforce debounce
        if ((now - s_last_btn_ms) < T_BTN_DEBOUNCE_MS) {
            return;
        }
        // Enforce cooldown
        if ((now - s_last_btn_ms) < T_SHOCK_COOLDOWN_MS && s_last_btn_ms != 0) {
            Serial.printf("[BTN] Cooldown active (%lu ms remaining)\n",
                          T_SHOCK_COOLDOWN_MS - (now - s_last_btn_ms));
            return;
        }
        // Only trigger if currently in standby
        if (s_state != AED_STATE_STANDBY) {
            Serial.println("[BTN] Shock sequence already running — ignored");
            return;
        }

        // ── START Stage 1: Mode 03 Alert Strobe Burst (~360ms) ───────────
        s_last_btn_ms  = now;
        s_stage_start_ms = now;
        s_state = AED_STATE_STAGE1;

        Serial.println("══════════════════════════════════════════════════");
        Serial.println(">>> [MODE 03: SHOCK ON (ALERT)] WARM AMBER BURST <<<");
        Serial.println("══════════════════════════════════════════════════");

        led_set_mode(LED_MODE_SHOCK_ON_ALERT);
        espnow_send(CMD_SHOCK_ON, 1);
    }

    // ── Stage 1 -> Stage 2 transition after 360ms Alert Burst ─────────────
    if (s_state == AED_STATE_STAGE1) {
        if ((now - s_stage_start_ms) >= 360) {
            s_state = AED_STATE_STAGE2;
            s_stage_start_ms = now;

            Serial.println("══════════════════════════════════════════════════");
            Serial.println(">>> [MODE 04: SHOCK GIVING] URGENT RED 5 Hz HAZARD<<<");
            Serial.println(">>> Firing 1ms sync pulse on GPIO 7              <<<");
            Serial.println("══════════════════════════════════════════════════");

            led_set_mode(LED_MODE_SHOCK_GIVING);
            fire_sync_pulse();
            espnow_send(CMD_SHOCK_ON, 2);
        }
    }

    // ── Stage 2 -> Stage 3 transition after 5s Red Hazard ─────────────────
    else if (s_state == AED_STATE_STAGE2) {
        if ((now - s_stage_start_ms) >= T_SHOCK_GIVING_MS) {
            s_state = AED_STATE_STAGE3;
            s_stage_start_ms = now;

            Serial.println("══════════════════════════════════════════════════");
            Serial.println(">>> [MODE 05: SHOCK OFF] PURE GREEN 1s LATCH     <<<");
            Serial.println(">>> Safe to Touch Patient — Resume CPR            <<<");
            Serial.println("══════════════════════════════════════════════════");

            led_set_mode(LED_MODE_SHOCK_OFF_LATCH);
            espnow_send(CMD_SHOCK_OFF, 3);
        }
    }

    // ── Stage 3 -> Return to Standby after 1s Green Latch ─────────────────
    else if (s_state == AED_STATE_STAGE3) {
        if ((now - s_stage_start_ms) >= T_SHOCK_OFF_LATCH_MS) {
            s_state = AED_STATE_STANDBY;
            Serial.println(">>> Returned to MODE 02: PEER ACTIVE & READY (Solid Green) <<<");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// ESP-NOW Send Callback
// ─────────────────────────────────────────────────────────────────────────────
static void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    Serial.printf("[ESP-NOW] Send to %02X:%02X:%02X:%02X:%02X:%02X -> %s\n",
        mac_addr[0], mac_addr[1], mac_addr[2],
        mac_addr[3], mac_addr[4], mac_addr[5],
        (status == ESP_NOW_SEND_SUCCESS) ? "SUCCESS" : "FAIL");
}

// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("\n══════════════════════════════════════════════════");
    Serial.println(" CPR External AED Unit v2.1.0 (Arduino ESP-NOW)");
    Serial.println(" Target  : ESP32-C3 Super Mini");
    Serial.println(" Red: GPIO 3 | Grn: GPIO 4 | Blu: GPIO 5 | Sync: GPIO 7");
    Serial.println("══════════════════════════════════════════════════");

    // Initialize Outputs
    pinMode(PIN_STATUS_LED, OUTPUT);
    onboard_led_set(false);

    pinMode(PIN_SYNC_OUT, OUTPUT);
    digitalWrite(PIN_SYNC_OUT, LOW);

    pinMode(PIN_RGB_RED,   OUTPUT);
    pinMode(PIN_RGB_GREEN, OUTPUT);
    pinMode(PIN_RGB_BLUE,  OUTPUT);
    rgb_set(false, false, false);

    // Shock Button (Active-LOW, internal pull-up, falling edge ISR)
    pinMode(PIN_SHOCK_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_SHOCK_BUTTON), shock_button_isr, FALLING);

    // Wi-Fi Station mode for ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Print MAC address
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    Serial.printf("[MCU] AED Wi-Fi STA MAC : %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("[MCU] Target Manikin MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  MANIKIN_MAC[0], MANIKIN_MAC[1], MANIKIN_MAC[2],
                  MANIKIN_MAC[3], MANIKIN_MAC[4], MANIKIN_MAC[5]);
    Serial.printf("[MCU] ESP-NOW Channel   : %d\n", ESPNOW_CHANNEL);

    // ESP-NOW Init
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERROR] ESP-NOW initialization FAILED — halting");
        led_set_mode(LED_MODE_OFF);
        while (1) { delay(1000); }
    }

    esp_now_register_send_cb(on_data_sent);

    // Register Manikin as ESP-NOW peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, MANIKIN_MAC, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;
    peerInfo.ifidx   = WIFI_IF_STA;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ERROR] Failed to add ESP-NOW peer — halting");
        led_set_mode(LED_MODE_OFF);
        while (1) { delay(1000); }
    }

    s_espnow_ok = true;
    Serial.println("[ESP-NOW] Initialized. Peer registered.");

    // Announce AED active
    espnow_send(CMD_AED_ON, 0);

    // In accordance with Clinical Resuscitation Visual System:
    // Once peer is registered: Mode 02: PEER ACTIVE & READY (Solid Green)
    led_set_mode(LED_MODE_PEER_ACTIVE);
    Serial.println(">>> [MODE 02: PEER ACTIVE & READY] SOLID GREEN steady glow <<<");
}

// ─────────────────────────────────────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    shock_sm_update();
    led_update();

    static unsigned long s_hb_ms = 0;
    if ((millis() - s_hb_ms) >= 5000) {
        s_hb_ms = millis();
        const char *state_str[] = { "STANDBY", "MODE 03: ALERT BURST", "MODE 04: RED HAZARD", "MODE 05: GREEN LATCH" };
        Serial.printf("[HB] State: %s | LED Mode: %d | ESP-NOW: %s\n",
                      state_str[(int)s_state], (int)s_led_mode,
                      s_espnow_ok ? "OK" : "FAIL");
    }
}
