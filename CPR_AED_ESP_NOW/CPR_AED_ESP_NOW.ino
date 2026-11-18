/**
 * @file    CPR_AED_ESP_NOW.ino
 * @brief   CPR External AED Shock Unit — Arduino Firmware (ESP-NOW Protocol)
 * @version 2.0.0
 * @target  ESP32-C3 (Arduino IDE, esp32 by Espressif board package)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  PROTOCOL: ESP-NOW ONLY (NO BLE)
 *  Sends 4-command packets to Manikin MCU:
 *    CMD 0 = AED_OFF   — AED unit powered off / deactivated
 *    CMD 1 = AED_ON    — AED unit powered on / activated
 *    CMD 2 = SHOCK_ON  — Shock sequence active (stage 1=yellow, 2=red)
 *    CMD 3 = SHOCK_OFF — Shock sequence complete
 *
 *  SHOCK SEQUENCE (triggered by GPIO 9 button press):
 *    Stage 1 → 5s Yellow  : Instruction / Rhythm Analysis
 *    Stage 2 → 5s Red     : Delivering Shock (1ms sync pulse on GPIO 7)
 *    Stage 3 → Solid Green: Shock Complete → return to standby
 *
 *  GPIO MAP:
 *    GPIO 9  — Shock Button    (Active-LOW, internal pull-up)
 *    GPIO 6  — Power Switch    (Option 1: hardware cut, not used in code)
 *    GPIO 8  — Onboard LED     (Active-LOW, Blue status LED)
 *    GPIO 7  — Sync Pulse Out  (1ms HIGH pulse at shock Stage 2 start)
 *    GPIO 3  — RGB Red
 *    GPIO 4  — RGB Green
 *    GPIO 5  — RGB Blue
 *
 *  TARGET MANIKIN MAC: E8:3D:C1:82:1B:F0 (Wi-Fi STA MAC for ESP-NOW)
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ─────────────────────────────────────────────────────────────────────────────
// CONFIGURATION — Hardware pins & timing (matches aed_config.h)
// ─────────────────────────────────────────────────────────────────────────────
#define PIN_SHOCK_BUTTON       9    // Active-LOW shock button (BOOT pin or external)
#define PIN_POWER_SWITCH       6    // Power switch (Option 1: hardware cut — not read in code)
#define PIN_STATUS_LED         8    // Active-LOW onboard blue LED
#define PIN_SYNC_OUT           7    // Hardware sync pulse output to Manikin

#define PIN_RGB_RED            3    // RGB Red channel
#define PIN_RGB_GREEN          4    // RGB Green channel
#define PIN_RGB_BLUE           5    // RGB Blue channel

// RGB polarity: 0 = Common Cathode (Active-HIGH), 1 = Common Anode (Active-LOW)
#define RGB_LED_ACTIVE_LOW     0

// Timing constants (ms)
#define T_BTN_DEBOUNCE_MS      50
#define T_SHOCK_COOLDOWN_MS    11000  // 11s lockout after a full 10s sequence
#define T_SHOCK_INSTRUCTION_MS 5000  // 5s Yellow stage
#define T_SHOCK_GIVING_MS      5000  // 5s Red stage
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
    uint8_t  stage;  // 0=idle, 1=yellow/instruction, 2=red/delivering, 3=done
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
    AED_STATE_STAGE1  = 1,  // Yellow — Instruction 5s
    AED_STATE_STAGE2  = 2,  // Red    — Delivering 5s
    AED_STATE_STAGE3  = 3   // Green  — Complete
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
    LED_MODE_BLUE_BREATH,    // Standby: deep blue breathing
    LED_MODE_SOLID_GREEN,    // ESP-NOW ready / shock done
    LED_MODE_YELLOW,         // Stage 1: Yellow steady
    LED_MODE_RED_STROBE,     // Stage 2: Red rapid strobe
    LED_MODE_GREEN_LATCH     // Stage 3: 1s solid green, then back to standby
} led_mode_t;

static volatile led_mode_t s_led_mode = LED_MODE_BLUE_BREATH;

// ─── PWM Breathing engine ─────────────────────────────────────────────────
// Gamma-corrected sine-based 150-step breath table (3s cycle @ 20ms tick)
static const uint8_t BREATH_TABLE[150] = {
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  2,
      2,  3,  3,  4,  5,  6,  7,  9, 10, 12, 14, 16, 18, 20, 22,
     25, 28, 30, 33, 36, 40, 43, 46, 49, 53, 56, 60, 63, 66, 70,
     73, 76, 79, 82, 85, 87, 89, 92, 93, 95, 97, 98, 99, 99,100,
    100,100,100,100,100,100,100,100,100,100,100,100, 99, 99, 98,
     97, 95, 93, 92, 89, 87, 85, 82, 79, 76, 73, 70, 66, 63, 60,
     56, 53, 49, 46, 43, 40, 36, 33, 30, 28, 25, 22, 20, 18, 16,
     14, 12, 10,  9,  7,  6,  5,  4,  3,  3,  2,  2,  1,  1,  1,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};
static uint8_t s_breath_step = 0;
static unsigned long s_breath_tick_ms = 0;

// ─── Strobe engine ───────────────────────────────────────────────────────
static bool s_strobe_state = false;
static unsigned long s_strobe_tick_ms = 0;
#define STROBE_INTERVAL_MS 100   // 5 Hz strobe

// ─── Latch timer ─────────────────────────────────────────────────────────
static unsigned long s_latch_start_ms = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Low-level RGB output helper
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

static void rgb_pwm(uint8_t r_pct, uint8_t g_pct, uint8_t b_pct)
{
    // Map 0-100% to 0-255 for analogWrite (LEDC)
    // Active-LOW boards: invert the duty
#if RGB_LED_ACTIVE_LOW
    analogWrite(PIN_RGB_RED,   255 - (uint8_t)(r_pct * 255 / 100));
    analogWrite(PIN_RGB_GREEN, 255 - (uint8_t)(g_pct * 255 / 100));
    analogWrite(PIN_RGB_BLUE,  255 - (uint8_t)(b_pct * 255 / 100));
#else
    analogWrite(PIN_RGB_RED,   (uint8_t)(r_pct * 255 / 100));
    analogWrite(PIN_RGB_GREEN, (uint8_t)(g_pct * 255 / 100));
    analogWrite(PIN_RGB_BLUE,  (uint8_t)(b_pct * 255 / 100));
#endif
}

static void onboard_led_set(bool on)
{
    // Onboard LED is Active-LOW on ESP32-C3 Super Mini
    digitalWrite(PIN_STATUS_LED, on ? LOW : HIGH);
}

// ─────────────────────────────────────────────────────────────────────────────
// LED Mode Engine (called every loop iteration, non-blocking)
// ─────────────────────────────────────────────────────────────────────────────
static void led_set_mode(led_mode_t mode)
{
    s_led_mode = mode;
    s_breath_step = 0;
    s_breath_tick_ms = millis();
    s_strobe_tick_ms = millis();
    s_strobe_state   = false;
    if (mode == LED_MODE_GREEN_LATCH) {
        s_latch_start_ms = millis();
    }
}

static void led_update(void)
{
    unsigned long now = millis();

    switch (s_led_mode) {
        case LED_MODE_OFF:
            rgb_set(false, false, false);
            onboard_led_set(false);
            break;

        case LED_MODE_BLUE_BREATH:
            // Update breath every 20ms
            if ((now - s_breath_tick_ms) >= 20) {
                s_breath_tick_ms = now;
                uint8_t v = BREATH_TABLE[s_breath_step];
                s_breath_step = (s_breath_step + 1) % 150;
                rgb_pwm(0, 0, v);
                onboard_led_set(v > 45);
            }
            break;

        case LED_MODE_SOLID_GREEN:
            rgb_set(false, true, false);
            onboard_led_set(true);
            break;

        case LED_MODE_YELLOW:
            // Warm Yellow: 100% R + 80% G
            rgb_pwm(100, 80, 0);
            onboard_led_set(true);
            break;

        case LED_MODE_RED_STROBE:
            // 5 Hz red strobe
            if ((now - s_strobe_tick_ms) >= STROBE_INTERVAL_MS) {
                s_strobe_tick_ms = now;
                s_strobe_state = !s_strobe_state;
                rgb_set(s_strobe_state, false, false);
                onboard_led_set(s_strobe_state);
            }
            break;

        case LED_MODE_GREEN_LATCH:
            // Solid green for T_SHOCK_OFF_LATCH_MS, then return to standby blue
            rgb_set(false, true, false);
            onboard_led_set(true);
            if ((now - s_latch_start_ms) >= T_SHOCK_OFF_LATCH_MS) {
                led_set_mode(LED_MODE_BLUE_BREATH);
            }
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// ESP-NOW Send Callback
// ─────────────────────────────────────────────────────────────────────────────
static void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    Serial.printf("[ESP-NOW] Send to %02X:%02X:%02X:%02X:%02X:%02X → %s\n",
        mac_addr[0], mac_addr[1], mac_addr[2],
        mac_addr[3], mac_addr[4], mac_addr[5],
        (status == ESP_NOW_SEND_SUCCESS) ? "SUCCESS" : "FAIL");
}

// ─────────────────────────────────────────────────────────────────────────────
// Send an ESP-NOW command packet to the Manikin
// ─────────────────────────────────────────────────────────────────────────────
static void espnow_send(aed_cmd_t cmd, uint8_t stage)
{
    if (!s_espnow_ok) {
        Serial.println("[WARN] ESP-NOW not initialized — packet dropped");
        return;
    }
    aed_espnow_pkt_t pkt;
    pkt.cmd   = (uint8_t)cmd;
    pkt.stage = stage;
    pkt.ts    = (uint32_t)millis();

    esp_err_t result = esp_now_send(MANIKIN_MAC, (uint8_t *)&pkt, sizeof(pkt));
    Serial.printf("[ESP-NOW] TX cmd=%d stage=%d ts=%lu → %s\n",
                  pkt.cmd, pkt.stage, (unsigned long)pkt.ts,
                  (result == ESP_OK) ? "queued" : "err");
}

// ─────────────────────────────────────────────────────────────────────────────
// Hardware sync pulse — 1ms HIGH on GPIO 7
// ─────────────────────────────────────────────────────────────────────────────
static void fire_sync_pulse(void)
{
    digitalWrite(PIN_SYNC_OUT, HIGH);
    delayMicroseconds(T_SYNC_PULSE_US);
    digitalWrite(PIN_SYNC_OUT, LOW);
    Serial.println("[SYNC] 1ms hardware sync pulse fired on GPIO 7");
}

// ─────────────────────────────────────────────────────────────────────────────
// Shock Button ISR (GPIO 9, Active-LOW)
// ─────────────────────────────────────────────────────────────────────────────
static void IRAM_ATTR shock_button_isr(void)
{
    s_btn_pressed = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Shock Sequence — Non-blocking state machine, called in loop()
// ─────────────────────────────────────────────────────────────────────────────
static void shock_sm_update(void)
{
    unsigned long now = millis();

    // ── Handle button press (debounce + cooldown in ISR path) ──────────────
    if (s_btn_pressed) {
        s_btn_pressed = false;
        // Enforce debounce
        if ((now - s_last_btn_ms) < T_BTN_DEBOUNCE_MS) {
            return;
        }
        // Enforce cooldown (11s)
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

        // ── START Stage 1: Yellow / Instruction ───────────────────────────
        s_last_btn_ms  = now;
        s_stage_start_ms = now;
        s_state = AED_STATE_STAGE1;

        Serial.println("══════════════════════════════════════════════════");
        Serial.println(">>> [SHOCK STAGE 1/3] INSTRUCTION (YELLOW 5s) <<<");
        Serial.println("══════════════════════════════════════════════════");

        led_set_mode(LED_MODE_YELLOW);
        espnow_send(CMD_SHOCK_ON, 1);  // Notify Manikin: Stage 1 start
    }

    // ── Stage 1 → Stage 2 transition after 5s ─────────────────────────────
    if (s_state == AED_STATE_STAGE1) {
        if ((now - s_stage_start_ms) >= T_SHOCK_INSTRUCTION_MS) {
            s_state = AED_STATE_STAGE2;
            s_stage_start_ms = now;

            Serial.println("══════════════════════════════════════════════════");
            Serial.println(">>> [SHOCK STAGE 2/3] DELIVERING SHOCK (RED 5s) <<<");
            Serial.println(">>> Firing 1ms sync pulse on GPIO 7              <<<");
            Serial.println("══════════════════════════════════════════════════");

            led_set_mode(LED_MODE_RED_STROBE);
            fire_sync_pulse();               // 1ms hardware pulse to Manikin
            espnow_send(CMD_SHOCK_ON, 2);    // Notify Manikin: Stage 2 (delivering)
        }
    }

    // ── Stage 2 → Stage 3 transition after 5s ─────────────────────────────
    else if (s_state == AED_STATE_STAGE2) {
        if ((now - s_stage_start_ms) >= T_SHOCK_GIVING_MS) {
            s_state = AED_STATE_STAGE3;
            s_stage_start_ms = now;

            Serial.println("══════════════════════════════════════════════════");
            Serial.println(">>> [SHOCK STAGE 3/3] SHOCK COMPLETE (GREEN)     <<<");
            Serial.println(">>> Safe to Touch Patient — Resume CPR            <<<");
            Serial.println("══════════════════════════════════════════════════");

            led_set_mode(LED_MODE_GREEN_LATCH);  // 1s green, then back to blue breath
            espnow_send(CMD_SHOCK_OFF, 3);       // Notify Manikin: Shock complete
        }
    }

    // ── Stage 3 → Standby (after green latch timeout in led_update) ───────
    else if (s_state == AED_STATE_STAGE3) {
        // LED_MODE_GREEN_LATCH auto-transitions to BLUE_BREATH after 1s
        // We just need to bring state machine back to standby
        if (s_led_mode == LED_MODE_BLUE_BREATH) {
            s_state = AED_STATE_STANDBY;
            Serial.println("[AED] Returned to STANDBY — ready for next press");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("══════════════════════════════════════════════════════");
    Serial.println(" CPR External AED Unit v2.0.0 [Arduino / ESP-NOW]");
    Serial.println(" Target: ESP32-C3 Super Mini");
    Serial.println(" Protocol: ESP-NOW ONLY (No BLE)");
    Serial.println("══════════════════════════════════════════════════════");

    // ── GPIO Setup ─────────────────────────────────────────────────────────
    // Onboard Status LED (Active-LOW)
    pinMode(PIN_STATUS_LED, OUTPUT);
    onboard_led_set(false);

    // Hardware Sync Pulse Output
    pinMode(PIN_SYNC_OUT, OUTPUT);
    digitalWrite(PIN_SYNC_OUT, LOW);

    // RGB LED outputs
    pinMode(PIN_RGB_RED,   OUTPUT);
    pinMode(PIN_RGB_GREEN, OUTPUT);
    pinMode(PIN_RGB_BLUE,  OUTPUT);
    rgb_set(false, false, false);

    // Shock Button (Active-LOW, internal pull-up, falling edge ISR)
    pinMode(PIN_SHOCK_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_SHOCK_BUTTON), shock_button_isr, FALLING);

    // ── Wi-Fi (Station mode, no connection — required for ESP-NOW) ─────────
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Print MAC address of this AED unit
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    Serial.printf("[MCU] AED Wi-Fi STA MAC : %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("[MCU] Target Manikin MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  MANIKIN_MAC[0], MANIKIN_MAC[1], MANIKIN_MAC[2],
                  MANIKIN_MAC[3], MANIKIN_MAC[4], MANIKIN_MAC[5]);
    Serial.printf("[MCU] ESP-NOW Channel   : %d\n", ESPNOW_CHANNEL);

    // ── ESP-NOW Init ───────────────────────────────────────────────────────
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

    // ── Boot: Announce AED active ──────────────────────────────────────────
    // With hardware cut switch (Option 1), board only powers on when switch is ON.
    // So at boot, AED is always ON → send AED_ON immediately.
    espnow_send(CMD_AED_ON, 0);
    Serial.println("[AED] Boot complete. Sending AED_ON. Waiting for shock button...");

    // Start LED in blue breathing standby mode
    led_set_mode(LED_MODE_BLUE_BREATH);
}

// ─────────────────────────────────────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    // 1. Update shock sequence state machine (non-blocking)
    shock_sm_update();

    // 2. Update LED animations (non-blocking)
    led_update();

    // 3. Periodic heartbeat log every 5 seconds
    static unsigned long s_hb_ms = 0;
    if ((millis() - s_hb_ms) >= 5000) {
        s_hb_ms = millis();
        const char *state_str[] = { "STANDBY", "STAGE1-YELLOW", "STAGE2-RED", "STAGE3-GREEN" };
        Serial.printf("[HB] State: %s | LED: %d | ESP-NOW: %s\n",
                      state_str[(int)s_state], (int)s_led_mode,
                      s_espnow_ok ? "OK" : "FAIL");
    }
}
