/**
 * @file    ble_service.c
 * @brief   Zephyr BLE GATT Nordic UART Service & Battery Service Implementation
 * @version 9.0.0
 */

#include "ble_service.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>

LOG_MODULE_REGISTER(ble_service, LOG_LEVEL_INF);

static struct bt_conn *default_conn = NULL;
static volatile bool s_client_connected = false;
static volatile bool s_is_advertising = false;
static bool s_tx_notify_enabled = false;

static struct k_work_delayable s_adv_work;
static void adv_work_handler(struct k_work *work);

static ble_heart_mode_callback_t       s_heart_cb    = NULL;
static ble_announce_request_callback_t s_announce_cb = NULL;
static ble_aed_mode_callback_t         s_aed_cb      = NULL;
static ble_air_mode_callback_t         s_air_cb      = NULL;

/* Nordic UART Service (NUS) UUIDs */
static struct bt_uuid_128 nus_uuid    = BT_UUID_INIT_128(BT_UUID_NUS_SERVICE_VAL);
static struct bt_uuid_128 nus_rx_uuid = BT_UUID_INIT_128(BT_UUID_NUS_RX_VAL);
static struct bt_uuid_128 nus_tx_uuid = BT_UUID_INIT_128(BT_UUID_NUS_TX_VAL);

static uint8_t s_tx_buffer[32];
static uint16_t s_tx_len = 0;

static ssize_t on_nus_rx_write(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               const void *buf, uint16_t len,
                               uint16_t offset, uint8_t flags)
{
    const char *cmd_buf = (const char *)buf;
    if (len < 1) return len;

    const char cmd = cmd_buf[0];
    switch (cmd) {
        case 'C':
        case '?':
            if (s_announce_cb) {
                s_announce_cb();
            }
            break;

        case 'A': {
            aed_mode_t mode = AED_MODE_DISABLED;
            if (len >= 2) {
                if (cmd_buf[1] == '2') {
                    mode = AED_MODE_CHEST;
                } else if (cmd_buf[1] == '1') {
                    mode = AED_MODE_HIP;
                } else if (cmd_buf[1] == '0') {
                    mode = AED_MODE_DISABLED;
                }
            }
            if (s_aed_cb) {
                s_aed_cb(mode);
            }
            break;
        }

        case 'M': {
            bool enable = true;
            if (len >= 2 && cmd_buf[1] == '0') {
                enable = false;
            }
            if (s_air_cb) {
                s_air_cb(enable);
            }
            break;
        }

        case 'H':
        case 'V': {
            uint8_t mode = 1;
            if (len >= 2) {
                mode = cmd_buf[1] - '0';
            }
            if (s_heart_cb) {
                if (mode == 0) s_heart_cb(HEART_STOPPED);
                else if (mode == 2) s_heart_cb(HEART_CONTINUOUS);
                else if (mode == 3) s_heart_cb(HEART_SINGLE_BEAT);
                else s_heart_cb(HEART_NORMAL);
            }
            break;
        }

        case 'B':
            if (s_heart_cb) {
                s_heart_cb(HEART_SINGLE_BEAT);
            }
            break;

        case 'S':
            if (s_heart_cb) {
                s_heart_cb(HEART_STOPPED);
            }
            break;

        default:
            break;
    }

    return len;
}

static ssize_t on_nus_tx_read(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len,
                              uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, s_tx_buffer, s_tx_len);
}

static void on_nus_tx_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    s_tx_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("NUS TX CCC notification %s", s_tx_notify_enabled ? "enabled" : "disabled");
}

/* GATT Service Declaration */
BT_GATT_SERVICE_DEFINE(nus_svc,
    BT_GATT_PRIMARY_SERVICE(&nus_uuid),
    BT_GATT_CHARACTERISTIC(&nus_rx_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, on_nus_rx_write, NULL),
    BT_GATT_CHARACTERISTIC(&nus_tx_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           on_nus_tx_read, NULL, NULL),
    BT_GATT_CCC(on_nus_tx_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* Advertising Data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME) - 1),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SERVICE_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

int ble_service_start_advertising(void)
{
    if (s_client_connected) {
        return 0;
    }

    /* Stop previous advertising to guarantee clean state */
    bt_le_adv_stop();
    s_is_advertising = false;

    int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return err;
    }

    s_is_advertising = true;
    LOG_INF("Advertising started as: %s", BLE_DEVICE_NAME);
    return 0;
}

static void adv_work_handler(struct k_work *work)
{
    if (s_client_connected) {
        return;
    }

    int err = ble_service_start_advertising();
    if (err) {
        LOG_WRN("Advertising restart failed (err %d), retrying in 500 ms...", err);
        k_work_reschedule(&s_adv_work, K_MSEC(500));
    }
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("BLE Connection failed (err 0x%02x)", err);
        s_is_advertising = false;
        k_work_reschedule(&s_adv_work, K_MSEC(50));
        return;
    }
    k_work_cancel_delayable(&s_adv_work);
    default_conn = bt_conn_ref(conn);
    s_client_connected = true;
    s_is_advertising = false;
    LOG_INF(">>> BLE Client connected! (Just Works bonding active)");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("<<< BLE Client disconnected (reason 0x%02x)", reason);
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
    s_client_connected = false;
    s_is_advertising = false;
    s_tx_notify_enabled = false;

    if (s_heart_cb) s_heart_cb(HEART_STOPPED);
    if (s_aed_cb)   s_aed_cb(AED_MODE_DISABLED);
    if (s_air_cb)   s_air_cb(false);

    /* In Zephyr, the stack still holds a reference to the conn object inside disconnected().
     * Restarting advertising synchronously here can fail with -ENOMEM when connection slots are constrained.
     * We schedule advertising restart after a brief delay, and also hook into recycled().
     */
    k_work_reschedule(&s_adv_work, K_MSEC(50));
}

static void recycled(void)
{
    LOG_INF("BLE Connection object freed & recycled. Triggering advertising immediately.");
    k_work_reschedule(&s_adv_work, K_NO_WAIT);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
    .recycled     = recycled,
};

void ble_service_register_callbacks(ble_heart_mode_callback_t heart_cb,
                                    ble_announce_request_callback_t announce_cb,
                                    ble_aed_mode_callback_t aed_cb,
                                    ble_air_mode_callback_t air_cb)
{
    s_heart_cb    = heart_cb;
    s_announce_cb = announce_cb;
    s_aed_cb      = aed_cb;
    s_air_cb      = air_cb;
}

int ble_service_init(void)
{
    k_work_init_delayable(&s_adv_work, adv_work_handler);

    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

    LOG_INF("Bluetooth initialized successfully");

    return ble_service_start_advertising();
}

bool ble_service_is_connected(void)
{
    return s_client_connected;
}

bool ble_service_is_advertising(void)
{
    return s_is_advertising;
}

void ble_service_ensure_advertising(void)
{
    if (!s_client_connected && !s_is_advertising) {
        LOG_INF("Watchdog: advertising was inactive while disconnected, restarting...");
        k_work_reschedule(&s_adv_work, K_NO_WAIT);
    }
}

static int send_nus_data(const void *data, uint16_t len)
{
    if (!s_client_connected || !s_tx_notify_enabled) {
        return -ENOTCONN;
    }

    memcpy(s_tx_buffer, data, len);
    s_tx_len = len;

    /* Index 3 is TX Characteristic Value in nus_svc */
    return bt_gatt_notify(default_conn, &nus_svc.attrs[3], data, len);
}

int ble_service_send_announce(const AnnouncePacket *pkt)
{
    return send_nus_data(pkt, sizeof(AnnouncePacket));
}

int ble_service_send_compression(const CprPacket *pkt)
{
    return send_nus_data(pkt, sizeof(CprPacket));
}

int ble_service_send_aed(const CprPacket *pkt)
{
    return send_nus_data(pkt, sizeof(CprPacket));
}

int ble_service_send_air(const CprPacket *pkt)
{
    return send_nus_data(pkt, sizeof(CprPacket));
}

void ble_service_set_battery_level(uint8_t percentage)
{
    bt_bas_set_battery_level(percentage);
}

