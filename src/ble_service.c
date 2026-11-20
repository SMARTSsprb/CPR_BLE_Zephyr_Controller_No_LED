/**
 * @file    ble_service.c
 * @brief   Zephyr BLE GATT Nordic UART Service & Battery Service Implementation
 * @version 9.0.0
 */

#include "ble_service.h"
#include "aed_client.h"
#include <string.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
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
static struct k_spinlock peer_lock;
static struct k_spinlock cache_lock;

struct bt_conn *ble_app_ref(void)
{
    k_spinlock_key_t key = k_spin_lock(&peer_lock);
    struct bt_conn *app = default_conn ? bt_conn_ref(default_conn) : NULL;
    k_spin_unlock(&peer_lock, key);
    return app;
}

bool ble_is_app(struct bt_conn *conn)
{
    k_spinlock_key_t key = k_spin_lock(&peer_lock);
    bool match = conn && conn == default_conn;
    k_spin_unlock(&peer_lock, key);
    return match;
}

static struct k_work_delayable s_adv_work;
static void adv_work_handler(struct k_work *work);

static ble_heart_mode_callback_t       s_heart_cb    = NULL;
static ble_announce_request_callback_t s_announce_cb = NULL;
static ble_aed_mode_callback_t         s_aed_cb      = NULL;
static ble_air_mode_callback_t         s_air_cb      = NULL;
static ble_shock_mode_callback_t        s_shock_cb    = NULL;

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
    if (!ble_is_app(conn)) return BT_GATT_ERR(BT_ATT_ERR_AUTHORIZATION);
    if (len < 1) return len;
#ifdef CONFIG_CPR_AED_BRIDGE
    if (cmd_buf[0] == '@') return aed_client_command(conn, buf, len, offset);
#endif

    /* Multi-character ASCII string commands support */
    if (len >= 6 && memcmp(cmd_buf, "AED_ON", 6) == 0) {
        if (s_aed_cb) s_aed_cb(AED_MODE_HIP);
        return len;
    }
    if (len >= 7 && memcmp(cmd_buf, "AED_OFF", 7) == 0) {
        if (s_aed_cb) s_aed_cb(AED_MODE_DISABLED);
        return len;
    }
    if ((len >= 8 && memcmp(cmd_buf, "SHOCK_ON", 8) == 0) ||
        (len >= 9 && memcmp(cmd_buf, "SHOCK_ARM", 9) == 0)) {
        if (s_shock_cb) s_shock_cb(true);
        return len;
    }
    if ((len >= 9 && memcmp(cmd_buf, "SHOCK_OFF", 9) == 0) ||
        (len >= 12 && memcmp(cmd_buf, "SHOCK_DISARM", 12) == 0)) {
        if (s_shock_cb) s_shock_cb(false);
        return len;
    }

    const char cmd = cmd_buf[0];
    switch (cmd) {
        case 'C':
        case '?':
            if (s_announce_cb) {
                s_announce_cb();
            }
            break;

        case 'A': {
            aed_mode_t mode = AED_MODE_HIP;
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

        /* Shock commands from App: Shock ON (Arm) / Shock OFF (Disarm) */
        case 'K': {
            bool arm = true;
            if (len >= 2 && cmd_buf[1] == '0') {
                arm = false;
            }
            if (s_shock_cb) {
                s_shock_cb(arm);
            }
            break;
        }

        case 'E': /* Enable / Arm Shock (ON) */
            if (s_shock_cb) {
                s_shock_cb(true);
            }
            break;

        case 'D': /* Disable / Disarm Shock (OFF) */
            if (s_shock_cb) {
                s_shock_cb(false);
            }
            break;

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
    if (!ble_is_app(conn)) return BT_GATT_ERR(BT_ATT_ERR_AUTHORIZATION);
    k_spinlock_key_t key = k_spin_lock(&cache_lock);
    ssize_t n = bt_gatt_attr_read(conn, attr, buf, len, offset, s_tx_buffer, s_tx_len);
    k_spin_unlock(&cache_lock, key);
    return n;
}

static void on_nus_tx_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_DBG("NUS TX CCC changed: %u", value);
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
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) || info.role != BT_CONN_ROLE_PERIPHERAL) return;
    if (err) {
        LOG_ERR("BLE Connection failed (err 0x%02x)", err);
        s_is_advertising = false;
        k_work_reschedule(&s_adv_work, K_MSEC(50));
        return;
    }
    k_work_cancel_delayable(&s_adv_work);
    k_spinlock_key_t key = k_spin_lock(&peer_lock);
    if (default_conn) {
        k_spin_unlock(&peer_lock, key);
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }
    default_conn = bt_conn_ref(conn);
    s_client_connected = true;
    k_spin_unlock(&peer_lock, key);
    s_is_advertising = false;
    LOG_INF(">>> BLE Client connected! (Just Works bonding active)");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    k_spinlock_key_t key = k_spin_lock(&peer_lock);
    if (conn != default_conn) {
        k_spin_unlock(&peer_lock, key);
        return;
    }
    default_conn = NULL;
    s_client_connected = false;
    k_spin_unlock(&peer_lock, key);
    bt_conn_unref(conn);
    LOG_INF("App disconnected (reason 0x%02x)", reason);
    s_is_advertising = false;

    if (s_heart_cb) s_heart_cb(HEART_STOPPED);
    if (s_aed_cb)   s_aed_cb(AED_MODE_DISABLED);
    if (s_air_cb)   s_air_cb(false);
    if (s_shock_cb) s_shock_cb(false);

    /* In Zephyr, the stack still holds a reference to the conn object inside disconnected().
     * Restarting advertising synchronously here can fail with -ENOMEM when connection slots are constrained.
     * We schedule advertising restart after a brief delay, and also hook into recycled().
     */
    k_work_reschedule(&s_adv_work, K_MSEC(50));
}

static void recycled(void)
{
    ble_service_ensure_advertising();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
    .recycled     = recycled,
};

void ble_service_register_callbacks(ble_heart_mode_callback_t heart_cb,
                                    ble_announce_request_callback_t announce_cb,
                                    ble_aed_mode_callback_t aed_cb,
                                    ble_air_mode_callback_t air_cb,
                                    ble_shock_mode_callback_t shock_cb)
{
    s_heart_cb    = heart_cb;
    s_announce_cb = announce_cb;
    s_aed_cb      = aed_cb;
    s_air_cb      = air_cb;
    s_shock_cb    = shock_cb;
}

int ble_service_init(void)
{
    k_work_init_delayable(&s_adv_work, adv_work_handler);

    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

#ifdef CONFIG_CPR_AED_BRIDGE
    err = settings_load();
    if (!err) err = aed_client_init();
    if (err) LOG_ERR("AED bridge unavailable (%d); app link remains enabled", err);
#endif
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

int ble_app_send(struct bt_conn *app, const void *data, uint16_t len)
{
    const struct bt_gatt_attr *tx = &nus_svc.attrs[4];
    if (!ble_is_app(app) || !bt_gatt_is_subscribed(app, tx, BT_GATT_CCC_NOTIFY)) return -ENOTCONN;
    if (len > sizeof(s_tx_buffer)) return -EMSGSIZE;
    k_spinlock_key_t key = k_spin_lock(&cache_lock);
    memcpy(s_tx_buffer, data, len);
    s_tx_len = len;
    k_spin_unlock(&cache_lock, key);
    return bt_gatt_notify(app, tx, data, len);
}

static int send_nus_data(const void *data, uint16_t len)
{
    struct bt_conn *app = ble_app_ref();
    if (!app) return -ENOTCONN;
    int err = ble_app_send(app, data, len);
    bt_conn_unref(app);
    return err;
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

int ble_service_send_string(const char *str)
{
    if (!str) return -EINVAL;
    return send_nus_data(str, (uint16_t)strlen(str));
}

void ble_service_set_battery_level(uint8_t percentage)
{
    bt_bas_set_battery_level(percentage);
}

