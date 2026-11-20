#include "aed_client.h"
#include "aed_wire.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(aed_client, LOG_LEVEL_INF);
BUILD_ASSERT(DT_NODE_EXISTS(DT_NODELABEL(storage_partition)), "Bridge requires settings storage");

/* The bridge thread owns all connection procedures and assignment state. BLE
 * callbacks enqueue small snapshots, never run flash operations or app TX. */
enum kind { COMMAND, UP, DOWN, SERVICE, CHARACTER, CCC, END, SUBSCRIBED, FRAME, LOST };
struct message {
    enum kind kind;
    struct bt_conn *conn, *app;
    uint16_t handle, end;
    uint8_t data[14], len;
};
K_MSGQ_DEFINE(inbox, sizeof(struct message), 24, sizeof(void *));
K_SEM_DEFINE(started, 0, 1);
static atomic_t enabled;
static atomic_t dropped;
static struct bt_conn *peer, *owner;
static bt_addr_le_t target, saved;
static bool assigned, persisted;
static struct bt_gatt_discover_params discovery;
static struct bt_gatt_subscribe_params subscription;
static struct aed_stream stream;
static enum { IDLE, CONNECTING, SECURITY, FIND_SERVICE, FIND_CHARS, FIND_CCC, SUB, LIVE, CLOSING } phase;
static uint16_t service_end, rx, tx, tx_end;
static int64_t deadline, retry_at, hello_deadline;
static unsigned backoff = 1;
static const char *status = "@UNASSIGNED";

#define CANDIDATES 8
static bt_addr_le_t candidates[CANDIDATES];
static uint32_t token_base;
static unsigned count;
static struct k_spinlock scan_lock;
static bool scanning;
static int64_t scan_end, expires;

static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(BT_UUID_NUS_SERVICE_VAL);
static struct bt_uuid_128 rx_uuid = BT_UUID_INIT_128(BT_UUID_NUS_RX_VAL);
static struct bt_uuid_128 tx_uuid = BT_UUID_INIT_128(BT_UUID_NUS_TX_VAL);

static bool enqueue(struct message m)
{
    if (m.conn) bt_conn_ref(m.conn);
    if (!k_msgq_put(&inbox, &m, K_NO_WAIT)) return true;
    if (m.conn) bt_conn_unref(m.conn);
    if (m.app) bt_conn_unref(m.app);
    atomic_inc(&dropped);
    return false;
}

static int restore(const char *key, size_t len, settings_read_cb read, void *arg)
{
    if (strcmp(key, "peer")) return -ENOENT;
    if (len != sizeof(saved) || read(arg, &saved, len) != (ssize_t)len) return -EINVAL;
    if (saved.type != BT_ADDR_LE_PUBLIC && !(saved.type == BT_ADDR_LE_RANDOM && BT_ADDR_IS_STATIC(&saved.a))) return -EINVAL;
    target = saved;
    persisted = assigned = true;
    return 0;
}
SETTINGS_STATIC_HANDLER_DEFINE(aed_assignment, "trainer", NULL, restore, NULL, NULL);

static void reply(const char *s)
{
    if (owner && ble_is_app(owner)) ble_app_send(owner, s, strlen(s));
}
static void report(const char *s) { status = s; reply(s); }

static bool mfg(struct bt_data *d, void *arg)
{
    if (d->type == BT_DATA_MANUFACTURER_DATA && d->data_len == 3 &&
        d->data[0] == 0xc4 && d->data[1] == 0xc9 && d->data[2] == 0) {
        *(bool *)arg = true;
        return false;
    }
    return true;
}
static void advertisement(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
    ARG_UNUSED(rssi);
    if (type != BT_GAP_ADV_TYPE_ADV_IND) return;
    bool available = false;
    bt_data_parse(ad, mfg, &available);
    if (!available) return;
    k_spinlock_key_t k = k_spin_lock(&scan_lock);
    if (scanning && count < CANDIDATES) {
        unsigned i = 0;
        while (i < count && bt_addr_le_cmp(addr, &candidates[i])) i++;
        if (i == count) candidates[count++] = *addr;
    }
    k_spin_unlock(&scan_lock, k);
}
static void stop_scan(void)
{
    k_spinlock_key_t k = k_spin_lock(&scan_lock);
    bool stop = scanning;
    scanning = false;
    k_spin_unlock(&scan_lock, k);
    if (stop) bt_le_scan_stop();
}

static bool central(struct bt_conn *c)
{
    struct bt_conn_info i;
    return !bt_conn_get_info(c, &i) && i.role == BT_CONN_ROLE_CENTRAL;
}
static void connected(struct bt_conn *c, uint8_t err)
{
    if (central(c)) enqueue((struct message){ .kind = err ? DOWN : UP, .conn = c });
}
static void disconnected(struct bt_conn *c, uint8_t reason)
{
    ARG_UNUSED(reason);
    if (central(c)) enqueue((struct message){ .kind = DOWN, .conn = c });
}
BT_CONN_CB_DEFINE(aed_connections) = { .connected = connected, .disconnected = disconnected };

static uint8_t found(struct bt_conn *c, const struct bt_gatt_attr *attr, struct bt_gatt_discover_params *p)
{
    struct message m = { .conn = c, .kind = END };
    if (!attr) { enqueue(m); return BT_GATT_ITER_STOP; }
    m.handle = attr->handle;
    if (p->type == BT_GATT_DISCOVER_PRIMARY) {
        m.kind = SERVICE;
        m.end = ((struct bt_gatt_service_val *)attr->user_data)->end_handle;
    } else if (p->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        const struct bt_gatt_chrc *ch = attr->user_data;
        m.kind = CHARACTER;
        m.end = ch->value_handle;
        m.data[0] = !bt_uuid_cmp(ch->uuid, &rx_uuid.uuid) &&
            (ch->properties & (BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP)) ? 1 :
            !bt_uuid_cmp(ch->uuid, &tx_uuid.uuid) && (ch->properties & BT_GATT_CHRC_NOTIFY) ? 2 : 0;
        return enqueue(m) ? BT_GATT_ITER_CONTINUE : BT_GATT_ITER_STOP;
    } else { m.kind = CCC; }
    enqueue(m);
    return BT_GATT_ITER_STOP;
}
static int discover(uint8_t type, const struct bt_uuid *uuid, uint16_t first, uint16_t last)
{
    discovery = (struct bt_gatt_discover_params){ .uuid = uuid, .func = found, .type = type,
        .start_handle = first, .end_handle = last };
    return bt_gatt_discover(peer, &discovery);
}
static uint8_t notified(struct bt_conn *c, struct bt_gatt_subscribe_params *p, const void *data, uint16_t len)
{
    ARG_UNUSED(p);
    if (!c) return BT_GATT_ITER_STOP;
    if (!data) { enqueue((struct message){ .kind = LOST, .conn = c }); return BT_GATT_ITER_STOP; }
    if (len == 5) {
        struct message m = { .kind = FRAME, .conn = c, .app = ble_app_ref(), .len = len };
        memcpy(m.data, data, len);
        enqueue(m);
    }
    return BT_GATT_ITER_CONTINUE;
}
static void subscribed(struct bt_conn *c, uint8_t err, struct bt_gatt_subscribe_params *p)
{
    ARG_UNUSED(p);
    enqueue((struct message){ .kind = err ? LOST : SUBSCRIBED, .conn = c });
}
static void close_peer(const char *why)
{
    phase = CLOSING;
    report(why);
    if (peer) bt_conn_disconnect(peer, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}
static void check_bond(const struct bt_bond_info *b, void *arg)
{
    if (!bt_addr_le_cmp(&b->addr, bt_conn_get_dst(peer))) *(bool *)arg = true;
}
static void save_peer(void)
{
    bool bonded = false;
    bt_foreach_bond(BT_ID_DEFAULT, check_bond, &bonded);
    const bt_addr_le_t *id = bt_conn_get_dst(peer);
    if (!bonded || (id->type != BT_ADDR_LE_PUBLIC && !(id->type == BT_ADDR_LE_RANDOM && BT_ADDR_IS_STATIC(&id->a)))) {
        reply("@SESSION_ONLY"); return;
    }
    if (!persisted && settings_save_one("trainer/peer", id, sizeof(*id))) { reply("@STORE_ERROR"); return; }
    persisted = true;
    target = saved = *id;
}

static void handle_peer(const struct message *m)
{
    if (!peer || m->conn != peer) return;
    if (m->kind == DOWN) {
        bt_conn_unref(peer); peer = NULL; phase = IDLE;
        stream = (struct aed_stream){0};
        retry_at = k_uptime_get() + 1000 * backoff;
        backoff = MIN(backoff * 2, 32U);
        report(assigned ? "@DISCONNECTED" : "@UNASSIGNED");
        return;
    }
    if (phase == CLOSING) return;
    int err = 0;
    switch (m->kind) {
    case UP:
        if (phase != CONNECTING) return;
        phase = SECURITY; report("@PAIRING");
        err = bt_conn_set_security(peer, BT_SECURITY_L2);
        break;
    case SERVICE:
        if (phase != FIND_SERVICE) return;
        phase = FIND_CHARS; service_end = m->end; rx = tx = 0; tx_end = service_end;
        err = discover(BT_GATT_DISCOVER_CHARACTERISTIC, NULL, m->handle + 1, service_end);
        break;
    case CHARACTER:
        if (phase != FIND_CHARS) return;
        if (tx && m->handle > tx && tx_end == service_end) tx_end = m->handle - 1;
        if (m->data[0] == 1) rx = m->end;
        if (m->data[0] == 2 && !tx) tx = m->end;
        break;
    case END:
        if (phase != FIND_CHARS || !tx || !rx || tx >= tx_end) { close_peer("@BAD_SERVICE"); return; }
        phase = FIND_CCC;
        err = discover(BT_GATT_DISCOVER_DESCRIPTOR, BT_UUID_GATT_CCC, tx + 1, tx_end);
        break;
    case CCC:
        if (phase != FIND_CCC) return;
        phase = SUB;
        subscription = (struct bt_gatt_subscribe_params){ .notify = notified, .subscribe = subscribed,
            .ccc_handle = m->handle, .value_handle = tx, .value = BT_GATT_CCC_NOTIFY };
        atomic_set_bit(subscription.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);
        err = bt_gatt_subscribe(peer, &subscription);
        break;
    case SUBSCRIBED:
        if (phase != SUB) return;
        phase = LIVE; backoff = 1; save_peer(); report("@SUBSCRIBED");
        hello_deadline = k_uptime_get() + 3000;
        break;
    case FRAME:
        if (phase != LIVE && phase != SUB) return;
        switch (aed_decode(&stream, m->data, m->len)) {
        case AED_HELLO: report("@TELEMETRY_V1"); break;
        case AED_POWER_OFF: case AED_POWER_ON: case AED_SIM_DELIVERED:
            if (m->app && ble_is_app(m->app)) {
                int8_t state = m->data[2] == AED_POWER_OFF ? AED_STATE_OFF :
                    m->data[2] == AED_POWER_ON ? AED_STATE_ON : AED_STATE_SHOCK_DELIVERED;
                cpr_relay_aed(m->app, state);
            }
            break;
        }
        break;
    case LOST: close_peer("@TX_LOST"); break;
    default: break;
    }
    if (err) close_peer("@LINK_ERROR");
}

static bool token_index(const uint8_t *p, unsigned *index)
{
    uint32_t token = 0;
    for (int i = 0; i < 8; i++) {
        unsigned c = p[i];
        if (c >= '0' && c <= '9') c -= '0';
        else if (c >= 'A' && c <= 'F') c = c - 'A' + 10;
        else return false;
        token = (token << 4) | c;
    }
    *index = token - token_base;
    return !scanning && expires > k_uptime_get() && *index < count;
}
static void forget_owner(void)
{
    stop_scan(); expires = 0;
    if (owner) bt_conn_unref(owner);
    owner = NULL;
}
static void command(const struct message *m)
{
    if (!ble_is_app(m->conn)) return;
    if (owner != m->conn) { forget_owner(); owner = bt_conn_ref(m->conn); }
    const char *cmd = (const char *)m->data;
    if (!strcmp(cmd, "@GET")) {
        reply(status);
        if (assigned) {
            char line[21];
            snprintk(line, sizeof(line), "@PEER:%02X%02X%02X:%c", target.a.val[2], target.a.val[1], target.a.val[0], persisted ? 'B' : 'S');
            reply(line);
        }
        return;
    }
    if (!strcmp(cmd, "@LIST")) {
        if (peer && phase != LIVE) { reply("@BUSY"); return; }
        stop_scan(); expires = 0; token_base += CANDIDATES;
        k_spinlock_key_t k = k_spin_lock(&scan_lock);
        count = 0; scanning = true;
        k_spin_unlock(&scan_lock, k);
        if (bt_le_scan_start(BT_LE_SCAN_PASSIVE, advertisement)) { stop_scan(); reply("@SCAN_ERROR"); return; }
        scan_end = k_uptime_get() + 4000; reply("@SCANNING"); return;
    }
    bool clear = !strcmp(cmd, "@CLEAR");
    unsigned idx = 0;
    if (!clear && (!token_index(m->data + 5, &idx) || strncmp(cmd, "@SET:", 5))) { reply("@STALE"); return; }
    if (!clear && assigned && !bt_addr_le_cmp(&target, &candidates[idx])) { reply(status); return; }
    if (persisted) {
        if (settings_delete("trainer/peer")) { reply("@STORE_ERROR"); return; }
        if (bt_addr_le_cmp(&saved, bt_conn_get_dst(owner)) && bt_unpair(BT_ID_DEFAULT, &saved)) reply("@UNPAIR_ERROR");
    }
    stop_scan(); expires = 0;
    persisted = false; assigned = !clear; backoff = 1; retry_at = 0;
    if (!clear) target = candidates[idx];
    if (peer) close_peer("@RELEASING");
    else report(clear ? "@UNASSIGNED" : "@ASSIGNED");
}

ssize_t aed_client_command(struct bt_conn *app, const void *buf, uint16_t len, uint16_t offset)
{
    if (!atomic_get(&enabled)) return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    if (bt_conn_get_security(app) < BT_SECURITY_L2) return BT_GATT_ERR(BT_ATT_ERR_AUTHENTICATION);
    if (offset) return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    const char *p = buf;
    if (!((len == 4 && !memcmp(p,"@GET",4)) || (len == 5 && !memcmp(p,"@LIST",5)) ||
          (len == 6 && !memcmp(p,"@CLEAR",6)) || (len == 13 && !memcmp(p,"@SET:",5))))
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    struct message m = { .kind = COMMAND, .conn = app };
    memcpy(m.data, p, len);
    return enqueue(m) ? len : BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
}

static void tick(void)
{
    int64_t now = k_uptime_get();
    if (owner && !ble_is_app(owner)) forget_owner();
    if (atomic_set(&dropped, 0)) reply("@BRIDGE_OVERFLOW");
    if (scanning && now >= scan_end) {
        stop_scan(); expires = now + 30000;
        for (unsigned i = 0; i < count; i++) {
            char line[21]; const uint8_t *a = candidates[i].a.val;
            snprintk(line, sizeof(line), "@%08X:%02X%02X%02X", (unsigned)(token_base + i), a[2], a[1], a[0]);
            reply(line);
        }
        reply("@LIST_END");
    }
    if (!peer && assigned && !scanning && now >= retry_at) {
        if (bt_conn_le_create(&target, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &peer)) {
            peer = NULL; retry_at = now + 1000 * backoff; backoff = MIN(32U, backoff * 2);
            report("@CONNECT_FAILED"); return;
        }
        phase = CONNECTING; deadline = now + 30000; report("@CONNECTING");
    }
    if (!peer) return;
    struct bt_conn_info info;
    if (!bt_conn_get_info(peer, &info) && info.state == BT_CONN_STATE_DISCONNECTED) {
        handle_peer(&(struct message){.conn = peer, .kind = DOWN}); return;
    }
    if (phase == SECURITY && bt_conn_get_security(peer) >= BT_SECURITY_L2) {
        phase = FIND_SERVICE; report("@SECURED");
        if (discover(BT_GATT_DISCOVER_PRIMARY, &service_uuid.uuid, 1, UINT16_MAX)) close_peer("@DISCOVERY_ERROR");
    }
    if (phase == LIVE && !stream.ready && hello_deadline && now >= hello_deadline) {
        hello_deadline = 0; report("@NO_TELEMETRY");
    }
    if (phase != LIVE && phase != CLOSING && now >= deadline) close_peer("@TIMEOUT");
}
static void worker(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    k_sem_take(&started, K_FOREVER);
    for (;;) {
        struct message m;
        if (!k_msgq_get(&inbox, &m, K_MSEC(100))) {
            if (m.kind == COMMAND) command(&m); else handle_peer(&m);
            if (m.conn) bt_conn_unref(m.conn);
            if (m.app) bt_conn_unref(m.app);
        }
        tick();
    }
}
K_THREAD_DEFINE(aed_client_tid, 3072, worker, NULL, NULL, NULL, 7, 0, 0);
int aed_client_init(void)
{
    atomic_set(&enabled, 1);
    k_sem_give(&started);
    return 0;
}
