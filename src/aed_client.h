#pragma once

#include <zephyr/bluetooth/conn.h>
#include "cpr_config.h"

int aed_client_init(void);
ssize_t aed_client_command(struct bt_conn *app, const void *buf, uint16_t len, uint16_t offset);
struct bt_conn *ble_app_ref(void);
bool ble_is_app(struct bt_conn *conn);
int ble_app_send(struct bt_conn *app, const void *data, uint16_t len);
void cpr_relay_aed(struct bt_conn *app, int8_t state);
