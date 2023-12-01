

#ifndef BT_H
#define BT_H

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/mesh/models.h>
#include <bluetooth/mesh/dk_prov.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"
#include "lc_pwm_led.h"

static K_SEM_DEFINE(ble_init_ok, 0, 1);

void bt_ready(int err);
void ble_write_thread(void);

#endif