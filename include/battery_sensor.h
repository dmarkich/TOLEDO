#include "zephyr/sys/util.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/fuel_gauge.h>

#ifndef FUEL_GAUGE_H
#define FUEL_GAUGE_H

struct battery_info
{
    int voltage;
    uint32_t runtime_to_empty;
    uint32_t runtime_to_full;
    uint8_t relative_state_of_charge;
};
extern struct battery_info device_battery;

int read_gauge(struct battery_info *bat_info);

#endif