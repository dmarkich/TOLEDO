#include "zephyr/sys/util.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/fuel_gauge.h>

#ifndef FUEL_GAUGE_H
#define FUEL_GAUGE_H

int read_gauge(void);

#endif