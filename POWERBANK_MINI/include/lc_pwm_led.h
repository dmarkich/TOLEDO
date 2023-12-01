/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file
 * @brief Lc pwm led module
 */




#ifndef LC_PWM_LED_H__
#define LC_PWM_LED_H__

#include <nrf.h>
#include <nrfx.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <nrfx_ppi.h>
#include <zephyr/types.h>
#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"
#include <zephyr/drivers/pwm.h>

#ifdef __cplusplus
extern "C" {
#endif

void pwm_init(void);

void lc_pwm_led_init(void);

void lc_pwm_led_set(uint16_t desired_lvl);

#ifdef __cplusplus
}
#endif

#endif /* LC_PWM_LED_H__ */
