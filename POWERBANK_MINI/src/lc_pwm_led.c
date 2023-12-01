/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */


#include <lc_pwm_led.h>

// #define PWM_LED0_NODE	DT_ALIAS()

// #if DT_NODE_HAS_STATUS(PWM_LED0_NODE, okay)
// static const struct pwm_dt_spec led0 = PWM_DT_SPEC_GET(PWM_LED0_NODE);
// #else
// #error "Unsupported board: pwm-led0 devicetree alias is not defined"
// #endif

#define PWM_PERIOD 1024

// static nrfx_pwm_t my_pwm = NRFX_PWM_INSTANCE(1);

void pwm_init(void)
{
	// static nrfx_pwm_config_t config0 =
	// 	{
	// 		.output_pins =
	// 			{
	// 				45,	// channel 0
	// 				46,	// channel 1
	// 				42, // channel 2
	// 				44  // channel 3
	// 			},
	// 		.irq_priority = 5,
	// 		.base_clock = NRF_PWM_CLK_1MHz,
	// 		.count_mode = NRF_PWM_MODE_UP,
	// 		.top_value = PWM_COUNTERTOP,
	// 		.load_mode = NRF_PWM_LOAD_INDIVIDUAL,
	// 		.step_mode = NRF_PWM_STEP_AUTO};
	// return (nrfx_pwm_init(&my_pwm, &config0, NULL, NULL) == NRFX_SUCCESS) ? 0 : -1;

	// If PWM callbacks are to be used, remember to configure the interrupts correctly
	// IRQ_DIRECT_CONNECT(PWM1_IRQn, 0, nrfx_pwm_1_irq_handler, 0);
	// irq_enable(PWM1_IRQn);
}

void lc_pwm_led_set(uint16_t desired_lvl)
{
	// uint32_t scaled_lvl = (PWM_PERIOD * desired_lvl) / BT_MESH_LIGHTNESS_MAX; // add gamma correction

	// // pwm_set_dt(&pwm0, PWM_USEC(PWM_PERIOD), PWM_USEC(scaled_lvl));

	// static bool pwm_running = false;

	// // This array cannot be allocated on stack (hence "static") and it must be in RAM
	// static nrf_pwm_values_individual_t seq_values;

	// // Update the respective channels
	// seq_values.channel_0 = (scaled_lvl <= PWM_COUNTERTOP) ? scaled_lvl : PWM_COUNTERTOP;

	// nrf_pwm_sequence_t const seq =
	// 	{
	// 		.values.p_individual = &seq_values,
	// 		.length = NRF_PWM_VALUES_LENGTH(seq_values),
	// 		.repeats = 0,
	// 		.end_delay = 0};

	// if (!pwm_running)
	// {
	// 	pwm_running = true;
	// 	(void)nrfx_pwm_simple_playback(&my_pwm, &seq, 1000, NRFX_PWM_FLAG_LOOP);
	// }
}