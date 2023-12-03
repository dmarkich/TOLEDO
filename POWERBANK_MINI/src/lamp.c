#include <lamp.h>

#define MIN_PERIOD PWM_SEC(1U) / 128U
#define MAX_PERIOD PWM_SEC(1U)

// int set_lamp(uint16_t state) {
//     int ret;
//     ret = gpio_pin_set_dt(&led_lamp1, state);
//     if (ret < 0) {
//         return 0;
//     }
//     ret = gpio_pin_set_dt(&led_lamp2, state);
//     if (ret < 0) {
//         return 0;
//     }
//     ret = gpio_pin_set_dt(&led_lamp3, state);
//     if (ret < 0) {
//         return 0;
//     }
//     ret = gpio_pin_set_dt(&led_lamp4, state);
//     if (ret < 0) {
//         return 0;
//     }
//     return ret;
// }

// int init_lamp_leds(void) {
// 	int ret;
// 	if (
// 		!gpio_is_ready_dt(&led_lamp1) &&
// 		!gpio_is_ready_dt(&led_lamp2) &&
// 		!gpio_is_ready_dt(&led_lamp3) &&
// 		!gpio_is_ready_dt(&led_lamp4)
// 	) {
// 		return 0;
// 	}

// 	ret = gpio_pin_configure_dt(&led_lamp1, GPIO_OUTPUT_ACTIVE);
// 	if (ret < 0) {
// 		return 0;
// 	}
// 	ret = gpio_pin_configure_dt(&led_lamp2, GPIO_OUTPUT_ACTIVE);
// 	if (ret < 0) {
// 		return 0;
// 	}
// 	ret = gpio_pin_configure_dt(&led_lamp3, GPIO_OUTPUT_ACTIVE);
// 	if (ret < 0) {
// 		return 0;
// 	}
// 	ret = gpio_pin_configure_dt(&led_lamp4, GPIO_OUTPUT_ACTIVE);
// 	if (ret < 0) {
// 		return 0;
// 	}
// 	return ret;
// }

int init_lamp_leds_pwm(void) {
	if (!pwm_is_ready_dt(&led_lamp1)) {
		printk("Error: PWM device %s is not ready\n",
		       led_lamp1.dev->name);
		return 0;
	}
	if (!pwm_is_ready_dt(&led_lamp2)) {
		printk("Error: PWM device %s is not ready\n",
		       led_lamp2.dev->name);
		return 0;
	}
	if (!pwm_is_ready_dt(&led_lamp3)) {
		printk("Error: PWM device %s is not ready\n",
		       led_lamp3.dev->name);
		return 0;
	}
	if (!pwm_is_ready_dt(&led_lamp4)) {
		printk("Error: PWM device %s is not ready\n",
		       led_lamp4.dev->name);
		return 0;
	}
	return 1;
}

int set_lamp_pwm() {
	uint32_t max_period;
	uint32_t period;
	uint8_t dir = 0U;
	int ret;

	init_lamp_leds_pwm();

	printk("Calibrating for channel %d...\n", led_lamp1.channel);
	max_period = MAX_PERIOD;
	while (pwm_set_dt(&led_lamp1, max_period, max_period / 2U)) {
		max_period /= 2U;
		if (max_period < (4U * MIN_PERIOD)) {
			printk("Error: PWM device "
			       "does not support a period at least %lu\n",
			       4U * MIN_PERIOD);
			return 0;
		}
	}

	printk("Done calibrating; maximum/minimum periods %u/%lu nsec\n",
	       max_period, MIN_PERIOD);

	period = max_period;
	while (1) {
		ret = pwm_set_dt(&led_lamp1, period, period / 2U);
		if (ret) {
			printk("Error %d: failed to set pulse width\n", ret);
			return 0;
		}

		period = dir ? (period * 2U) : (period / 2U);
		if (period > max_period) {
			period = max_period / 2U;
			dir = 0U;
		} else if (period < MIN_PERIOD) {
			period = MIN_PERIOD * 2U;
			dir = 1U;
		}

		k_sleep(K_SECONDS(4U));
	}
	return 0;
}