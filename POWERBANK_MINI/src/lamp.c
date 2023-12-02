#include <lamp.h>

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






#define NUM_STEPS	50U
#define SLEEP_MSEC	25U

int fade_lamp(void)
{
	uint32_t pulse_width = 0U;
	uint32_t step = led_lamp1.period / NUM_STEPS;
	uint8_t dir = 1U;
	int ret;

	printk("PWM-based LED fade\n");

	init_lamp_leds_pwm();
	

	while (1) { 
		ret = pwm_set_pulse_dt(&led_lamp1, pulse_width);
		if (ret) {
			printk("Error %d: failed to set pulse width\n", ret);
			return 0;
		}
		ret = pwm_set_pulse_dt(&led_lamp2, pulse_width);
		if (ret) {
			printk("Error %d: failed to set pulse width\n", ret);
			return 0;
		}
		ret = pwm_set_pulse_dt(&led_lamp3, pulse_width);
		if (ret) {
			printk("Error %d: failed to set pulse width\n", ret);
			return 0;
		}
		ret = pwm_set_pulse_dt(&led_lamp4, pulse_width);
		if (ret) {
			printk("Error %d: failed to set pulse width\n", ret);
			return 0;
		}

		if (dir) {
			pulse_width += step;
			if (pulse_width >= led_lamp1.period) {
				pulse_width = led_lamp1.period - step;
				dir = 0U;
			}
		} else {
			if (pulse_width >= step) {
				pulse_width -= step;
			} else {
				pulse_width = step;
				dir = 1U;
			}
		}

		k_sleep(K_MSEC(SLEEP_MSEC));
	}
	return 0;
}
