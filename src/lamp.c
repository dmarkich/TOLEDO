#include <lamp.h>

int set_lamp(uint16_t state) {
    int ret;
    ret = gpio_pin_set_dt(&led_lamp1, state);
    if (ret < 0) {
        return 0;
    }
    ret = gpio_pin_set_dt(&led_lamp2, state);
    if (ret < 0) {
        return 0;
    }
    ret = gpio_pin_set_dt(&led_lamp3, state);
    if (ret < 0) {
        return 0;
    }
    ret = gpio_pin_set_dt(&led_lamp4, state);
    if (ret < 0) {
        return 0;
    }
    return ret;
}

int init_lamp_leds(void) {
	int ret;
	if (
		!gpio_is_ready_dt(&led_lamp1) &&
		!gpio_is_ready_dt(&led_lamp2) &&
		!gpio_is_ready_dt(&led_lamp3) &&
		!gpio_is_ready_dt(&led_lamp4)
	) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led_lamp1, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&led_lamp2, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&led_lamp3, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&led_lamp4, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	return ret;
}