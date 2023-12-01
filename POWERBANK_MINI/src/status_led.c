#include <status_led.h>

int init_front_leds(void) {
	int ret = 1;
	if (!gpio_is_ready_dt(&led_front1) && !gpio_is_ready_dt(&led_front2) && !gpio_is_ready_dt(&led_front3)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led_front1, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&led_front2, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&led_front3, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	ret = gpio_pin_set_dt(&led_front1, 0);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_set_dt(&led_front2, 0);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_set_dt(&led_front3, 0);
	if (ret < 0) {
		return 0;
	}

	return ret;
}

int set_led_status(int bat_level) {
    int ret = 1;

    if (bat_level >= 0 && bat_level <= 30) {
        ret = gpio_pin_set_dt(&led_front1, 1);
        if (ret < 0) {
            return 0;
        }
        ret = gpio_pin_set_dt(&led_front2, 0);
        if (ret < 0) {
            return 0;
        }
        ret = gpio_pin_set_dt(&led_front3, 0);
        if (ret < 0) {
            return 0;
        }
    } else if (bat_level > 30 && bat_level <= 60) {
        ret = gpio_pin_set_dt(&led_front1, 1);
        if (ret < 0) {
            return 0;
        }
        ret = gpio_pin_set_dt(&led_front2, 1);
        if (ret < 0) {
            return 0;
        }
        ret = gpio_pin_set_dt(&led_front3, 0);
        if (ret < 0) {
            return 0;
        }
    } else if (bat_level > 60 && bat_level <= 100) {
        ret = gpio_pin_set_dt(&led_front1, 1);
        if (ret < 0) {
            return 0;
        }
        ret = gpio_pin_set_dt(&led_front2, 1);
        if (ret < 0) {
            return 0;
        }
        ret = gpio_pin_set_dt(&led_front3, 1);
        if (ret < 0) {
            return 0;
        }
    }
    return ret;
}