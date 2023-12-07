
#include <zephyr/drivers/gpio.h>

#ifndef STATUS_LED_H
#define STATUS_LED_H

#define LED_FRONT1 DT_ALIAS(led_front1)
#define LED_FRONT2 DT_ALIAS(led_front2)
#define LED_FRONT3 DT_ALIAS(led_front3)

static const struct gpio_dt_spec led_front1 = GPIO_DT_SPEC_GET(LED_FRONT1, gpios);
static const struct gpio_dt_spec led_front2 = GPIO_DT_SPEC_GET(LED_FRONT2, gpios);
static const struct gpio_dt_spec led_front3 = GPIO_DT_SPEC_GET(LED_FRONT3, gpios);

int init_front_leds(void);
int set_led_status(int level);
int set_led_status_state(int first, int second, int third);

#endif