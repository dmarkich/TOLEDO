#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

#ifndef LAMP_H
#define LAMP_H

#define LED_OUT1 DT_ALIAS(led_out1)
#define LED_OUT2 DT_ALIAS(led_out2)
#define LED_OUT3 DT_ALIAS(led_out3)
#define LED_OUT4 DT_ALIAS(led_out4)

// static const struct gpio_dt_spec led_lamp1 = GPIO_DT_SPEC_GET(LED_OUT1, gpios);
// static const struct gpio_dt_spec led_lamp2 = GPIO_DT_SPEC_GET(LED_OUT2, gpios);
// static const struct gpio_dt_spec led_lamp3 = GPIO_DT_SPEC_GET(LED_OUT3, gpios);
// static const struct gpio_dt_spec led_lamp4 = GPIO_DT_SPEC_GET(LED_OUT4, gpios);

static const struct pwm_dt_spec led_lamp1 = PWM_DT_SPEC_GET(LED_OUT1);
static const struct pwm_dt_spec led_lamp2 = PWM_DT_SPEC_GET(LED_OUT2);
static const struct pwm_dt_spec led_lamp3 = PWM_DT_SPEC_GET(LED_OUT3);
static const struct pwm_dt_spec led_lamp4 = PWM_DT_SPEC_GET(LED_OUT4);



// int set_lamp(uint16_t state);
// int init_lamp_leds(void);
int init_lamp_leds_pwm(void);
int fade_lamp(void);

#endif
