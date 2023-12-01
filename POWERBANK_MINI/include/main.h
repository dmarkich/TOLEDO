
#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <bluetooth/mesh/dk_prov.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"
#include "lc_pwm_led.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>

#include <soc.h>
#include <zephyr/types.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <limits.h>

#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
// #include <zephyr/drivers/pwm.h>
#include <nrfx_pwm.h>

#include "string.h"

#include <stdbool.h>
#include <stdint.h>

// #include "nrf_delay.h"
// #include "nrf_log.h"
// #include "nrf_log_ctrl.h"
// #include "nrf_log_default_backends.h"


#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include "zephyr/sys/util.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <battery_sensor.h>
#include <lc_pwm_led.h>


#define PWM_PERIOD 1024
/*
#define PWM_LED0_NODE DT_ALIAS(&pwmleds)

#if DT_NODE_HAS_STATUS(PWM_LED0_NODE, okay)
static const struct pwm_dt_spec led0 = PWM_DT_SPEC_GET(PWM_LED0_NODE);
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#endif
*/



#define PWM_COUNTERTOP 100

#define STACKSIZE 1024
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

// #define I2C0_NODE1 DT_NODELABEL(mp2722)

#define STORAGE_NODE_LABEL storage
#define LONG_ID 1

/*
#define ADC_CHANNELS_IN_USE 8
#define SAADC_BUF_SIZE ADC_CHANNELS_IN_USE
#define SAADC_BUF_COUNT 2
#define SAADC_SAMPLE_FREQUENCY 8000


static nrf_saadc_value_t samples[SAADC_BUF_COUNT][SAADC_BUF_SIZE];
static nrfx_timer_t m_sample_timer = NRFX_TIMER_INSTANCE(1);
static nrf_ppi_channel_t m_timer_saadc_ppi_channel;
static nrf_ppi_channel_t m_saadc_internal_ppi_channel;
static const uint32_t saadc_sampling_rate = 1000; // milliseconds (ms)

static const nrf_saadc_input_t ANALOG_INPUT_MAP[ADC_CHANNELS_IN_USE] = {
	NRF_SAADC_INPUT_AIN0, NRF_SAADC_INPUT_AIN1, NRF_SAADC_INPUT_AIN2, NRF_SAADC_INPUT_AIN3,
	NRF_SAADC_INPUT_AIN4, NRF_SAADC_INPUT_AIN5, NRF_SAADC_INPUT_AIN6, NRF_SAADC_INPUT_AIN7};
*/

