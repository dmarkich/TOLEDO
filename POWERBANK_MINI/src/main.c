/*

 TOLEDO BT MESH PROTOTYPE - NRF52840


 https://docs.zephyrproject.org/latest/samples/fuel_gauge/max17048/README.html
 

 */

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

#include <device.h>
#include <drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
// #include <zephyr/drivers/pwm.h>
#include <nrfx_pwm.h>

#include "string.h"

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"

/*#include "nrf_drv_saadc.h"
#include <nrfx_saadc.h>
#include "nrfx_timer.h"
#include "nrfx_ppi.h"
*/

// #include "nrf_delay.h"
// #include "nrf_log.h"
// #include "nrf_log_ctrl.h"
// #include "nrf_log_default_backends.h"

#include "MP2722.h"

#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <fs/nvs.h>


#define PWM_PERIOD 1024
/*
#define PWM_LED0_NODE DT_ALIAS(&pwmleds)

#if DT_NODE_HAS_STATUS(PWM_LED0_NODE, okay)
static const struct pwm_dt_spec led0 = PWM_DT_SPEC_GET(PWM_LED0_NODE);
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#endif
*/
static nrfx_pwm_t my_pwm = NRFX_PWM_INSTANCE(1);


#define PWM_COUNTERTOP 100
#define NRFX_PPI_ENABLED 1


#define STACKSIZE CONFIG_BT_MESH_ADV_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define I2C0_NODE1 DT_NODELABEL(mp2722)

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




#define MAX17262 DT_INST(0, maxim_max17262)

#if DT_NODE_HAS_STATUS(MAX17262, okay)
#define MAX17262_LABEL DT_LABEL(MAX17262)
#else
#error Your devicetree has no enabled nodes with compatible "maxim,max17262"
#define MAX17262_LABEL "<none>"
#endif





static K_SEM_DEFINE(ble_init_ok, 0, 1);

static const struct i2c_dt_spec dev_i2cpsu = I2C_DT_SPEC_GET(I2C0_NODE1);

int16_t adc_psu_stat;
int16_t adc_psu_vpd;
int16_t adc_vbat;
int16_t adc_led_det1;
int16_t adc_led_det2;
int16_t adc_led_det3;
int16_t adc_led_det4;
int16_t adc_led_det5;

uint8_t psuwr[2];


bool setPSUreg(uint8_t reg, uint8_t val)
{
	psuwr[0] = reg;
	psuwr[1] = val;
	int err = i2c_write_dt(&dev_i2cpsu, &psuwr, 2);
	if (err != 0)
	{
		printk("ERROR WRITING TO PSU REGISTER %x, VALUE  %x\n\r", reg, val);
		return false;
	}

	return true;
}


void button1_pressed(const struct device *dev, struct gpio_callback *cb,
					uint32_t pins)
{
	
	printk("BUTTON 1 PRESSED\n");

}

void button2_pressed(const struct device *dev, struct gpio_callback *cb,
					 uint32_t pins)
{

	printk("BUTTON 2 PRESSED\n");
}

/*
static uint32_t next_free_buf_index(void)
{
	static uint32_t buffer_index = -1;
	buffer_index = (buffer_index + 1) % SAADC_BUF_COUNT;
	return buffer_index;
}

static void timer_handler(nrf_timer_event_t event_type, void *p_context)
{
}

*/

/*
static void adcevent_handler(nrfx_saadc_evt_t const *p_event)
{
	int err_code;
	switch (p_event->type)
	{
	case NRFX_SAADC_EVT_DONE:
		printk("BATTERY CURRENT = %i\nPV VOLTAGE = %i\nVBAT = %i\nLED DRIVER 1 FB = %i\nLED DRIVER 2 FB = %i\nLED DRIVER 3 FB = %i\nLED DRIVER 4 FB = %i\nLED DRIVER 5 FB = %i\n", p_event->data.done.p_buffer[0], p_event->data.done.p_buffer[1], p_event->data.done.p_buffer[2], p_event->data.done.p_buffer[3], p_event->data.done.p_buffer[4], p_event->data.done.p_buffer[5], p_event->data.done.p_buffer[6], p_event->data.done.p_buffer[7]);

		adc_psu_stat = (int16_t)p_event->data.done.p_buffer[2];
		adc_psu_vpd = (int16_t)p_event->data.done.p_buffer[0];
		adc_vbat = (int16_t)p_event->data.done.p_buffer[1];
		adc_led_det1 = (int16_t)p_event->data.done.p_buffer[5];
		adc_led_det2 = (int16_t)p_event->data.done.p_buffer[4];
		adc_led_det3 = (int16_t)p_event->data.done.p_buffer[6];
		adc_led_det4 = (int16_t)p_event->data.done.p_buffer[7];
		adc_led_det5 = (int16_t)p_event->data.done.p_buffer[3];
		break;

	case NRFX_SAADC_EVT_BUF_REQ:
		// Set up the next available buffer
		err_code = nrfx_saadc_buffer_set(&samples[next_free_buf_index()][0], SAADC_BUF_SIZE);
		APP_ERROR_CHECK(err_code);
		break;
	default:
		NRF_LOG_INFO("SAADC evt %d", p_event->type);
		break;
	}
}

static void timer_init(void)
{
	nrfx_err_t err_code;

	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG;
	timer_config.frequency = NRF_TIMER_FREQ_31250Hz;
	err_code = nrfx_timer_init(&m_sample_timer, &timer_config, timer_handler);
	APP_ERROR_CHECK(err_code);
	nrfx_timer_extended_compare(&m_sample_timer,
								NRF_TIMER_CC_CHANNEL0,
								nrfx_timer_ms_to_ticks(&m_sample_timer, saadc_sampling_rate),
								NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
								false);

	nrfx_timer_resume(&m_sample_timer);
}

static void ppi_init(void)
{
	// Trigger task sample from timer
	nrfx_err_t err_code = nrfx_ppi_channel_alloc(&m_timer_saadc_ppi_channel);
	APP_ERROR_CHECK(err_code);

	err_code = nrfx_ppi_channel_assign(m_timer_saadc_ppi_channel,
									   nrfx_timer_event_address_get(&m_sample_timer, NRF_TIMER_EVENT_COMPARE0),
									   nrf_saadc_task_address_get((NRF_SAADC_Type *)0x40007000, NRF_SAADC_TASK_SAMPLE));

	APP_ERROR_CHECK(err_code);

	err_code = nrfx_ppi_channel_enable(m_timer_saadc_ppi_channel);
	APP_ERROR_CHECK(err_code);
}

static void adc_configure(void)
{
	int err_code;

	nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
	saadc_adv_config.internal_timer_cc = 0;
	saadc_adv_config.start_on_end = true;

	err_code = nrfx_saadc_init(6); // NRFX_SAADC_CONFIG_IRQ_PRIORITY
	APP_ERROR_CHECK(err_code);

	static nrfx_saadc_channel_t channel_configs[ADC_CHANNELS_IN_USE];

	uint8_t channel_mask = 0;
	for (int i = 0; i < ADC_CHANNELS_IN_USE; i++)
	{
		nrf_saadc_input_t pin = ANALOG_INPUT_MAP[i];
		// Apply default config to each channel
		nrfx_saadc_channel_t config = NRFX_SAADC_DEFAULT_CHANNEL_SE(pin, i);

		// Replace some parameters in default config
		config.channel_config.reference = NRF_SAADC_REFERENCE_VDD4;
		config.channel_config.gain = NRF_SAADC_GAIN1_4;

		// Copy to list of channel configs
		memcpy(&channel_configs[i], &config, sizeof(config));

		// Update channel mask
		channel_mask |= 1 << i;
	}

	err_code = nrfx_saadc_channels_config(channel_configs, ADC_CHANNELS_IN_USE);
	APP_ERROR_CHECK(err_code);

	err_code = nrfx_saadc_advanced_mode_set(channel_mask,
											NRF_SAADC_RESOLUTION_14BIT,
											&saadc_adv_config,
											adcevent_handler);
	APP_ERROR_CHECK(err_code);

	// Configure two buffers to ensure double buffering of samples, to avoid data loss when the sampling frequency is high
	err_code = nrfx_saadc_buffer_set(&samples[next_free_buf_index()][0], SAADC_BUF_SIZE);
	APP_ERROR_CHECK(err_code);

	err_code = nrfx_saadc_buffer_set(&samples[next_free_buf_index()][0], SAADC_BUF_SIZE);
	APP_ERROR_CHECK(err_code);

	err_code = nrfx_saadc_mode_trigger();
	APP_ERROR_CHECK(err_code);
}

*/

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	dk_leds_init();
	dk_buttons_init(NULL);

	err = bt_mesh_init(bt_mesh_dk_prov_init(), model_handler_init());
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	printk("Mesh initialized\n");

	model_handler_start();
}


static int pwm_init(void)
{
	static nrfx_pwm_config_t const config0 =
		{
			.output_pins =
				{
					45,	// channel 0
					46,	// channel 1
					42, // channel 2
					44  // channel 3
				},
			.irq_priority = 5,
			.base_clock = NRF_PWM_CLK_1MHz,
			.count_mode = NRF_PWM_MODE_UP,
			.top_value = PWM_COUNTERTOP,
			.load_mode = NRF_PWM_LOAD_INDIVIDUAL,
			.step_mode = NRF_PWM_STEP_AUTO};
	return (nrfx_pwm_init(&my_pwm, &config0, NULL, NULL) == NRFX_SUCCESS) ? 0 : -1;

	// If PWM callbacks are to be used, remember to configure the interrupts correctly
	// IRQ_DIRECT_CONNECT(PWM1_IRQn, 0, nrfx_pwm_1_irq_handler, 0);
	// irq_enable(PWM1_IRQn);
}

void lc_pwm_led_set(uint16_t desired_lvl)
{
	uint32_t scaled_lvl = (PWM_PERIOD * desired_lvl) / BT_MESH_LIGHTNESS_MAX; // add gamma correction

	// pwm_set_dt(&pwm0, PWM_USEC(PWM_PERIOD), PWM_USEC(scaled_lvl));

	static bool pwm_running = false;

	// This array cannot be allocated on stack (hence "static") and it must be in RAM
	static nrf_pwm_values_individual_t seq_values;

	// Update the respective channels
	seq_values.channel_0 = (scaled_lvl <= PWM_COUNTERTOP) ? scaled_lvl : PWM_COUNTERTOP;

	nrf_pwm_sequence_t const seq =
		{
			.values.p_individual = &seq_values,
			.length = NRF_PWM_VALUES_LENGTH(seq_values),
			.repeats = 0,
			.end_delay = 0};

	if (!pwm_running)
	{
		pwm_running = true;
		(void)nrfx_pwm_simple_playback(&my_pwm, &seq, 1000, NRFX_PWM_FLAG_LOOP);
	}
}


void main(void)
{
	int err;

	printk("Initializing...\n");


	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

	pwm_init();

	/*
		adc_configure();
		ppi_init();
		timer_init();
	*/

	k_sem_give(&ble_init_ok);

	if (!device_is_ready(dev_i2cpsu.bus))
	{
		printk("I2C2 bus %s is not ready!\n\r", dev_i2cpsu.bus->name);
	}
	else
	{
		setPSUreg(0x00, MPP2722_REG_00);
		setPSUreg(0x01, MPP2722_REG_01);
		setPSUreg(0x02, MPP2722_REG_02);
		setPSUreg(0x03, MPP2722_REG_03);
		setPSUreg(0x04, MPP2722_REG_04);
		setPSUreg(0x05, MPP2722_REG_05);
		setPSUreg(0x06, MPP2722_REG_06);
		setPSUreg(0x07, MPP2722_REG_07);
		setPSUreg(0x08, MPP2722_REG_08);
		setPSUreg(0x09, MPP2722_REG_09);
		setPSUreg(0x0A, MPP2722_REG_0A);
		setPSUreg(0x0B, MPP2722_REG_0B);
		setPSUreg(0x0C, MPP2722_REG_0C);
		setPSUreg(0x0D, MPP2722_REG_0D);
		setPSUreg(0x0E, MPP2722_REG_0E);
		setPSUreg(0x0F, MPP2722_REG_0F);

	}

	for (;;)
	{
		k_sleep(K_MSEC(1000));
	}
}


void ble_write_thread(void)
{
	k_sem_take(&ble_init_ok, K_FOREVER);

	// 2ND THREAD STARTED AFTER BT STACK INITIALIZED - PSU MANAGEMENT

	const struct device *const fuelGauge = DEVICE_DT_GET_ONE(&max17262);

	if (!device_is_ready(fuelGauge))
	{
		printk("fuelGauge: device not ready.\n");
	}

	for (;;)
	{
		
		struct sensor_value voltage, avg_current, temperature;
		float i_avg;

		sensor_sample_fetch(fuelGauge);
		sensor_channel_get(fuelGauge, SENSOR_CHAN_GAUGE_VOLTAGE, &voltage);
		sensor_channel_get(fuelGauge, SENSOR_CHAN_GAUGE_AVG_CURRENT,
						   &avg_current);
		sensor_channel_get(fuelGauge, SENSOR_CHAN_GAUGE_TEMP, &temperature);

		i_avg = avg_current.val1 + (avg_current.val2 / 1000000.0);

		printk("V: %d.%06d V; I: %f mA; T: %d.%06d °C\n",
			   voltage.val1, voltage.val2, (double)i_avg,
			   temperature.val1, temperature.val2);



		k_sleep(K_MSEC(1000));
	}
	
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
				NULL, PRIORITY, 0, 0);
