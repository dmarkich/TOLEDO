/*

 TOLEDO BT MESH PROTOTYPE - NRF52840


 https://docs.zephyrproject.org/latest/samples/fuel_gauge/max17048/README.html
 

 */
#include <main.h>


static K_SEM_DEFINE(ble_init_ok, 0, 1);

// static const struct i2c_dt_spec dev_i2cpsu = I2C_DT_SPEC_GET(I2C0_NODE1);

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
	// int err = i2c_write_dt(&dev_i2cpsu, &psuwr, 2);
	// if (err != 0)
	// {
	// 	printk("ERROR WRITING TO PSU REGISTER %x, VALUE  %x\n\r", reg, val);
	// 	return false;
	// }

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

int main(void)
{
	int err;

	printk("Initializing...\n");


	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

	// pwm_init();

	/*
		adc_configure();
		ppi_init();
		timer_init();
	*/

	k_sem_give(&ble_init_ok);

	// if (!device_is_ready(dev_i2cpsu.bus))
	// {
	// 	printk("I2C2 bus %s is not ready!\n\r", dev_i2cpsu.bus->name);
	// }
	// else
	// {
	// 	setPSUreg(0x00, MPP2722_REG_00);
	// 	setPSUreg(0x01, MPP2722_REG_01);
	// 	setPSUreg(0x02, MPP2722_REG_02);
	// 	setPSUreg(0x03, MPP2722_REG_03);
	// 	setPSUreg(0x04, MPP2722_REG_04);
	// 	setPSUreg(0x05, MPP2722_REG_05);
	// 	setPSUreg(0x06, MPP2722_REG_06);
	// 	setPSUreg(0x07, MPP2722_REG_07);
	// 	setPSUreg(0x08, MPP2722_REG_08);
	// 	setPSUreg(0x09, MPP2722_REG_09);
	// 	setPSUreg(0x0A, MPP2722_REG_0A);
	// 	setPSUreg(0x0B, MPP2722_REG_0B);
	// 	setPSUreg(0x0C, MPP2722_REG_0C);
	// 	setPSUreg(0x0D, MPP2722_REG_0D);
	// 	setPSUreg(0x0E, MPP2722_REG_0E);
	// 	setPSUreg(0x0F, MPP2722_REG_0F);

	// }

	for (;;)
	{
		k_sleep(K_MSEC(1000));
	}
}


void ble_write_thread(void)
{
	k_sem_take(&ble_init_ok, K_FOREVER);

	// 2ND THREAD STARTED AFTER BT STACK INITIALIZED - PSU MANAGEMENT

	for (;;)
	{	
		k_sleep(K_MSEC(1000));
	}
	
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
				NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(fuel_gauge_thread_id, STACKSIZE, read_gauge, NULL, NULL, NULL, PRIORITY, 0, 0);
