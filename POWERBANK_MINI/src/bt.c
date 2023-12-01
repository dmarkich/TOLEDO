#include <bt.h>

void bt_ready(int err)
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

void ble_write_thread(void)
{
	k_sem_take(&ble_init_ok, K_FOREVER);

	// 2ND THREAD STARTED AFTER BT STACK INITIALIZED - PSU MANAGEMENT

	for (;;)
	{	
		k_sleep(K_MSEC(1000));
	}
	
}