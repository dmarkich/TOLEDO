#include <battery_sensor.h>

int read_gauge(struct battery_info *bat_info)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(max17048);
	int ret = 0;

	if (dev == NULL)
	{
		printk("\nError: no device found.\n");
		return 0;
	}

	if (!device_is_ready(dev))
	{
		printk("\nError: Device \"%s\" is not ready; "
			   "check the driver initialization logs for errors.\n",
			   dev->name);
		return 0;
	}

	printk("Found device \"%s\", getting fuel gauge data\n", dev->name);

	if (dev == NULL)
	{
		return 0;
	}

	struct fuel_gauge_get_property vals[] = {
		{property_type : FUEL_GAUGE_VOLTAGE},
		{property_type : FUEL_GAUGE_RUNTIME_TO_EMPTY},
		{property_type : FUEL_GAUGE_RUNTIME_TO_FULL},
		{property_type : FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE},
	};

	ret = fuel_gauge_get_prop(dev, vals, ARRAY_SIZE(vals));
	if (ret < 0)
	{
		printk("Error: cannot get properties\n");
		return 0;
	}
	else
	{

		bat_info->runtime_to_empty = vals[0].value.runtime_to_empty;
		bat_info->runtime_to_full = vals[1].value.runtime_to_full;
		bat_info->relative_state_of_charge = vals[2].value.relative_state_of_charge;
		bat_info->voltage = vals[3].value.voltage;
		device_battery = *bat_info;

		printk("Time to empty %d\n", vals[0].value.runtime_to_empty);

		printk("Time to full %d\n", vals[1].value.runtime_to_full);

		printk("Charge %d%%\n", vals[2].value.relative_state_of_charge);

		printk("Voltage %d\n", vals[3].value.voltage);

		return 1;
	}
}