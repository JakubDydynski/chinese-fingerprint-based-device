#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include "m080r.h"
LOG_MODULE_REGISTER(logging_blog, LOG_LEVEL_DBG);

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)

static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
															   {0});
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios,
															   {0});
static struct gpio_callback button0_cb_data, button1_cb_data;
int ret;
const struct device *sensor;
int flag_get = 0;

void button1_pressed(const struct device *dev, struct gpio_callback *cb,
					 uint32_t pins)
{
	flag_get = 2;
}
void button0_pressed(const struct device *dev, struct gpio_callback *cb,
					 uint32_t pins)
{
	flag_get = 1;
}

void print_basic_sensor_info(void)
{
	while (1)
	{
		LOG_INF("Hello World! from print_basic_sensor_info thread %s\n", CONFIG_BOARD);
		k_sleep(K_FOREVER);
	}
}
K_THREAD_DEFINE(print_basic_sensor_info_id, STACKSIZE, print_basic_sensor_info, NULL, NULL, NULL,
				PRIORITY, 0, 0);

void main(void)
{

	printk("M080R Example Application \n");

	btn_init();
	btn_callback_config();

	sensor = DEVICE_DT_GET(DT_NODELABEL(m080r0));

	if (!device_is_ready(sensor)) {
		printk("Sensor not ready");
	} else {
		printk("SENDOR READY!\n");
	}

	enum pm_device_state *state;
	enum pm_device_action state_suspend = PM_DEVICE_ACTION_SUSPEND;

	

	while (1) {

		// ret = sensor_sample_fetch(sensor);
		// if (ret < 0) {
		// 	printk("Could not fetch sample (%d)", ret);
		// }

		struct sensor_value val;
		if (flag_get == 1) {
			flag_get = 0;
			// pm_device_action_run(sensor, state_suspend);
				k_msleep(1000);
				sensor_attr_set(sensor, SENSOR_CHAN_SLEEP, SENSOR_ATTR_NORMAL_SLEEP, &val);
		}
		else if (flag_get == 2)
		{
			flag_get = 0;
			printk("match: \n");
			ret = sensor_attr_set(sensor, SENSOR_CHAN_MATCH_FINGERPRINT, SENSOR_ATTR_MAX, &val);
			if (ret < 0) {
				LOG_ERR("Could not get sample (%d)", ret);
				return 0;
			}
			if (val.val1 == 0)
			{
				printk("no match\n");
				printk("register: \n");
				ret = sensor_attr_set(sensor, SENSOR_CHAN_REGISTER_FINGERPRINT, SENSOR_ATTR_MAX, &val);
				if (ret < 0) {
					LOG_ERR("Could not get sample (%d)", ret);
					return 0;
				}

				printk("save: \n");
				ret = sensor_attr_set(sensor, SENSOR_CHAN_SAVE_FINGERPRINT, SENSOR_ATTR_MAX, &val);
				if (ret < 0) {
					LOG_ERR("Could not get sample (%d)", ret);
					return 0;
				}
				printk("id of saved sensor: %d\n", val.val1);
			}
			else
			{
				printk("id of matched sensor: %d\n", val.val1);
			}
			
		}
		
		k_sleep(K_MSEC(1000));
		
	}
}


void btn_init()
{
	int ret;
	if (!device_is_ready(button0.port))
	{
		printk("Error: button device %s is not ready\n",
			   button0.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button0, GPIO_INPUT);
	if (ret != 0)
	{
		printk("Error %d: failed to configure %s pin %d\n",
			   ret, button0.port->name, button0.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button0,
										  GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0)
	{
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			   ret, button0.port->name, button0.pin);
		return;
	}

	if (!device_is_ready(button1.port))
	{
		printk("Error: button device %s is not ready\n",
			   button1.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (ret != 0)
	{
		printk("Error %d: failed to configure %s pin %d\n",
			   ret, button1.port->name, button1.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button1,
										  GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0)
	{
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			   ret, button1.port->name, button1.pin);
		return;
	}
}

void btn_callback_config()
{
	gpio_init_callback(&button0_cb_data, button0_pressed, BIT(button0.pin));
	gpio_add_callback(button0.port, &button0_cb_data);
	printk("Set up button at %s pin %d\n", button0.port->name, button0.pin);

	gpio_init_callback(&button1_cb_data, button1_pressed, BIT(button1.pin));
	gpio_add_callback(button1.port, &button1_cb_data);
	printk("Set up button at %s pin %d\n", button1.port->name, button1.pin);

}
