/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include "m080r.h"
LOG_MODULE_REGISTER(logging_blog, LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 5000

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define SW0_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
															   {0});

static struct gpio_callback button0_cb_data;
int ret;
const struct device *sensor;
unsigned int key;
int flag_get = 0;


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
	enum pm_device_action state_resume = PM_DEVICE_ACTION_RESUME;
	//normal sleep mode
	struct sensor_trigger trig = { .type = SENSOR_TRIG_DATA_READY};
	k_msleep(1000);
	sensor_trigger_set(sensor, &trig, NULL);
	k_msleep(100);
	pm_device_action_run(sensor, state_suspend);

	while (1) {

		struct sensor_value val;
		if (flag_get == 1) {
			pm_device_action_run(sensor, state_resume);
			k_msleep(100);
			flag_get = 0;
			printk("match: \n");
			ret = sensor_attr_set(sensor, SENSOR_CHAN_MATCH_FINGERPRINT, SENSOR_ATTR_MAX, &val);
			printk("id of matched sensor: %d\n", val.val1);
			ret = sensor_attr_set(sensor, SENSOR_CHAN_SLEEP, SENSOR_ATTR_NORMAL_SLEEP, &val);
			k_msleep(100);
			pm_device_action_run(sensor, state_suspend);
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
}

void btn_callback_config()
{
	gpio_init_callback(&button0_cb_data, button0_pressed, BIT(button0.pin));
	gpio_add_callback(button0.port, &button0_cb_data);
	printk("Set up button at %s pin %d\n", button0.port->name, button0.pin);
}
