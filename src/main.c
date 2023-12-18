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

LOG_MODULE_REGISTER(logging_blog, LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 5000

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

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
	int ret;
	const struct device *sensor;

	printk("M080R Example Application \n");

	sensor = DEVICE_DT_GET(DT_NODELABEL(m080r0));

	if (!device_is_ready(sensor)) {
		printk("Sensor not ready");
	} else {
		printk("SENDOR READY!");
	}


	while (1) {
		// struct sensor_value val;

		ret = sensor_sample_fetch(sensor);
		if (ret < 0) {
			printk("Could not fetch sample (%d)", ret);
		}

		// ret = sensor_channel_get(sensor, SENSOR_CHAN_PROX, &val);
		// if (ret < 0) {
		// 	LOG_ERR("Could not get sample (%d)", ret);
		// 	return 0;
		// }

		// printk("Sensor value: %d\n", val.val1);

		k_sleep(K_MSEC(1000));
	}
}
