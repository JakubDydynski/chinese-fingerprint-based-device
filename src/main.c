/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <string.h>
#include <zephyr/logging/log.h>
#include "fingerprint_sensor.h"

LOG_MODULE_REGISTER(logging_blog, LOG_LEVEL_DBG);

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

// #define UART1_NODE DT_NODELABEL(arduino_serial)
#define UART1_NODE DT_ALIAS(uartdebug)

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define MSG_SIZE 22

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   5000

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static const struct device *const uart_sensor = DEVICE_DT_GET(UART1_NODE);


static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
															   {0});
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios,
															   {0});

static struct gpio_callback button0_cb_data, button1_cb_data;
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;
/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */

void button0_pressed(const struct device *dev, struct gpio_callback *cb,
					 uint32_t pins)
{
	sensor_t sensor;
	sensor.uart = uart_sensor;
	sensor_init(&sensor);
	// send_uart(command, sizeof(command), uart_sensor);
}

void button1_pressed(const struct device *dev, struct gpio_callback *cb,
					 uint32_t pins)
{

}

void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_sensor)) {
		return;
	}

	while (uart_irq_rx_ready(uart_sensor)) {

		uart_fifo_read(uart_sensor, &c, 1);
		// LOG_INF("Received: %x", c);
		rx_buf[rx_buf_pos++] = c;

		if(rx_buf_pos == MSG_SIZE){
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
			rx_buf_pos = 0;
		}
	}
}

// /*
//  * send a null-terminated string character by character to the UART interface
//  */
// void send_uart(char *buf, int n, const struct device *const uart_device)
// {
// 	for (int i = 0; i < n; i++) {
// 		uart_poll_out(uart_device, buf[i]);
// 	}
// }

void print_basic_sensor_info(void){
	while(1){
		LOG_INF("Hello World! from print_basic_sensor_info thread %s\n", CONFIG_BOARD);
		k_sleep(K_FOREVER);
	}
}
K_THREAD_DEFINE(print_basic_sensor_info_id, STACKSIZE, print_basic_sensor_info, NULL, NULL, NULL,
		PRIORITY, 0, 0);


void main(void)
{
	int ret;
	/* ------------------------------------------------------------------------------------------------------------------ */
	/*                                                         LED                                                        */
	/* ------------------------------------------------------------------------------------------------------------------ */
	if (!device_is_ready(led.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}
	/* ------------------------------------------------------------------------------------------------------------------ */
	/*                                                         BUTTONS                                                        */
	/* ------------------------------------------------------------------------------------------------------------------ */
	btn_init();
	btn_callback_config();

	/* ------------------------------------------------------------------------------------------------------------------ */
	/*                                                        UART                                                        */
	/* ------------------------------------------------------------------------------------------------------------------ */
	char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return;
	}

	if (!device_is_ready(uart_sensor)) {
		printk("UART device not found!\r\n");
		return;
	} else {
		printk("UART_dbg device found!\r\n");
	}

	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_sensor, serial_cb, NULL);
	uart_irq_rx_enable(uart_sensor);
	
	/* indefinitely wait for input from the user */
	// while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
	// 	LOG_INF("Received array: %s", tx_buf);
	// }

	/* ------------------------------------------------------------------------------------------------------------------ */
	/*                                                        LOOP                                                        */
	/* ------------------------------------------------------------------------------------------------------------------ */

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return;
		}
		k_msleep(SLEEP_TIME_MS);
		k_wakeup(print_basic_sensor_info_id);
		if(k_msgq_get(&uart_msgq, &tx_buf, K_MSEC(100)) == 0){
			print_response(tx_buf, MSG_SIZE);	
		}
	}
}
void print_response(char *buf, int n)
{
	for (int i = 0; i < n; i++) {
		printk("%x ", buf[i]);
	}
	printk("\n");
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


