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
typedef enum status{
	OK,
	ERROR
}status_t;

status_t sensor_init(void){
	return OK;
}

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
// #define UART1_NODE DT_NODELABEL(arduino_serial)
#define UART1_NODE DT_NODELABEL(uart1)

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define MSG_SIZE 22

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   2000

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

static const struct device *const uart_sensor = DEVICE_DT_GET(UART1_NODE);
/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_sensor)) {
		return;
	}

	while (uart_irq_rx_ready(uart_sensor)) {

		uart_fifo_read(uart_sensor, &c, 1);

		rx_buf[rx_buf_pos++] = c;

		if(rx_buf_pos == MSG_SIZE){
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
			rx_buf_pos = 0;
		}
	}
}

/*
 * send a null-terminated string character by character to the UART interface
 */
void send_uart(char *buf, int n)
{
	for (int i = 0; i < n; i++) {
		uart_poll_out(uart_sensor, buf[i]);
	}
}

void print_basic_sensor_info(void){
	while(1){
		printk("Hello World! %s\n", CONFIG_BOARD);
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
	/*                                                        UART                                                        */
	/* ------------------------------------------------------------------------------------------------------------------ */
	char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return;
	}

	if (!device_is_ready(uart_sensor)) {
		printk("UART device not found!");
		return;
	} else {
		printk("UART_dbg device found!");
	}

	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_sensor, serial_cb, NULL);
	uart_irq_rx_enable(uart_sensor);

	uint8_t command[] = {0xF1, 0x1F, 0xE2, 0x2E, 0xB6, 0x6B, 0xA8, 0x8A, 0x00, 0x07, 0x86, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0xFA};
	send_uart(command, sizeof(command));

	/* indefinitely wait for input from the user */
	// while (k_msgq_get(&uart_msgq, &tx_buf, K_MSEC(200)) == 0) {
	// 	// print_uart("Echo: ");
	// 	// print_uart(tx_buf);
	// 	// print_uart("\r\n");
	// }
	k_msgq_get(&uart_msgq, &tx_buf, K_MSEC(200));
	printk("%s\n", tx_buf);
	k_msgq_get(&uart_msgq, &tx_buf, K_MSEC(200));
	printk("%s\n", tx_buf);
	k_msgq_get(&uart_msgq, &tx_buf, K_MSEC(200));
	printk("%s\n", tx_buf);
	k_msgq_get(&uart_msgq, &tx_buf, K_MSEC(200));
	printk("%s\n", tx_buf);

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
	}
}


