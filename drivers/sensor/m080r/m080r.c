/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT chinese_m080r
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(m080r, CONFIG_SENSOR_LOG_LEVEL);

#define MSG_SIZE 22
/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static char heartbeat_command[] = {0xF1, 0x1F, 0xE2, 0x2E, 0xB6, 0x6B, 0xA8, 0x8A, 0x00, 0x07, 0x86, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0xFA};
/*
 * send a null-terminated string character by character to the UART interface
 */

static char rx_buf[MSG_SIZE];
static char tx_buf[MSG_SIZE];
static int rx_buf_pos;

typedef struct  __attribute__((__packed__)){
	uint8_t header[8];
	uint16_t length;
	uint8_t checksum;
	// transmission or response
}frame_header_t;

typedef struct {
	uint8_t password[4];
	uint8_t command[2];
	uint8_t* data;
	uint8_t checksum;
}transmission_frame_t;

typedef struct{
	uint8_t password[4];
	uint8_t command[2];
	uint8_t error_code[4];
	uint8_t* data;
	uint8_t checksum;
}response_frame_t;

 struct m080r_data
 {
    frame_header_t header;
    transmission_frame_t content;
    response_frame_t response;
 };


struct m080r_config {
	struct gpio_dt_spec input;
    struct device * uart;
};

void print_response(char *buf, int n)
{
	for (int i = 0; i < n; i++)
	{
		printk("%x ", buf[i]);
	}
	printk("\n");
}

void send_uart(char *buf, int n, const struct device *const uart_device)
{
	for (int i = 0; i < n; i++) {
		uart_poll_out(uart_device, buf[i]);
	}
}

void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(dev))
	{
		return;
	}

	while (uart_irq_rx_ready(dev))
	{

		uart_fifo_read(dev, &c, 1);
		// LOG_INF("Received: %x", c);
		rx_buf[rx_buf_pos++] = c;

		if (rx_buf_pos == MSG_SIZE)
		{
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
			rx_buf_pos = 0;
		}
	}
}

static int m080r_sample_fetch(const struct device *dev,
				      enum sensor_channel chan)
{
	const struct m080r_config *config = dev->config;
	struct m080r_data *data = dev->data;
    struct device * uart_device = config->uart;

    send_uart(heartbeat_command, sizeof(heartbeat_command), uart_device);
    k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER);
    print_response(tx_buf, MSG_SIZE);
	return 0;
}

static int m080r_channel_get(const struct device *dev,
				     enum sensor_channel chan,
				     struct sensor_value *val)
{
	struct m080r_data *data = dev->data;

	if (chan != SENSOR_CHAN_PROX) {
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api m080r_api = {
	.sample_fetch = &m080r_sample_fetch,
	.channel_get = &m080r_channel_get,
};

static int m080r_init(const struct device *dev)
{
	const struct m080r_config *config = dev->config;

	int ret;

    if(!device_is_ready(config->uart)){
        LOG_ERR("UART device not ready!");
        return -ENODEV;
    }

	uart_irq_callback_user_data_set(config->uart, serial_cb, NULL);
	uart_irq_rx_enable(config->uart);

	return 0;
}

#define M080R_INIT(i)						       \
	static struct m080r_data m080r_data_##i;	       \
									       \
	static const struct m080r_config m080r_config_##i = {  \
		.input = GPIO_DT_SPEC_INST_GET(i, input_gpios),		       \
        .uart = DT_INST_PHANDLE(i,uart_parent_node), \
	};								       \
									       \
	DEVICE_DT_INST_DEFINE(i, m080r_init, NULL,		       \
			      &m080r_data_##i,			       \
			      &m080r_config_##i, POST_KERNEL,	       \
			      CONFIG_SENSOR_INIT_PRIORITY, &m080r_api);

DT_INST_FOREACH_STATUS_OKAY(M080R_INIT)