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

#define MSG_SIZE_0DATA 22
#define MSG_SIZE_3DATA (22 + 3)
#define MSG_SIZE_6DATA (22 + 6)
#define MSG_SIZE_2DATA (22 + 2)
/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE_6DATA, 10, 4);

static char heartbeat_command[] = {0xF1, 0x1F, 0xE2, 0x2E, 0xB6, 0x6B, 0xA8, 0x8A, 0x00, 0x07, 0x86, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0xFA};
static const char header_init[] = {0xF1, 0x1F, 0xE2, 0x2E, 0xB6, 0x6B, 0xA8, 0x8A};
/*
 * send a null-terminated string character by character to the UART interface
 */

static char rx_buf[MSG_SIZE_6DATA];
static char tx_buf[MSG_SIZE_6DATA];
static int rx_buf_pos;
static int current_rx_data_size = MSG_SIZE_0DATA; // global variable needed to handle different size messages

typedef struct  __attribute__((__packed__)){
	uint8_t header[8];
	uint8_t length[2];
	uint8_t checksum;
	// transmission or response
}frame_header_t;

typedef struct {
	uint8_t password[4];
	uint8_t command[2];
	uint8_t* data;
	uint8_t data_len;
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
	int last_id;
 };


struct m080r_config {
	struct gpio_dt_spec int_gpios;
    struct device * uart;
};
/* ------------------------------------------------------------------------------------------------------------------ */
/*                                                        UART                                                        */
/* ------------------------------------------------------------------------------------------------------------------ */
static void print_response(char *buf, int n)
{
	for (int i = 0; i < n; i++)
	{
		printk("%02x ", buf[i]);
	}
	printk("\n");
}

static void send_uart(char *buf, int n, const struct device *const uart_device)
{	
	printk("Sending: ");
	for (int i = 0; i < n; i++) {
		uart_poll_out(uart_device, buf[i]);
		printk("%02x ", buf[i]);
	}
	printk("\n");
}

static void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(dev))
	{
		return;
	}

	while (uart_irq_rx_ready(dev))
	{

		uart_fifo_read(dev, &c, 1);
		rx_buf[rx_buf_pos++] = c;

		if (rx_buf_pos == current_rx_data_size)
		{
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
			rx_buf_pos = 0;
		}
	}
}
/* ------------------------------------------------------------------------------------------------------------------ */
/*                                                   Helper methods                                                   */
/* ------------------------------------------------------------------------------------------------------------------ */

static void prepare_header(struct m080r_data *data, int length)
{
// header checksum
// 82- 11
// 85 - 8 a więc 
// 8D - 0
	memcpy(data->header.header, header_init, sizeof(header_init));
	data->header.length[0] = 0x00;
	data->header.length[1] = length;
	data->header.checksum = 0x8D - length; // documentation - header checksum is dependent on length
}
#define SENT_HEADER() send_uart((char *)&data->header, sizeof(data->header), uart_device) 
#define SENT_APP_DATA() \
do {\
	send_uart((char *)data->content.password, sizeof(data->content.password), uart_device); \
	send_uart((char *)data->content.command, sizeof(data->content.command), uart_device);\
	if (data->content.data_len > 0)\
	send_uart((char *)data->content.data, data->content.data_len, uart_device);\
	send_uart((char *)&data->content.checksum, sizeof(data->content.checksum), uart_device);\
	data->content.checksum = 0;\
} while(0)
#define GET_RESPONSE_AND_PRINT() \
do {\
	k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER);\
	print_response(tx_buf, current_rx_data_size);\
} while(0)

static void set_command(struct m080r_data *data ,int command, int response_length)
{
	data->content.command[0] = 0x01;	
	data->content.command[1] = command;
	current_rx_data_size = response_length;
}

static void set_data(struct m080r_data *data, uint8_t *data_buff, int data_length)
{
	data->content.data = data_buff;
	data->content.data_len = data_length;
}

static void calc_checksum(struct m080r_data *data)
{
	for (int i = 0; i < sizeof(data->content.password); i++)
	{
		data->content.checksum += data->content.password[i];
	}
	for (int i = 0; i < data->content.data_len; i++)
	{
		data->content.checksum += data->content.data[i];
	}
	data->content.checksum += data->content.command[0];
	data->content.checksum += data->content.command[1];
	data->content.checksum = (~data->content.checksum) + 1;
}

/* ------------------------------------------------------------------------------------------------------------------ */
/*                                               Fingerprint sensor api                                               */
/* ------------------------------------------------------------------------------------------------------------------ */

static int m080r_sample_fetch(const struct device *dev,
				      enum sensor_channel chan)
{
	const struct m080r_config *config = dev->config;
	struct m080r_data *data = dev->data;
    struct device * uart_device = config->uart;
	//for testing purposes only
	current_rx_data_size = MSG_SIZE_0DATA;
    send_uart(heartbeat_command, sizeof(heartbeat_command), uart_device);

    k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER);
    print_response(tx_buf, MSG_SIZE_0DATA);
	return 0;
}

static int m080r_register_fingerprint(const struct device *dev,
				     enum sensor_channel chan,
				     struct sensor_value *val)
{
	const struct m080r_config *config = dev->config;
	struct m080r_data *data = dev->data;
    struct device * uart_device = config->uart;

	if (chan != SENSOR_CHAN_PROX) {
		return -ENOTSUP;
	}
	/* ---------------------------------------------- register fingerprint ---------------------------------------------- */
	prepare_header(data, 0x08);

	for (uint8_t i = 1; i < 7; i++)
	{
		set_command(data, 0x11, MSG_SIZE_0DATA); // TODO: zamiast 0x11 wstawic jakies enumy z komendami, wtedy też msg_size mozna by ogarnąć
		uint8_t data_buff[] = {i};
		set_data(data, data_buff, sizeof(data_buff));
		calc_checksum(data);

		SENT_HEADER();
		SENT_APP_DATA();
		GET_RESPONSE_AND_PRINT();
		k_sleep(K_MSEC(1000));
	}
	/* --------------------------------------------- query register results --------------------------------------------- */
	prepare_header(data, 0x07);
	set_command(data, 0x12, MSG_SIZE_3DATA);
	set_data(data, NULL, 0);
	calc_checksum(data);

	SENT_HEADER();
	SENT_APP_DATA();
	GET_RESPONSE_AND_PRINT();
	k_sleep(K_MSEC(1000));

	data->last_id = tx_buf[22];  //id of the last registered fingerprint
	val->val1 = tx_buf[20]; // error code

	return 0;
}
static int m080r_fingerprint_save(const struct device *dev,
					enum sensor_channel chan,
					enum sensor_attribute attr,
					struct sensor_value *val)
{
	const struct m080r_config *config = dev->config;
	struct m080r_data *data = dev->data;
    struct device * uart_device = config->uart;

	/* ------------------------------------------------ save fingerprint ------------------------------------------------ */
	uint8_t data_buff[] = {0x00, data->last_id}; // id of the last registered fingerprint
	prepare_header(data, 0x09);
	set_command(data, 0x13, MSG_SIZE_0DATA);
	set_data(data, data_buff, sizeof(data_buff));
	calc_checksum(data);

	SENT_HEADER();
	SENT_APP_DATA();
	GET_RESPONSE_AND_PRINT();

	k_sleep(K_MSEC(1000));
	/* ------------------------------------------------ query save result ----------------------------------------------- */

	prepare_header(data, 0x07);
	set_command(data, 0x14, MSG_SIZE_2DATA);
	set_data(data, NULL, 0);
	calc_checksum(data);

	SENT_HEADER();
	SENT_APP_DATA();
	GET_RESPONSE_AND_PRINT();

	k_sleep(K_MSEC(1000));

	val->val1 = tx_buf[22]; // id of sensor

	return 0;
}
static int m080r_match_fingerprint(const struct device *dev,
					enum sensor_channel chan,
					enum sensor_attribute attr,
					struct sensor_value *val)
{

	const struct m080r_config *config = dev->config;
	struct m080r_data *data = dev->data;
    struct device * uart_device = config->uart;

	/* ------------------------------------------------ match fingerprint ----------------------------------------------- */

	prepare_header(data, 0x07);
	set_command(data, 0x21, MSG_SIZE_0DATA);
	set_data(data, NULL, 0);
	calc_checksum(data);

	SENT_HEADER();
	SENT_APP_DATA();
	GET_RESPONSE_AND_PRINT();

	k_sleep(K_MSEC(1000));
	
	/* --------------------------------------------- query matching results --------------------------------------------- */

	int command = 0x22;
	int length = 0x07;
	prepare_header(data, 0x07);
	set_command(data, 0x22, MSG_SIZE_6DATA);
	set_data(data, NULL, 0);
	calc_checksum(data);
	SENT_HEADER();
	SENT_APP_DATA();
	GET_RESPONSE_AND_PRINT();

	k_sleep(K_MSEC(1000));

	val->val1 = tx_buf[24]; // id of fingerprint

	return 0;
}

static const struct sensor_driver_api m080r_api = {
	.sample_fetch = &m080r_sample_fetch,
	.channel_get = &m080r_register_fingerprint,
	.attr_get = &m080r_match_fingerprint,
	.attr_set = &m080r_fingerprint_save,
};

static int m080r_init(const struct device *dev)
{
	const struct m080r_config *config = dev->config;

    if(!device_is_ready(config->uart)){
        LOG_ERR("UART device not ready!");
        return -ENODEV;
    }

	uart_irq_callback_user_data_set(config->uart, serial_cb, NULL);
	uart_irq_rx_enable(config->uart);

	return 0;
}

#define M080R_INIT(i)						       					\
	static struct m080r_data m080r_data_##i;	       				\
									       							\
	static const struct m080r_config m080r_config_##i = {  			\
		.uart = DEVICE_DT_GET(DT_INST_PHANDLE(i,uart_node)),		\
		.int_gpios = GPIO_DT_SPEC_INST_GET(i, irq_gpios),			\
	};								       							\
									       							\
	DEVICE_DT_INST_DEFINE(i, m080r_init, NULL,		       			\
			      &m080r_data_##i,			       					\
			      &m080r_config_##i, POST_KERNEL,	       			\
			      CONFIG_SENSOR_INIT_PRIORITY, &m080r_api);

DT_INST_FOREACH_STATUS_OKAY(M080R_INIT)