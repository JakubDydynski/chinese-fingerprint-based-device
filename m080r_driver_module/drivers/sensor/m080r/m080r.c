/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

/* ------------------------------------------------------------------------------------------------------------------ */
/*                                                      Includes                                                      */
/* ------------------------------------------------------------------------------------------------------------------ */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include "m080r.h"

/* ------------------------------------------------------------------------------------------------------------------ */
/*                                                       Defines                                                      */
/* ------------------------------------------------------------------------------------------------------------------ */

#define DT_DRV_COMPAT chinese_m080r
#define MSG_SIZE_0DATA 22
#define MSG_SIZE_3DATA (22 + 3)
#define MSG_SIZE_6DATA (22 + 6)
#define MSG_SIZE_2DATA (22 + 2)
#define CMD_GROUP1 0x0100U
#define CMD_GROUP2 0x0200U
#define CMD_REGISTER_FINGERPRINT 		(CMD_GROUP1 + 0x11U)
#define CMD_REGISTER_FINGERPRINT_QUERY  (CMD_GROUP1 + 0x12U)
#define CMD_SAVE_FINGERPRINT 			(CMD_GROUP1 + 0x13U)
#define CMD_SAVE_FINGERPRINT_QUERY  	(CMD_GROUP1 + 0x14U)
#define CMD_MATCH_FINGERPRINT 			(CMD_GROUP1 + 0x21U)
#define CMD_MATCH_FINGERPRINT_QUERY 	(CMD_GROUP1 + 0x22U)
#define CMD_SLEEP 						(CMD_GROUP2 + 0x0CU)

/* ------------------------------------------------------------------------------------------------------------------ */
/*                                                        Types                                                       */
/* ------------------------------------------------------------------------------------------------------------------ */
typedef enum {
	CMD_REGISTER_FINGERPRINT_ID,
	CMD_REGISTER_FINGERPRINT_QUERY_ID,
	CMD_SAVE_FINGERPRINT_ID,
	CMD_SAVE_FINGERPRINT_QUERY_ID,
	CMD_MATCH_FINGERPRINT_ID,
	CMD_MATCH_FINGERPRINT_QUERY_ID,
	CMD_SLEEP_ID,
} m080r_cmd_id_t;

typedef struct m080r_command
{
	uint16_t cmd_id;
	uint8_t msg_size;
}m080r_command_t;

typedef struct __attribute__((__packed__))
{
	uint8_t header[8];
	uint8_t length[2];
	uint8_t checksum;
} frame_header_t;

typedef struct
{
	uint8_t password[4];
	uint8_t command[2];
	uint8_t *data;
	uint8_t data_len;
	uint8_t checksum;
} transmission_frame_t;

typedef struct
{
	uint8_t password[4];
	uint8_t command[2];
	uint8_t error_code[4];
	uint8_t *data;
	uint8_t checksum;
} response_frame_t;

struct m080r_data
{
	frame_header_t header;
	transmission_frame_t content;
	response_frame_t response;
	int last_id;
};

struct m080r_config
{
	struct gpio_dt_spec int_gpios;
	struct gpio_dt_spec pwr_en_gpios;
	struct device *uart;
	struct device *m080r_dev;
};
typedef int (*sensor_action_t)(const struct device *dev, enum sensor_attribute attr, struct sensor_value *val);
/* ------------------------------------------------------------------------------------------------------------------ */
/*                                                     Global data                                                    */
/* ------------------------------------------------------------------------------------------------------------------ */
/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE_6DATA, 10, 4);
LOG_MODULE_REGISTER(m080r, CONFIG_SENSOR_LOG_LEVEL);

static const char heartbeat_command[] = {0xF1, 0x1F, 0xE2, 0x2E, 0xB6, 0x6B, 0xA8, 0x8A, 0x00, 0x07, 0x86, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0xFA};
static const char header_init[] = {0xF1, 0x1F, 0xE2, 0x2E, 0xB6, 0x6B, 0xA8, 0x8A};
static char rx_buf[MSG_SIZE_6DATA];
static char tx_buf[MSG_SIZE_6DATA];
static int rx_buf_pos;
static int current_rx_data_size = MSG_SIZE_0DATA; // global variable needed to handle different size messages
static struct gpio_callback int1_cb;
static const m080r_command_t cmd_tab[]= {
		{CMD_REGISTER_FINGERPRINT, MSG_SIZE_0DATA},
		{CMD_REGISTER_FINGERPRINT_QUERY, MSG_SIZE_3DATA},
		{CMD_SAVE_FINGERPRINT, MSG_SIZE_0DATA},
		{CMD_SAVE_FINGERPRINT_QUERY, MSG_SIZE_2DATA},
		{CMD_MATCH_FINGERPRINT, MSG_SIZE_0DATA},
		{CMD_MATCH_FINGERPRINT_QUERY, MSG_SIZE_6DATA},
		{CMD_SLEEP, MSG_SIZE_0DATA},
};

static int m080r_register_fingerprint(const struct device *dev, enum sensor_attribute attr, struct sensor_value *val);
static int m080r_save_fingerprint(const struct device *dev, enum sensor_attribute attr, struct sensor_value *val);
static int m080r_match_fingerprint(const struct device *dev, enum sensor_attribute attr, struct sensor_value *val);
static int m080r_sleep(const struct device *dev, enum sensor_attribute attr, struct sensor_value *val);
static const sensor_action_t sensor_action[] = {
 	m080r_register_fingerprint,
 	m080r_save_fingerprint,
 	m080r_match_fingerprint,
 	m080r_sleep,
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
	for (int i = 0; i < n; i++)
	{
		uart_poll_out(uart_device, buf[i]);
		printk("%02x ", buf[i]);
	}
	printk("\n");
}

static void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(dev))
		return;

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
/*                                                        GPIO                                                        */
/* ------------------------------------------------------------------------------------------------------------------ */

static int m080r_init_int_pin(const struct gpio_dt_spec *pin,
							  struct gpio_callback *pin_cb,
							  gpio_callback_handler_t handler)
{
	int ret;

	if (!pin->port)
	{
		return 0;
	}

	if (!device_is_ready(pin->port))
	{
		printk("%s not ready", pin->port->name);
		return -ENODEV;
	}

	gpio_init_callback(pin_cb, handler, BIT(pin->pin));

	ret = gpio_pin_configure_dt(pin, GPIO_INPUT);
	if (ret)
	{
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(pin, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret)
	{
		return ret;
	}

	ret = gpio_add_callback(pin->port, pin_cb);
	if (ret)
	{
		return ret;
	}
	return 0;
}

static int m080r_init_pwr_en_pin(const struct gpio_dt_spec *pin)
{
	int ret;

	if (!device_is_ready(pin->port))
	{
		printk("%s not ready", pin->port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(pin, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

static void m080r_int1_callback(const struct device *dev,
								struct gpio_callback *cb, uint32_t pins)
{
	// printk("touch out interrupt\n");
}

/* ------------------------------------------------------------------------------------------------------------------ */
/*                                                   Helper methods                                                   */
/* ------------------------------------------------------------------------------------------------------------------ */

#define SENT_HEADER() send_uart((char *)&data->header, sizeof(data->header), uart_device)
#define SENT_APP_DATA()                                                                          \
	do                                                                                           \
	{                                                                                            \
		send_uart((char *)data->content.password, sizeof(data->content.password), uart_device);  \
		send_uart((char *)data->content.command, sizeof(data->content.command), uart_device);    \
		if (data->content.data_len > 0)                                                          \
			send_uart((char *)data->content.data, data->content.data_len, uart_device);          \
		send_uart((char *)&data->content.checksum, sizeof(data->content.checksum), uart_device); \
		data->content.checksum = 0;                                                              \
	} while (0)
#define GET_RESPONSE_AND_PRINT()                       \
	do                                                 \
	{                                                  \
		k_msgq_get(&uart_msgq, &tx_buf, K_MSEC(2000)); \
		print_response(tx_buf, current_rx_data_size);  \
	} while (0)

	
static void prepare_header(struct m080r_data *data, int data_length)
{
	data->content.data_len = data_length;
	memcpy(data->header.header, header_init, sizeof(header_init));
	data_length += sizeof(data->content.password) + sizeof(data->content.command) + sizeof(data->content.checksum); // documenation
	data->header.length[0] = data_length >> 8;
	data->header.length[1] = data_length;
	data->header.checksum = 0x8D - data_length; // documentation - header checksum is dependent on length
}

static void set_command(struct m080r_data *data, m080r_cmd_id_t cdm_id)
{
	data->content.command[0] = cmd_tab[cdm_id].cmd_id >> 8;
	data->content.command[1] = cmd_tab[cdm_id].cmd_id;
	current_rx_data_size = cmd_tab[cdm_id].msg_size;
}

static inline void set_data(struct m080r_data *data, uint8_t *data_buff)
{
	data->content.data = data_buff;
}

static void set_checksum(struct m080r_data *data)
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

static int m080r_heartbeat(const struct device *dev,
							  enum sensor_channel chan)
{
	const struct m080r_config *config = dev->config;
	struct m080r_data *data = dev->data;
	struct device *uart_device = config->uart;
	// for testing purposes only
	current_rx_data_size = MSG_SIZE_0DATA;
	send_uart(heartbeat_command, sizeof(heartbeat_command), uart_device);

	k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER);
	print_response(tx_buf, MSG_SIZE_0DATA);
	return 0;
}

static int m080r_fingerprint_action(const struct device *dev,
								  enum sensor_channel chan,
								  enum sensor_attribute attr,
								  struct sensor_value *val)
{
	if (chan < SENSOR_CHAN_M080R_START || chan > SENSOR_CHAN_SLEEP)
	{
		return -ENOTSUP;
	}
	// if (attr < SENSOR_ATTR_M080R_START || attr > SENSOR_ATTR_DEEP_SLEEP)
	// {
	// 	return -ENOTSUP;
	// }
	return sensor_action[chan - SENSOR_CHAN_M080R_START](dev, attr, val);
}

static int m080r_register_fingerprint(const struct device *dev, enum sensor_attribute attr, struct sensor_value *val)
{
	const struct m080r_config *config = dev->config;
	struct m080r_data *data = dev->data;
	struct device *uart_device = config->uart;

	for (uint8_t i = 1; i < 7; i++)
	{
		uint8_t data_buff[] = {i};
		prepare_header(data, sizeof(data_buff));
		set_command(data, CMD_REGISTER_FINGERPRINT_ID);
		set_data(data, data_buff);
		set_checksum(data);

		SENT_HEADER();
		SENT_APP_DATA();
		GET_RESPONSE_AND_PRINT();
		k_sleep(K_MSEC(1000));
	}

	prepare_header(data, 0);
	set_command(data, CMD_REGISTER_FINGERPRINT_QUERY_ID);
	set_data(data, NULL);
	set_checksum(data);
	SENT_HEADER();
	SENT_APP_DATA();
	GET_RESPONSE_AND_PRINT();
	k_sleep(K_MSEC(1000));

	data->last_id = tx_buf[22]; // id of the last registered fingerprint
	val->val1 = tx_buf[20];		// error code

	return 0;
}
static int m080r_save_fingerprint(const struct device *dev, enum sensor_attribute attr, struct sensor_value *val)
{
		const struct m080r_config *config = dev->config;
	struct m080r_data *data = dev->data;
	struct device *uart_device = config->uart;

	uint8_t data_buff[] = {0x00, data->last_id}; // id of the last registered fingerprint
	prepare_header(data, sizeof(data_buff));
	set_command(data, CMD_SAVE_FINGERPRINT_ID);
	set_data(data, data_buff);
	set_checksum(data);
	SENT_HEADER();
	SENT_APP_DATA();
	GET_RESPONSE_AND_PRINT();

	k_sleep(K_MSEC(1000));

	prepare_header(data, 0);
	set_command(data, CMD_SAVE_FINGERPRINT_QUERY_ID);
	set_data(data, NULL);
	set_checksum(data);
	SENT_HEADER();
	SENT_APP_DATA();
	GET_RESPONSE_AND_PRINT();

	k_sleep(K_MSEC(1000));

	val->val1 = tx_buf[22]; // id of fingerprint

	return 0;
}
static int m080r_match_fingerprint(const struct device *dev, enum sensor_attribute attr, struct sensor_value *val)
{

	const struct m080r_config *config = dev->config;
	struct m080r_data *data = dev->data;
	struct device *uart_device = config->uart;

	prepare_header(data, 0);
	set_command(data, CMD_MATCH_FINGERPRINT_ID);
	set_data(data, NULL);
	set_checksum(data);
	SENT_HEADER();
	SENT_APP_DATA();
	GET_RESPONSE_AND_PRINT();

	k_sleep(K_MSEC(1000));

	prepare_header(data, 0);
	set_command(data, CMD_MATCH_FINGERPRINT_QUERY_ID);
	set_data(data, NULL);
	set_checksum(data);
	SENT_HEADER();
	SENT_APP_DATA();
	GET_RESPONSE_AND_PRINT();

	k_sleep(K_MSEC(1000));

	val->val1 = tx_buf[26]; // id of fingerprint
	val->val2 = tx_buf[22]; // matching result

	return 0;
}

static int m080r_sleep(const struct device *dev, enum sensor_attribute attr, struct sensor_value *val)
{
	const struct m080r_config *config = dev->config;
	struct m080r_data *data = dev->data;
	struct device *uart_device = config->uart;
	uint8_t data_buff[] = {0x00};
	data_buff[0] = attr == SENSOR_ATTR_NORMAL_SLEEP ? 0x00 : 0x01;
	
	prepare_header(data, sizeof(data_buff));
	set_command(data, CMD_SLEEP_ID);
	set_data(data, data_buff);
	set_checksum(data);
	SENT_HEADER();
	SENT_APP_DATA();
	GET_RESPONSE_AND_PRINT();

	k_sleep(K_MSEC(1000));
	return 0;
}

static const struct sensor_driver_api m080r_api = {
	.sample_fetch = &m080r_heartbeat,
	.attr_set = &m080r_fingerprint_action,
};

static int m080r_init(const struct device *dev)
{
	int ret;
	struct m080r_config *config = dev->config;
	if (!device_is_ready(config->uart))
	{
		LOG_ERR("UART device not ready!");
		return -ENODEV;
	}

	uart_irq_callback_user_data_set(config->uart, serial_cb, NULL);
	uart_irq_rx_enable(config->uart);

	ret = m080r_init_int_pin(&config->int_gpios, &int1_cb, m080r_int1_callback);
	printk("m080r_init_int_pin: %d\n", ret);
	ret = m080r_init_pwr_en_pin(&config->pwr_en_gpios);
	printk("m080r_init_pwr_en_pin: %d\n", ret);

	return 0;
}

static int m080r_driver_pm_action(const struct device *dev,
								  enum pm_device_action action)
{
	struct m080r_config *config = dev->config;
	switch (action)
	{
	case PM_DEVICE_ACTION_SUSPEND:
		/* suspend the device */
		printk("m080r_driver_pm_action: suspend\n");
		// m080r_sleep(dev, NULL, NULL);
		// write transistor pin to high
		gpio_pin_set_dt(&config->pwr_en_gpios, 0);
		break;
	case PM_DEVICE_ACTION_RESUME:
		/* resume the device */
		// write transistor pin to low
		printk("m080r_driver_pm_action: resume\n");
		gpio_pin_set_dt(&config->pwr_en_gpios, 1);
		break;
	case PM_DEVICE_ACTION_TURN_ON:
		/*
		 * powered on the device, used when the power
		 * domain this device belongs is resumed.
		 */

		break;
	case PM_DEVICE_ACTION_TURN_OFF:
		/*
		 * power off the device, used when the power
		 * domain this device belongs is suspended.
		 */

		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

#define M080R_INIT(i)                                                         \
	static struct m080r_data m080r_data_##i;                                  \
                                                                              \
	static const struct m080r_config m080r_config_##i = {                     \
		.uart = DEVICE_DT_GET(DT_INST_PHANDLE(i, uart_node)),                 \
		.int_gpios = GPIO_DT_SPEC_INST_GET_BY_IDX_OR(i, irq_gpios, 0, {}),    \
		.pwr_en_gpios = GPIO_DT_SPEC_INST_GET_BY_IDX_OR(i, irq_gpios, 1, {}), \
	};                                                                        \
	PM_DEVICE_DT_INST_DEFINE(i, m080r_driver_pm_action);                      \
	DEVICE_DT_INST_DEFINE(i, m080r_init, PM_DEVICE_DT_INST_GET(i),            \
						  &m080r_data_##i,                                    \
						  &m080r_config_##i, POST_KERNEL,                     \
						  CONFIG_SENSOR_INIT_PRIORITY, &m080r_api);

DT_INST_FOREACH_STATUS_OKAY(M080R_INIT)