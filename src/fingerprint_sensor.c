#include "fingerprint_sensor.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>


static char command[] = {0xF1, 0x1F, 0xE2, 0x2E, 0xB6, 0x6B, 0xA8, 0x8A, 0x00, 0x07, 0x86, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0xFA};
/*
 * send a null-terminated string character by character to the UART interface
 */
void send_uart(char *buf, int n, const struct device *const uart_device)
{
	for (int i = 0; i < n; i++) {
		uart_poll_out(uart_device, buf[i]);
	}
}

// trzeba bedzie się podpiąć do DT_INST_FOREACH_STATUS_OKAY(M080R_CREATE_INST), przykład drivera bmi 270
// 1. driver jest uruchamiany przed aplikacją
// 2. driver znajduje się w innej przestrzeni uprawnień niż aplikacja
// TODO: const struct device *dev 
// TODO: analizator ramek, interpretuje i wysyła wyżej
status_t sensor_send_command(sensor_t* sensor, frame_header_t* header, transmission_frame_t* content)
{
	// header
	send_uart((char*)header, sizeof(header), sensor->uart);
	// app layer
	send_uart(content->password, sizeof(content->password), sensor->uart);
	send_uart(content->command, sizeof(content->command), sensor->uart);
	send_uart(content->data, header->length, sensor->uart);
	send_uart(&(content->checksum), sizeof(content->checksum), sensor->uart);

	return OK;
}

status_t sensor_receive_response(frame_header_t* header, response_frame_t* content)
{
	return OK;
}

status_t sensor_init(sensor_t* sensor)
{
	
	send_uart(command, sizeof(command), sensor->uart);
	return OK;
}


/*
 * Copyright (c) 2021 Bosch Sensortec GmbH
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */



#define DT_DRV_COMPAT chinese_M080R





LOG_MODULE_REGISTER(M080R, CONFIG_SENSOR_LOG_LEVEL);


#define M080R_WR_LEN                           256
#define M080R_CONFIG_FILE_RETRIES              15
#define M080R_CONFIG_FILE_POLL_PERIOD_US       10000
#define M080R_INTER_WRITE_DELAY_US             1000

struct M080R_config {
	const struct M080R_bus_io *bus_io;
};

/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for M080Rs accessed via uart.
 */




static int M080R_bus_check_uart(const struct uart_dt_spec* uart)
{
	return uart_is_ready_dt(uart) ? 0 : -ENODEV;
}

static int M080R_reg_read_uart(const union M080R_bus *bus,
			       uint8_t start, uint8_t *data, uint16_t len)
{
	int ret;
	uint8_t addr;

	k_usleep(M080R_uart_ACC_DELAY_US);
	return 0;
}

static int M080R_reg_write_uart(const union M080R_bus *bus, uint8_t start,
				const uint8_t *data, uint16_t len)
{
	int ret;
	uint8_t addr;

	k_usleep(M080R_uart_ACC_DELAY_US);
	return 0;
}

static int M080R_bus_init_uart(const union M080R_bus *bus)
{
	uint8_t tmp;

	/* Single read of uart initializes the chip to uart mode
	 */
	return M080R_reg_read_uart(bus, M080R_REG_CHIP_ID, &tmp, 1);
}

const struct M080R_bus_io M080R_bus_io_uart = {
	.check = M080R_bus_check_uart,
	.read = M080R_reg_read_uart,
	.write = M080R_reg_write_uart,
	.init = M080R_bus_init_uart,
};




static inline int M080R_bus_check(const struct device *dev)
{
	const struct M080R_config *cfg = dev->config;

	return cfg->bus_io->check();
}

static inline int M080R_bus_init(const struct device *dev)
{
	const struct M080R_config *cfg = dev->config;

	return cfg->bus_io->init();
}

static int M080R_reg_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
	const struct M080R_config *cfg = dev->config;

	return cfg->bus_io->read(reg, data, length);
}

static int M080R_reg_write(const struct device *dev, uint8_t reg,
			    const uint8_t *data, uint16_t length)
{
	const struct M080R_config *cfg = dev->config;

	return cfg->bus_io->write(reg, data, length);
}

static int M080R_reg_write_with_delay(const struct device *dev,
				       uint8_t reg,
				       const uint8_t *data,
				       uint16_t length,
				       uint32_t delay_us)
{
	int ret = 0;

	ret = M080R_reg_write(dev, reg, data, length);
	if (ret == 0) {
		k_usleep(delay_us);
	}
	return ret;
}

// sensor_init,   sensor_enter_sleep_mode	   sensor_enter_normal_mode,   sensor_heartbeat,


static int M080R_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
}

static int M080R_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
}

static int M080R_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
}

static int M080R_init(const struct device *dev)
{
	int ret;
	struct M080R_data *data = dev->data; // data returned by sensor response_frame_t
	uint8_t chip_id;
	uint8_t soft_reset_cmd;
	uint8_t init_ctrl;
	uint8_t msg;
	uint8_t tries;
	uint8_t adv_pwr_save;

	ret = M080R_bus_check(dev);
	if (ret < 0) {
		LOG_ERR("Could not initialize bus");
		return ret;
	}

	// data->acc_odr = M080R_ACC_ODR_100_HZ;
	// data->acc_range = 8;
	// data->gyr_odr = M080R_GYR_ODR_200_HZ;
	// data->gyr_range = 2000;

	// k_usleep(M080R_POWER_ON_TIME);

	ret = M080R_bus_init(dev);
	if (ret != 0) {
		LOG_ERR("Could not initiate bus communication");
		return ret;
	}

	// ret = M080R_reg_read(dev, M080R_REG_CHIP_ID, &chip_id, 1);
	// if (ret != 0) {
	// 	return ret;
	// }

	// if (chip_id != M080R_CHIP_ID) {
	// 	LOG_ERR("Unexpected chip id (%x). Expected (%x)",
	// 		chip_id, M080R_CHIP_ID);
	// 	return -EIO;
	// }

	// soft_reset_cmd = M080R_CMD_SOFT_RESET;
	// ret = M080R_reg_write(dev, M080R_REG_CMD, &soft_reset_cmd, 1);
	// if (ret != 0) {
	// 	return ret;
	// }

	// k_usleep(M080R_SOFT_RESET_TIME);

	// ret = M080R_reg_read(dev, M080R_REG_PWR_CONF, &adv_pwr_save, 1);
	// if (ret != 0) {
	// 	return ret;
	// }

	// adv_pwr_save = M080R_SET_BITS_POS_0(adv_pwr_save,
	// 				     M080R_PWR_CONF_ADV_PWR_SAVE,
	// 				     M080R_PWR_CONF_ADV_PWR_SAVE_DIS);
	// ret = M080R_reg_write_with_delay(dev, M080R_REG_PWR_CONF,
	// 				  &adv_pwr_save, 1,
	// 				  M080R_INTER_WRITE_DELAY_US);
	// if (ret != 0) {
	// 	return ret;
	// }

	// init_ctrl = M080R_PREPARE_CONFIG_LOAD;
	// ret = M080R_reg_write(dev, M080R_REG_INIT_CTRL, &init_ctrl, 1);
	// if (ret != 0) {
	// 	return ret;
	// }

	// ret = write_config_file(dev);

	// if (ret != 0) {
	// 	return ret;
	// }

	// init_ctrl = M080R_COMPLETE_CONFIG_LOAD;
	// ret = M080R_reg_write(dev, M080R_REG_INIT_CTRL, &init_ctrl, 1);
	// if (ret != 0) {
	// 	return ret;
	// }

	// /* Timeout after M080R_CONFIG_FILE_RETRIES x
	//  * M080R_CONFIG_FILE_POLL_PERIOD_US microseconds.
	//  * If tries is M080R_CONFIG_FILE_RETRIES by the end of the loop,
	//  * report an error
	//  */
	// for (tries = 0; tries <= M080R_CONFIG_FILE_RETRIES; tries++) {
	// 	ret = M080R_reg_read(dev, M080R_REG_INTERNAL_STATUS, &msg, 1);
	// 	if (ret != 0) {
	// 		return ret;
	// 	}

	// 	msg &= M080R_INST_MESSAGE_MSK;
	// 	if (msg == M080R_INST_MESSAGE_INIT_OK) {
	// 		break;
	// 	}

	// 	k_usleep(M080R_CONFIG_FILE_POLL_PERIOD_US);
	// }

	// if (tries == M080R_CONFIG_FILE_RETRIES) {
	// 	return -EIO;
	// }

	// adv_pwr_save = M080R_SET_BITS_POS_0(adv_pwr_save,
	// 				     M080R_PWR_CONF_ADV_PWR_SAVE,
	// 				     M080R_PWR_CONF_ADV_PWR_SAVE_EN);
	// ret = M080R_reg_write_with_delay(dev, M080R_REG_PWR_CONF,
	// 				  &adv_pwr_save, 1,
	// 				  M080R_INTER_WRITE_DELAY_US);

	return ret;
}

static const struct sensor_driver_api M080R_driver_api = {
	.sample_fetch = M080R_sample_fetch,
	.channel_get = M080R_channel_get,
	.attr_set = M080R_attr_set
};

/* Initializes a struct M080R_config for an instance on an uart bus. */
#define M080R_CONFIG_UART(inst)			       \
	{					       \
		.bus.uart = UART_DT_SPEC_INST_GET(inst), \
		.bus_io = &M080R_bus_io_uart,	       \
	}


#define M080R_CREATE_INST(inst)					\
									\
	static struct M080R_data M080R_drv_##inst;			\
									\
	static const struct M080R_config M080R_config_##inst =	\
		COND_CODE_1(DT_INST_ON_BUS(inst, uart),			\
			    (M080R_CONFIG_uart(inst)); \
									\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			      M080R_init,				\
			      NULL,					\
			      &M080R_drv_##inst,			\
			      &M080R_config_##inst,			\
			      POST_KERNEL,				\
			      CONFIG_SENSOR_INIT_PRIORITY,		\
			      &M080R_driver_api);

DT_INST_FOREACH_STATUS_OKAY(M080R_CREATE_INST)
