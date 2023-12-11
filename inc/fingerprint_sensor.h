#ifndef FINGERPRINT_SENSOR_H
#define FINGERPRINT_SENSOR_H
/* ------------------------------------------------------------------------------------------------------------------ */
/*                                                      includes                                                      */
/* ------------------------------------------------------------------------------------------------------------------ */
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

/* ------------------------------------------------------------------------------------------------------------------ */
/*                                                       defines                                                      */
/* ------------------------------------------------------------------------------------------------------------------ */
#define M080R_REG_CHIP_ID         0x00
#define M080R_REG_ERROR           0x02
#define M080R_REG_STATUS          0x03
#define M080R_REG_AUX_X_LSB       0x04

#define M080R_uart_ACC_DELAY_US	  500

/* ------------------------------------------------------------------------------------------------------------------ */
/*                                                       macros                                                       */
/* ------------------------------------------------------------------------------------------------------------------ */
#define M080R_SET_BITS(reg_data, bitname, data)		  \
	((reg_data & ~(bitname##_MSK)) | ((data << bitname##_POS) \
					  & bitname##_MSK))
#define M080R_SET_BITS_POS_0(reg_data, bitname, data) \
	((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))

/* ------------------------------------------------------------------------------------------------------------------ */
/*                                                        types                                                       */
/* ------------------------------------------------------------------------------------------------------------------ */
struct M080R_data {
	int16_t ax, ay, az, gx, gy, gz;
	uint8_t acc_range, acc_odr, gyr_odr;
	uint16_t gyr_range;
};

typedef int (*M080R_bus_check_fn)();
typedef int (*M080R_bus_init_fn)();
typedef int (*M080R_reg_read_fn)(
				  uint8_t start,
				  uint8_t *data,
				  uint16_t len);
typedef int (*M080R_reg_write_fn)(
				   uint8_t start,
				   const uint8_t *data,
				   uint16_t len);


struct M080R_bus_io {
	M080R_bus_check_fn check;
	M080R_reg_read_fn read;
	M080R_reg_write_fn write;
	M080R_bus_init_fn init;
};

extern const struct M080R_bus_io M080R_bus_io_uart;




#include <zephyr/kernel.h>
typedef enum status{
	OK,
	ERROR
}status_t;

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

typedef struct{
    struct device * uart;
}sensor_t;

// Your code goes here
status_t sensor_send_command(sensor_t* sensor, frame_header_t* header, transmission_frame_t* content);
status_t sensor_receive_response(frame_header_t* header, response_frame_t* content);
status_t sensor_init(sensor_t* sensor);
#endif // FINGERPRINT_SENSOR_H
