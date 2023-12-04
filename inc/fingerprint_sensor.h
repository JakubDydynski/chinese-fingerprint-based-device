#ifndef FINGERPRINT_SENSOR_H
#define FINGERPRINT_SENSOR_H
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
status_t sensor_send_command(frame_header_t* header, transmission_frame_t* content, sensor_t* sensor);
status_t sensor_receive_response(frame_header_t* header, response_frame_t* content);
status_t sensor_init(sensor_t* sensor);
#endif // FINGERPRINT_SENSOR_H
