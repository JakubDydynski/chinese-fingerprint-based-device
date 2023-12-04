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

// trzeba bedzie się podpiąć do DT_INST_FOREACH_STATUS_OKAY(BMI270_CREATE_INST), przykład drivera bmi 270
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