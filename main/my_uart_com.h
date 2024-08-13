#ifndef MY_UART_COM_H
#define MY_UART_COM_H

#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "constants.h"
#include "data_structs.h"
#include "esp_log.h"

// External declarations for constants and structs
extern const uart_port_t uart_num;
extern const int uart_buffer_size;
extern const uart_config_t uart_config;

// Function prototypes
void uart_init();
void uart_send_accel_data(mpuDataType *mpu_data_t);
int uart_send_fft_components(uint8_t *metadata_buffer, size_t metadata_size, uint8_t *indices_buffer, size_t indices_size, uint8_t *complex_data_buffer, size_t complex_size);

#endif // UART_COM_H
