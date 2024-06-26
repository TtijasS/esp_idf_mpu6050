#ifndef MY_UART_COM_H
#define MY_UART_COM_H

#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "constants.h"
#include "data_structs.h"

// External declarations for constants and structs
extern const uart_port_t uart_num;
extern const int uart_buffer_size;
extern const uart_config_t uart_config;

// Function prototypes
void uart_init();
void uart_send_accel_data(mpu_data_type *mpu_data);

#endif // UART_COM_H
