#include "uart_com.h"

//Define UART configuration
const uart_config_t uart_config = {
	.baud_rate = 3000000, // You can change this to your required baud rate
	.data_bits = UART_DATA_8_BITS,
	.parity = UART_PARITY_DISABLE,
	.stop_bits = UART_STOP_BITS_1,
	.flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

void uart_init()
{
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins (TX: GPIO43, RX: GPIO44, RTS: unused, CTS: unused)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, NULL, 0));
}

void send_accel_data_over_uart(mpu_data_type *mpu_data)
{
    uint8_t data[sizeof(float) * 3];
    memcpy(data, &mpu_data->accel_gyro_g[3], sizeof(float) * 3);
    uart_write_bytes(uart_num, (const char *)data, sizeof(float) * 3);
    uart_write_bytes(uart_num, "\t", 1);
}