#include "my_uart_com.h"

// Constants
const uart_port_t uart_num = UART_NUM_0;
const int uart_buffer_size = (UART_BUFFER_SIZE * sizeof(float));

// Constant struct definition
const uart_config_t uart_config = {
    .baud_rate = UART_BAUD_RATE, // You can change this to your required baud rate
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
};

// Function definitions
void uart_init()
{
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins (TX: GPIO43, RX: GPIO44, RTS: unused, CTS: unused)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, NULL, 0));
}

void uart_send_accel_data(mpuDataType *mpu_data_t)
{
    uint8_t data[sizeof(float) * 3];
    memcpy(data, &mpu_data_t->accel_gyro_g[3], sizeof(float) * 3);
    uart_write_bytes(uart_num, (const char *)data, sizeof(float) * 3);
    uart_write_bytes(uart_num, "\t", 1);
}

void uart_send_fft_components(uint8_t *metadata_buffer, size_t metadata_size, uint8_t *data_buffer, size_t data_size)
{
    // ESP_LOGI(TAG, "metadata_size: %u, data_size: %u", metadata_size, data_size);
    // Send metadata
    uart_write_bytes(uart_num, "\xfd\xfd\xfd\xfd\xfd", 5); // Start of transmission
    uart_write_bytes(uart_num, (const char *)metadata_buffer, metadata_size);
    uart_write_bytes(uart_num, "\xff\xfd\xfd\xfd\xff", 5); // Start of transmission
    // Send data
    uart_write_bytes(uart_num, "\xfe\xfe\xfe\xfe\xfe", 5); // Start of transmission
    uart_write_bytes(uart_num, (const char *)data_buffer, data_size);
    uart_write_bytes(uart_num, "\xff\xfe\xfe\xfe\xff", 5); // Start of transmission
}
