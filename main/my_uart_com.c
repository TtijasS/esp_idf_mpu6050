#include "my_uart_com.h"

// Constants
const uart_port_t uart_num = UART_NUM_0;
const int uart_buffer_size = (UART_BUFFER_SIZE * sizeof(float));

/**
 * @brief Uart configuration
 * 
 */
const uart_config_t uart_config = {
    .baud_rate = UART_BAUD_RATE, // You can change this to your required baud rate
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
};

/**
 * @brief Init custom uart communication
 * 
 */
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

/**
 * @brief UART write metadata, indices and complex data buffers with distinct separator flags
 * 
 * @param metadata_buffer metadata holding the number of readings and the number of most significant components information
 * @param metadata_size size of metadata buffer
 * @param indices_buffer indices of ms components
 * @param indices_size size of indices buffer
 * @param complex_data_buffer real and complex parts of ms components
 * @param complex_size size of complex data buffer
 * @return 0 OK
 * @return -1 NULL operators passed
 * @return -2 failed to write metadata buffer
 * @return -3 failed to write indices buffer
 * @return -4 failed to write complex data buffer
 */
int uart_send_fft_components(uint8_t *metadata_buffer, size_t metadata_size, uint8_t *indices_buffer, size_t indices_size, uint8_t *complex_data_buffer, size_t complex_size)
{
    if (metadata_buffer == NULL || indices_buffer == NULL || complex_data_buffer == NULL)
    {
        return -1;
    }
    const char* TAG = "SEND FFT";
    // ESP_LOGI(TAG, "metadata_size: %u, data_size: %u", metadata_size, data_size);
    // Send metadata
    uart_write_bytes(uart_num, "\xfd\xfd\xfd\xfd\xfd", 5); // Start of transmission
    if (uart_write_bytes(uart_num, (const char *)metadata_buffer, metadata_size) != 0)
    {
        return -2;
    }
    uart_write_bytes(uart_num, "\xff\xfd\xfd\xfd\xff", 5); // Start of transmission

    // Send complex data indices
    uart_write_bytes(uart_num, "\xfc\xfc\xfc\xfc\xfc", 5); // Start of transmission
    if (uart_write_bytes(uart_num, (const char *)indices_buffer, indices_size) != 0)
    {
        return -3;
    }
    uart_write_bytes(uart_num, "\xff\xfc\xfc\xfc\xff", 5); // Start of transmission
    
    // Send complex data
    uart_write_bytes(uart_num, "\xfe\xfe\xfe\xfe\xfe", 5); // Start of transmission
    if (uart_write_bytes(uart_num, (const char *)complex_data_buffer, complex_size) != 0)
    {
        return -4;
    }
    uart_write_bytes(uart_num, "\xff\xfe\xfe\xfe\xff", 5); // Start of transmission
    
    return 0;
}
