#include <string.h> // Include string.h for memcpy
#include <stdio.h> // Include stdio.h for standard input/output functions
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> // Include FreeRTOS task header
#include "driver/uart.h"
#include "constants.h"
#include "mpu6050.h"
#include "my_i2c_config.h"
#include "data_structs.h"
#include "custom_functions.h"
#include "driver/uart.h"
#include <string.h>

const uart_port_t uart_num = UART_NUM_0;
const int uart_buffer_size = (1024 * sizeof(float));

void init_uart()
{
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 3000000, // You can change this to your required baud rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins (TX: GPIO43, RX: GPIO44, RTS: unused, CTS: unused)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, NULL, 0));
}

void send_float_over_uart(float value)
{
    uint8_t data[sizeof(float)];
    memcpy(data, &value, sizeof(float));
    uart_write_bytes(uart_num, (const char *)data, sizeof(float));
}

void send_accel_data_over_uart(float value1, float value2, float value3)
{
    // Send first float
    send_float_over_uart(value1);

    // Send second float
    send_float_over_uart(value2);

    // Send third float
    send_float_over_uart(value3);
    // Send newline
    uart_write_bytes(uart_num, "\n\r", 2);
}

void app_main(void)
{
    // Setup master handles and initialize I2C
    init_i2c();
    init_uart();

    // Setup the MPU6050 registers
    mpu_initial_setup(&i2c_buffer);

    while (1)
    {
        mpu_data_read_extract(&i2c_buffer, &mpu_data);
        mpu_data_substract_err(&mpu_data);
        mpu_data_to_fs(&mpu_data);

        // Send the three floats over UART
        send_accel_data_over_uart(mpu_data.accel_gyro_g[3], mpu_data.accel_gyro_g[4], mpu_data.accel_gyro_g[5]);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    ESP_ERROR_CHECK(i2c_del_master_bus(master_bus_handle));
}
