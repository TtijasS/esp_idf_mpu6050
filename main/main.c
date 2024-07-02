#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "constants.h"
#include "mpu6050.h"
#include "my_i2c_com.h"
#include "data_structs.h"
#include "custom_functions.h"
#include "my_uart_com.h"
#include "my_fft.h"

// __attribute__((aligned(16))) float data_sampled[3][N_SAMPLES];   // Sampled data; x, y, z axes
__attribute__((aligned(16))) float data_sampled[N_SAMPLES]; // Sampled data

void app_main(void)
{
    ESP_LOGI(TAG, "Start Example.");

    // Init I2C settings
    i2c_init();
    // Init custom UART settings (commented out to check FFT outputs)
    // uart_init();
    // Initialize FFT
    fft_init();

    // Setup the MPU6050 registers
    mpu_initial_setup(&i2c_buffer);

    int i = 0;
    while (1)
    {
        // Read and process MPU6050 data

        if (!mpu_data_read_extract(&i2c_buffer, &mpu_data))
        {
            ESP_LOGE(TAG, "Error reading MPU6050 data.");
            continue;
        }
        mpu_data_substract_err(&mpu_data);
        mpu_data_to_fs(&mpu_data);

        // // Copy data to respective axis arrays
        // memcpy(&data_sampled[0][i], &mpu_data.accel_gyro_g[3], sizeof(float)); // X-axis
        // memcpy(&data_sampled[1][i], &mpu_data.accel_gyro_g[4], sizeof(float)); // Y-axis
        // memcpy(&data_sampled[2][i], &mpu_data.accel_gyro_g[5], sizeof(float)); // Z-axis

        memcpy(&data_sampled[i], &mpu_data.accel_gyro_g[3], sizeof(float)); // X-axis
        i++;
        if (i == N_SAMPLES)
        {
            // prepare window array and then apply in on data_sampled. Results are stored to fft_components
            fft_apply_window_on_fft_complex(data_sampled, window, fft_components);
            
            // Run the fft calculations
            fft_calculate_re_im(fft_components, N_SAMPLES*2);

            // Calculate magnitudes
            fft_calculate_magnitudes(magnitudes_indexed, N_SAMPLES);

            fft_send_percentiles_over_uart(95.0, N_SAMPLES);
            // fft_plot_magnitudes(N_SAMPLES, 0, 30);
            i = 0;
        }

        // Use the configured UART settings to send accelerometer data
        // uart_send_accel_data(&mpu_data);

        // Adjust delay as needed
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_ERROR_CHECK(i2c_del_master_bus(master_bus_handle));
}
