#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "constants.h"
#include "mpu6050.h"
#include "my_i2c_com.h"
#include "data_structs.h"
#include "custom_functions.h"
#include "my_uart_com.h"
#include "my_fft.h"

__attribute__((aligned(16))) float data_sampled_x[N_SAMPLES]; // Sampled data
__attribute__((aligned(16))) float data_sampled_y[N_SAMPLES]; // Sampled data
__attribute__((aligned(16))) float data_sampled_z[N_SAMPLES]; // Sampled data

void app_main(void)
{
    ESP_LOGI(TAG, "Start Example.");

    // Init I2C and UART
    i2c_init();
    // Configure MPU6050
    mpu_initial_setup(&i2c_buffer_t);
    // Configure UART
    uart_init();
    // Init FFT memory
    fft_init();

    ESP_LOGI(TAG, "i2c frequency Hz: %lu", i2c_master_device_config.scl_speed_hz);
    // mpu_data_calibrate(&i2c_buffer_t, &mpu_data_t, 100);
    // mpu_avg_err_print(&mpu_data_t);


    // uint64_t deltatime = esp_timer_get_time();
    size_t i = 0;
    TickType_t last_wake_time = xTaskGetTickCount();
    while (i < N_SAMPLES)
    {
        if (!mpu_data_read_extract_accel(&i2c_buffer_t, &mpu_data_t))
        {
            ESP_LOGE(TAG, "Error reading MPU6050 data.");
            continue;
        }
        // true == only the accelerometer data handled
        mpu_data_substract_err(&mpu_data_t, true);
        mpu_data_to_fs(&mpu_data_t, true);
        // if ((i > 0) && ((mpu_data_t.accel_gyro_g[0] == data_sampled_x[i-1]) || (mpu_data_t.accel_gyro_g[1] == data_sampled_y[i-1]) || (mpu_data_t.accel_gyro_g[2] == data_sampled_z[i-1])))
        // {
        //     continue;
        // }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
        memcpy(&data_sampled_x[i], &mpu_data_t.accel_gyro_g[0], sizeof(float)); // X-axis
        memcpy(&data_sampled_y[i], &mpu_data_t.accel_gyro_g[1], sizeof(float)); // X-axis
        memcpy(&data_sampled_z[i], &mpu_data_t.accel_gyro_g[2], sizeof(float)); // X-axis
        i++;
        last_wake_time = xTaskGetTickCount();
    }
    // deltatime = esp_timer_get_time() - deltatime;
    // ESP_LOGI(TAG, "Time elapsed: %llu ms", deltatime);
    // ESP_LOGI(TAG, "%llu us per batch", deltatime / N_SAMPLES);
    for (size_t i = 0; i < N_SAMPLES; i++)
    {
        printf("%.5f, ", data_sampled_x[i]);
    }

    // fft_prepare_window();

    fft_prepare_complex_arr(data_sampled_x, fft_window_arr, fft_complex_arr, N_SAMPLES);
    ESP_LOGI(TAG, "Window prepared and data merged to fft_components");

    fft_calculate_re_im(fft_complex_arr, N_SAMPLES);
    ESP_LOGI(TAG, "FFT calculated");

    fft_calculate_magnitudes(fft_magnitudes_arr, MAGNITUDES_SIZE);

    fft_sort_magnitudes(fft_magnitudes_arr, MAGNITUDES_SIZE);
    ESP_LOGI(TAG, "Magnitudes sorted");

    uint32_t n_msb_components = fft_percentile_n_components(50, MAGNITUDES_SIZE);

    fft_send_msb_components_over_uart(N_SAMPLES, n_msb_components);

    ESP_LOGI(TAG, "End Example.");

    // delete i2c handle
    ESP_ERROR_CHECK(i2c_del_master_bus(i2c_master_bus_handle));
}
