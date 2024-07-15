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

__attribute__((aligned(16))) float data_sampled[N_SAMPLES]; // Sampled data

void app_main(void)
{
    printf("\n");
    ESP_LOGI(TAG, "Start Example.");

    // Init I2C and UART
    i2c_init();
    // Configure MPU6050
    mpu_initial_setup(&i2c_buffer);
    // Configure UART
    uart_init();
    uart_set_baudrate(uart_num, 115200);

    // Init FFT memory
    fft_init();

    ESP_LOGI(TAG, "i2c frequency Hz: %lu", i2c_master_device_config.scl_speed_hz);
    // mpu_data_calibrate(&i2c_buffer, &mpu_data, 100);
    // mpu_avg_err_print(&mpu_data);

    int i = 0;
    uint64_t deltatime = 0;
    
    deltatime = esp_timer_get_time();
    while (1)
    {
        if (!mpu_data_read_extract(&i2c_buffer, &mpu_data))
        {
            ESP_LOGE(TAG, "Error reading MPU6050 data.");
            continue;
        }
        mpu_data_substract_err(&mpu_data, false);
        mpu_data_to_fs(&mpu_data, false);
        memcpy(&data_sampled[i], &mpu_data.accel_gyro_g[0], sizeof(float)); // X-axis
        // printf("%.4f\n", mpu_data.accel_gyro_g[3]);
        // vTaskDelay(pdMS_TO_TICKS(1));
        i++;
        if (i == N_SAMPLES)
        {
            deltatime = esp_timer_get_time() - deltatime;
            i = 0;
            ESP_LOGI(TAG, "Data successfully sampled in %llu ms, %llu us per sample batch, %llu us per byte", deltatime / 1000, deltatime / N_SAMPLES, deltatime / N_SAMPLES / 14);
            break;
        }
    }

    for (int i = 0; i < 40; i++)
        printf("%.3f; ", data_sampled[i]);
    
    printf("\n");

    vTaskDelay(pdMS_TO_TICKS(100));
    deltatime = esp_timer_get_time();
    while (1)
    {
        if (!mpu_data_read_extract_accel(&i2c_buffer, &mpu_data))
        {
            ESP_LOGE(TAG, "Error reading MPU6050 data.");
            continue;
        }
        mpu_data_substract_err(&mpu_data, true);
        mpu_data_to_fs(&mpu_data, true);
        memcpy(&data_sampled[i], &mpu_data.accel_gyro_g[0], sizeof(float)); // X-axis
        // printf("%.4f\n", mpu_data.accel_gyro_g[3]);
        // vTaskDelay(pdMS_TO_TICKS(1));
        i++;
        if (i == N_SAMPLES)
        {
            deltatime = esp_timer_get_time() - deltatime;
            i = 0;
            ESP_LOGI(TAG, "Data successfully sampled in %llu ms, %llu us per sample batch, %llu us per byte", deltatime / 1000, deltatime / N_SAMPLES, deltatime / N_SAMPLES / 6);
            break;
        }
    }

    for (int i = 0; i < 40; i++)
        printf("%.3f; ", data_sampled[i]);
    printf("\n");

    // vTaskDelay(pdMS_TO_TICKS(100));
   
    fft_apply_hann_windowing(data_sampled, fft_window_arr, fft_complex_arr);
    ESP_LOGI(TAG, "Window prepared and data merged to fft_components");

    fft_calculate_re_im(fft_complex_arr, N_SAMPLES);
    ESP_LOGI(TAG, "FFT calculated");
    vTaskDelay(pdMS_TO_TICKS(50));

    fft_calculate_magnitudes(fft_magnitudes_arr, MAGNITUDES_SIZE);

    fft_sort_magnitudes(fft_magnitudes_arr, MAGNITUDES_SIZE);
    ESP_LOGI(TAG, "Magnitudes sorted");

    uint32_t n_msb_components = fft_percentile_n_components(80, MAGNITUDES_SIZE);

    fft_send_msb_components_over_uart(N_SAMPLES, n_msb_components);

    ESP_LOGI(TAG, "End Example.");

    // delete i2c handle
    ESP_ERROR_CHECK(i2c_del_master_bus(i2c_master_bus_handle));

}

