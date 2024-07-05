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

SemaphoreHandle_t xSemaphoreMPUDataReady;
SemaphoreHandle_t xSemaphoreFFTReady;

void task_mpu_data_reading(void *pvParameters)
{
    ESP_LOGI(TAG, "Sampling task started");
    int i = 0;
    while (1)
    {
        if (!mpu_data_read_extract(&i2c_buffer, &mpu_data))
        {
            ESP_LOGE(TAG, "Error reading MPU6050 data.");
            continue;
        }
        mpu_data_substract_err(&mpu_data);
        mpu_data_to_fs(&mpu_data);

        memcpy(&data_sampled[i], &mpu_data.accel_gyro_g[3], sizeof(float)); // X-axis
        // printf("%.2f\n", data_sampled[i]);
        i++;
        if (i == N_SAMPLES)
        {
            i = 0;
            // Signal FFT task
            ESP_LOGI(TAG, "Data successfully sampled");
            break;
            xSemaphoreGive(xSemaphoreMPUDataReady);
        }
        vTaskDelay(pdMS_TO_TICKS(10));

    }
}

void task_fft_calculations(void *pvParameters)
{
    while (1)
    {
        // Wait for data to be ready
        if (xSemaphoreTake(xSemaphoreMPUDataReady, portMAX_DELAY) == pdTRUE)
        {
            // Prepare window array and apply on data_sampled. Results are stored to fft_components
            fft_apply_window_on_fft_complex(data_sampled, window, fft_components);
            ESP_LOGI(TAG, "Applied window");

            // Run the FFT calculations
            fft_calculate_re_im(fft_components, N_SAMPLES * 2);
            ESP_LOGI(TAG, "Calculated fft");

            // Calculate magnitudes
            fft_calculate_magnitudes(magnitudes_indexed, N_SAMPLES);
            ESP_LOGI(TAG, "Calculated magnitudes");

            // Signal UART task
            xSemaphoreGive(xSemaphoreFFTReady);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}

void task_uart_fft_components(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xSemaphoreFFTReady, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI(TAG, "Sending data over UART");
            fft_send_percentiles_over_uart(95.0, N_SAMPLES);
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Start Example.");

    // Init I2C settings
    i2c_init();

    // Setup the MPU6050 registers
    mpu_initial_setup(&i2c_buffer);

    // // Init custom UART settings (commented out to check FFT outputs)
    // uart_init();
    // // Initialize FFT
    // fft_init();


    xSemaphoreMPUDataReady = xSemaphoreCreateBinary();
    xSemaphoreFFTReady = xSemaphoreCreateBinary();

    if (xSemaphoreMPUDataReady == NULL || xSemaphoreFFTReady == NULL)
    {
        ESP_LOGE(TAG, "Failed to create semaphores");
        return;
    }

    // Create tasks
    xTaskCreate(task_mpu_data_reading, "Sensor Task", SENSOR_TASK_STACK_SIZE, NULL, 1, NULL);
    // xTaskCreate(task_fft_calculations, "FFT Task", FFT_TASK_STACK_SIZE, NULL, 2, NULL);
    // xTaskCreate(task_uart_fft_components, "UART Task", UART_TASK_STACK_SIZE, NULL, 1, NULL);

    vTaskDelete(NULL);

    ESP_ERROR_CHECK(i2c_del_master_bus(master_bus_handle));

}
