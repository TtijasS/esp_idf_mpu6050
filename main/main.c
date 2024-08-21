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
#include "my_uart_com.h"
#include "my_fft.h"
#include "program_tasks.h"

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    xTaskCreatePinnedToCore(task_initialization, "Init task", TASK_INIT_STACK_SIZE, NULL, 10, &notif_init, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(task_mpu6050_data_sampling, "Data sampling task", TASK_MPU_SAMPLING_STACK_SIZE, NULL, 10, &notif_data_sampling, tskNO_AFFINITY);
    xTaskCreate(task_fft_calculation, "FFT calculation task", TASK_FFT_CALC_STACK_SIZE, NULL, 10, &notif_fft_calculation);
    xTaskCreate(task_fft_send_components, "Send FFT components task", TASK_SEND_FFT_STACK_SIZE, NULL, 10, &notif_send_fft_components);
    xTaskCreate(task_send_data_samples, "Send data samples task", TASK_SEND_DATA_SAMPLES_STACK_SIZE, NULL, 10, &notif_send_data_samples);

    UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Free stack size: %u B", stack_hwm);
    xTaskNotifyGive(notif_init); // Start the init task after all the tasks get created and are put into suspended mode
    
}
