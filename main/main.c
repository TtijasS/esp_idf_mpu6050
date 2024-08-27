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
#include "my_fft.h"
#include "program_tasks.h"
#include "uart_isr_handler.h"

void app_main(void)
{
    const char *TAG = "MAIN APP";
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
	queue_msg_handle = xQueueCreate(4, sizeof(TaskQueueMessage_type));

    if (xTaskCreatePinnedToCore(task_initialization, "Init task", TASK_INIT_STACK_SIZE, NULL, 10, NULL, tskNO_AFFINITY) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create init task");
    }

    // UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
    // ESP_LOGI(TAG, "Free stack size: %u B", stack_hwm);
    
}
