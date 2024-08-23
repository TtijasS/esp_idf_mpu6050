#ifndef PROGRAM_TASKS_H
#define PROGRAM_TASKS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "constants.h"
#include "mpu6050.h"
#include "my_i2c_com.h"
#include "data_structs.h"
#include "my_uart_com.h"
#include "my_fft.h"
// #include "uart_isr_handler.h"

extern TaskHandle_t notif_init;
extern TaskHandle_t notif_data_sampling;
extern TaskHandle_t notif_fft_calculation;
extern TaskHandle_t notif_send_fft_components;
extern TaskHandle_t notif_send_data_samples;

extern float data_sampled_x[N_SAMPLES];

void task_initialization(void *params);
void task_mpu6050_data_sampling(void *params);
void task_fft_calculation(void *params);
void task_fft_send_components(void *params);
void task_send_data_samples(void *params);

#endif // PROGRAM_TASKS_H