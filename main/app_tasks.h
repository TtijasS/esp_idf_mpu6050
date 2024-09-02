#ifndef PROGRAM_TASKS_H
#define PROGRAM_TASKS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "constants.h"
#include "mpu6050.h"
#include "my_i2c_com.h"
#include "data_structs.h"
#include "my_fft.h"
#include "uart_isr_handler.h"

// Task handles
extern TaskHandle_t handl_mpu_sampling_begin;
extern TaskHandle_t handl_fft_calculation;
extern TaskHandle_t handl_uart_data_samples;
extern TaskHandle_t handl_uart_fft_components;

// Semaphores
extern SemaphoreHandle_t semphr_sampling_request_a;
extern SemaphoreHandle_t semphr_sampling_request_b;
extern SemaphoreHandle_t semphr_uart_request;

// Queueu handles for UART ISR events
extern QueueHandle_t queue_uart_event_queue;
extern QueueHandle_t queue_enqueued_msg_processing;
extern QueueHandle_t queue_fft_calculation;


// Structs
typedef struct FFTQueueMessage_type
{
	bool array_number;
	float *array_ptr;
	
}FFTQueueMessage_type;

typedef struct TaskQueueMessage_type
{
	size_t msg_size;
	uint8_t *msg_ptr;
	
}TaskQueueMessage_type;


extern float *data_samples_a;
extern float *data_samples_b;
extern indexed_float_type *indexed_magnitudes;
extern float *fft_complex_arr;


void task_initialization(void *params);
void task_mpu6050_data_sampling(void *params);
void task_fft_calculation(void *params);
void task_uart_fft_components(void *params);
void task_uart_data_samples(void *params);

// UART ISR MONITORING
void task_uart_isr_monitoring(void *);
void task_queue_msg_handler(void *);

#endif // PROGRAM_TASKS_H