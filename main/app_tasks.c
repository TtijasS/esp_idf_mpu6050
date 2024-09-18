#include "app_tasks.h"

TaskHandle_t handl_mpu_sampling_begin;
TaskHandle_t handl_fft_calculation;
TaskHandle_t handl_uart_fft_components;
// TaskHandle_t handl_uart_data_samples;

SemaphoreHandle_t semphr_sampling_request_a;
SemaphoreHandle_t semphr_sampling_request_b;
SemaphoreHandle_t semphr_uart_request;

QueueHandle_t queue_uart_event_queue;
QueueHandle_t queue_enqueued_msg_processing;
QueueHandle_t queue_fft_calculation;

bool data_ready_a = false;
bool data_ready_b = false;
bool fft_ready_a = false;
bool fft_ready_b = false;
bool sampling_a = false;
bool sampling_b = false;

float *data_samples_a;
float *data_samples_b;
indexed_float_type *indexed_magnitudes;
float *fft_complex_arr;

void task_initialization(void *params)
{
	const char *TAG = "TSK INIT";
	// esp_log_level_set(TAG, ESP_LOG_INFO);

	int error_code = 0;

	// // Allocate memory for data_sampled in DRAM with 8-bit and 32-bit alignment
	// data_sampled = (float *)heap_caps_malloc(N_SAMPLES * sizeof(float), MALLOC_CAP_SPIRAM);
	// if (!data_sampled)
	// {
	// 	ESP_LOGE(TAG, "Failed to allocate memory for data_sampled");
	// 	vTaskDelete(NULL);
	// }

	data_samples_a = (float *)heap_caps_malloc(N_SAMPLES * sizeof(float), MALLOC_CAP_SPIRAM);
	data_samples_b = (float *)heap_caps_malloc(N_SAMPLES * sizeof(float), MALLOC_CAP_SPIRAM);
	if (data_samples_a == NULL || data_samples_b == NULL)
	{
		ESP_LOGE(TAG, "Failed to allocate memory for data_sampled");
		vTaskDelete(NULL);
	}

	// Allocate memory for indexed_magnitudes in DRAM with 8-bit alignment
	indexed_magnitudes = (indexed_float_type *)heap_caps_malloc(MAGNITUDES_SIZE * sizeof(indexed_float_type), MALLOC_CAP_SPIRAM);
	if (!indexed_magnitudes)
	{
		ESP_LOGE(TAG, "Failed to allocate memory for indexed_magnitudes");
		vTaskDelete(NULL);
	}

	// Allocate aligned memory for fft_complex_arr in PSRAM with 16-byte alignment
	fft_complex_arr = (float *)heap_caps_aligned_alloc(16, N_SAMPLES * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
	// fft_complex_arr = (float*)heap_caps_aligned_alloc(16, N_SAMPLES * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
	if (!fft_complex_arr)
	{
		ESP_LOGE(TAG, "Failed to allocate memory for fft_complex_arr");
		vTaskDelete(NULL);
	}
	// Init I2C and UART
	if ((error_code = i2c_init()) != 0)
	{
		ESP_LOGE(TAG, "Failed to init i2c with error code %d", error_code);
		vTaskDelete(NULL);
	}
	// Configure MPU6050
	if ((error_code = mpu_initial_setup(&i2c_buffer_t)) != 0)
	{
		ESP_LOGE(TAG, "Failed to init i2c with error code %d", error_code);
		vTaskDelete(NULL);
	}
	// Init FFT memory
	if ((error_code = fft_init()) != 0)
	{
		ESP_LOGE(TAG, "Failed to init fft with error code %d", error_code);
		vTaskDelete(NULL);
	}

	// Create semaphore binaries
	semphr_sampling_request_a = xSemaphoreCreateBinary();
	semphr_sampling_request_b = xSemaphoreCreateBinary();
	semphr_uart_request = xSemaphoreCreateBinary();

	// Create queues
	queue_enqueued_msg_processing = xQueueCreate(4, sizeof(TaskQueueMessage_type));
	queue_fft_calculation = xQueueCreate(2, sizeof(FFTQueueMessage_type));

	// Enable data requests
	xSemaphoreGive(semphr_sampling_request_a);
	xSemaphoreGive(semphr_sampling_request_b);

	// Create FFT tasks
	if (xTaskCreatePinnedToCore(task_mpu6050_data_sampling, "MPU Data sampling task", TASK_MPU_SAMPLING_STACK_SIZE, NULL, 15, &handl_mpu_sampling_begin, APP_CPU_NUM) != pdPASS)
	{
		ESP_LOGE(TAG, "Failed to create mpu data sampling task");
		vTaskDelete(NULL);
	}
	if (xTaskCreate(task_fft_calculation, "FFT calculation task", TASK_FFT_CALC_STACK_SIZE, NULL, 10, &handl_fft_calculation) != pdPASS)
	{
		ESP_LOGE(TAG, "Failed to create fft calculation task");
		vTaskDelete(NULL);
	}
	if (xTaskCreate(task_uart_fft_components, "Send FFT components task", TASK_SEND_FFT_STACK_SIZE, NULL, 10, &handl_uart_fft_components) != pdPASS)
	{
		ESP_LOGE(TAG, "Failed to create fft components uart transmission task");
		vTaskDelete(NULL);
	}
	// if (xTaskCreate(task_uart_data_samples, "Send data samples task", TASK_SEND_DATA_SAMPLES_STACK_SIZE, NULL, 10, &handl_uart_data_samples) != pdPASS)
	// {
	// 	ESP_LOGE(TAG, "Failed to create fft components uart transmission task");
	// 	vTaskDelete(NULL);
	// }

	// Init UART with ISR queue
	if ((error_code = myuart_init_with_isr_queue(&uart_config, UART_NUM, UART_TXD, UART_RXD, UART_TX_BUFF_SIZE, UART_RX_BUFF_SIZE, &queue_uart_event_queue, UART_EVENT_QUEUE_SIZE, 0)) != 0)
	{
		ESP_LOGE(TAG, "Failed to init uart with isr queue, error code %d", error_code);
		vTaskDelete(NULL);
	}
	// Create UART ISR tasks
	if (xTaskCreatePinnedToCore(&task_uart_isr_monitoring, "UART ISR monitoring task", TASK_ISRUART_STACK_SIZE, NULL, 18, NULL, 0) != pdPASS)
	{
		ESP_LOGE(TAG, "Failed to create uart isr monitoring task");
		vTaskDelete(NULL);
	}
	if (xTaskCreatePinnedToCore(&task_queue_msg_handler, "Receive queue msg task", TASK_MSG_Q_STACK_SIZE, NULL, 10, NULL, 1) != pdPASS)
	{
		ESP_LOGE(TAG, "Failed to create receive queue msg task");
		vTaskDelete(NULL);
	}
	// Enable pattern detection and reset the pattern queue
	if ((error_code = uart_enable_pattern_det_baud_intr(UART_NUM, '+', UART_PATTERN_SIZE, 8, 0, 0)) != 0)
	{
		ESP_LOGE(TAG, "Failed to enable pattern detection baud interrupt");
		vTaskDelete(NULL);
	}
	if ((error_code = uart_pattern_queue_reset(UART_NUM, UART_PAT_QUEUE_SIZE)) != 0)
	{
		ESP_LOGE(TAG, "Failed to reset uart pattern queue");
		vTaskDelete(NULL);
	}

	if (DEBUG_STACKS == 1)
	{
		UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
		ESP_LOGI(TAG, "Free stack size: %u B", stack_hwm);
		ESP_LOGI(TAG, "Stack in use: %u of %u B", (TASK_INIT_STACK_SIZE - stack_hwm), TASK_INIT_STACK_SIZE);
	}

	vTaskDelete(NULL);
}

void task_mpu6050_data_sampling(void *params)
{
	const char *TAG = "TSK DATA SAMPL";
	const char *MSG_A_RDY = "A DATRDY";	 // Data samples A ready
	const char *MSG_B_RDY = "B DATRDY";	 // Data samples B ready
	const char *MPU_ERR_MSG = "MPU ERR"; // MPU reading data error
	esp_log_level_set(TAG, ESP_LOG_ERROR);
	UBaseType_t old_free_heap = 0;

	size_t index_a = 0;
	size_t index_b = 0;
	TickType_t last_wake_time;
	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		last_wake_time = xTaskGetTickCount();
		while (sampling_a || sampling_b)
		{
			if (!mpu_data_read_extract_accel(&i2c_buffer_t, &mpu_data_t))
			{
				ESP_LOGE(TAG, "Error reading MPU6050 data.");
				uart_write_bytes(UART_NUM, MPU_ERR_MSG, strlen(MPU_ERR_MSG));
				sampling_a = false;
				sampling_b = false;
				continue;
			}
			mpu_data_substract_err(&mpu_data_t, true);
			mpu_data_to_fs(&mpu_data_t, true);

			// copy value to the data_samples arrays
			if (sampling_a)
			{
				// Update array A
				if (index_a < N_SAMPLES)
				{
					memcpy(&data_samples_a[index_a], &mpu_data_t.accel_gyro_g[0], sizeof(float)); // X-axis
					index_a++;
				}
				// Raise data A ready flag and stop updating A
				else
				{
					uart_write_bytes(UART_NUM, MSG_A_RDY, strlen(MSG_A_RDY));
					index_a = 0;
					sampling_a = false;
					fft_ready_a = false;
					data_ready_a = true;
					xSemaphoreGive(semphr_sampling_request_a);
					xSemaphoreGive(semphr_uart_request);
				}
			}
			if (sampling_b)
			{
				// Update array B
				if (index_b < N_SAMPLES)
				{
					memcpy(&data_samples_b[index_b], &mpu_data_t.accel_gyro_g[0], sizeof(float)); // X-axis
					index_b++;
				}
				// Raise data B ready flag and stop updating B
				else
				{
					uart_write_bytes(UART_NUM, MSG_B_RDY, strlen(MSG_B_RDY));
					index_b = 0;
					sampling_b = false;
					fft_ready_b = false;
					data_ready_b = true;
					xSemaphoreGive(semphr_sampling_request_b);
				}
			}
			// Wait until 1 ms has passed (1 ms sampling frequency)
			vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
			last_wake_time = xTaskGetTickCount();
		}
	}
}

void task_fft_calculation(void *params)
{
	const char *TAG = "TSK FFT CALC";
	esp_log_level_set(TAG, ESP_LOG_ERROR);
	const char *MSG_A_RDY = "A FFTRDY"; // FFT data A ready
	const char *MSG_B_RDY = "B FFTRDY"; // FFT data B ready
	FFTQueueMessage_type data_in_queue;

	while (1)
	{
		if (xQueueReceive(queue_fft_calculation, &data_in_queue, portMAX_DELAY))
		{
			if (data_in_queue.array_ptr == NULL)
				continue;
			// Copy sampled data to the complex array. Re parts only, im all to zero.
			fft_prepare_complex_arr(data_in_queue.array_ptr, fft_complex_arr, N_SAMPLES);
			// ESP_LOGI(TAG, "Window prepared and data merged to fft_components");

			fft_calculate_re_im(fft_complex_arr, N_SAMPLES);
			// // ESP_LOGI(TAG, "FFT calculated");

			fft_calculate_magnitudes(indexed_magnitudes, fft_complex_arr, MAGNITUDES_SIZE);

			fft_sort_magnitudes(indexed_magnitudes, MAGNITUDES_SIZE);

			if (DEBUG_STACKS == 1)
			{
				UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
				ESP_LOGI(TAG, "Free stack size: %u B", stack_hwm);
				ESP_LOGI(TAG, "Stack in use: %u of %u B", (TASK_FFT_CALC_STACK_SIZE - stack_hwm), TASK_FFT_CALC_STACK_SIZE);
			}

			if (data_in_queue.array_number == 0)
			{
				fft_ready_a = true;
				fft_ready_b = false;
				uart_write_bytes(UART_NUM, MSG_A_RDY, strlen(MSG_A_RDY));
			}
			else
			{
				fft_ready_a = false;
				fft_ready_b = true;
				uart_write_bytes(UART_NUM, MSG_B_RDY, strlen(MSG_B_RDY));
			}
			xTaskNotifyGive(handl_uart_fft_components);
		}
	}
}

void task_uart_fft_components(void *params)
{
	const char *TAG = "T FFT SEND COMP";
	esp_log_level_set(TAG, ESP_LOG_ERROR);

	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		uint32_t n_ms_components = fft_percentile_n_components(99, MAGNITUDES_SIZE);

		int error_code = fft_send_ms_components_over_uart(fft_complex_arr, indexed_magnitudes, N_SAMPLES, n_ms_components);
		if (error_code != 0)
			ESP_LOGE(TAG, "Error %d", error_code);

		if (DEBUG_STACKS == 1)
		{
			UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
			ESP_LOGI(TAG, "Free stack size: %u B", stack_hwm);
			ESP_LOGI(TAG, "Stack in use: %u of %u B", (TASK_SEND_FFT_STACK_SIZE - stack_hwm), TASK_SEND_FFT_STACK_SIZE);
		}

		// ESP_LOGI(TAG, "FFT components sent");
		// xTaskNotifyGive(handl_uart_data_samples);
		xSemaphoreGive(semphr_uart_request);
	}
}

void task_uart_data_samples(void *params)
{
	const char *TAG = "TSK SEND DATA";
	esp_log_level_set(TAG, ESP_LOG_ERROR);

	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		uart_write_bytes(UART_NUM, "\xfe\xfe\xfe\xfe\xff", 5);
		if (data_ready_a && fft_ready_a)
		{
			for (size_t i = 0; i < N_SAMPLES; i++)
			{
				uart_write_bytes(UART_NUM, (const char *)&data_samples_a[i], sizeof(float));
			}
		}
		else if (data_ready_b && fft_ready_b)
		{
			for (size_t i = 0; i < N_SAMPLES; i++)
			{
				uart_write_bytes(UART_NUM, (const char *)&data_samples_b[i], sizeof(float));
			}
		}
		uart_write_bytes(UART_NUM, "\xff\xfe\xfe\xfe\xfe", 5);

		// if (DEBUG_STACKS == 1)
		// {
		// 	UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
		// 	ESP_LOGI(TAG, "Free stack size: %u B", stack_hwm);
		// 	ESP_LOGI(TAG, "Stack in use: %u of %u B", (TASK_SEND_DATA_SAMPLES_STACK_SIZE - stack_hwm), TASK_SEND_DATA_SAMPLES_STACK_SIZE);
		// 	ESP_LOGI(TAG, "Data samples sent");
		// }
		xSemaphoreGive(semphr_uart_request);
	}
}

void task_uart_isr_monitoring(void *params)
{
	const char *TAG = "UART ISR TASK";

	uart_event_t uart_event;
	int error_flag = 0;
	int encapsulation_counter = 0;

	while (1)
	{
		// Receive the entire uart_event_t structure from the queue
		if (xQueueReceive(queue_uart_event_queue, (void *)&uart_event, portMAX_DELAY) == pdTRUE)
		{
			switch (uart_event.type)
			{
			/**
			 * @brief Pattern detection case
			 *
			 * Multiple identical characters of specific type detected in a row.
			 * Number of chars is defined with macro UART_PATTERN_SIZE
			 */
			case UART_PATTERN_DET:
				int pattern_index = uart_pattern_pop_pos(UART_NUM);
				if (pattern_index == -1)
				{
					ESP_LOGE(TAG, "Pattern index -1");
					break;
				}
				if ((error_flag = myuart_encapsulation_handler(UART_NUM, &encapsulation_counter, &pattern_index)) < 0)
				{
					ESP_LOGE(TAG, "Uart encapsulation handler error %d", error_flag);
				}
				break;

			default:
				break;
			}
		}
	}
	ESP_LOGW(TAG, "KILLING THE UART ISR TASK");
	vTaskDelete(NULL);
}

void task_queue_msg_handler(void *params)
{
	const char *TAG = "TSK QUEUE MSG HANDL";
	FFTQueueMessage_type fft_queue_msg_a = {
		.array_number = 0,
		.array_ptr = data_samples_a};

	FFTQueueMessage_type fft_queue_msg_b = {
		.array_number = 1,
		.array_ptr = data_samples_b};

	TaskQueueMessage_type enqueued_message;

	// START SAMPLING
	const char *A_START = "A START";
	const char *B_START = "B START";
	// STARTED SAMPLING CONFIRMATION
	const char *A_SAMPLING = "A SAMPLING";
	const char *B_SAMPLING = "B SAMPLING";
	// SEND DATA
	const char *A_SEND = "A SEND";
	const char *B_SEND = "B SEND";
	// SAMPLING BUSY
	const char *A_BUSY = "A BUSY";
	const char *B_BUSY = "B BUSY";
	// CONFIRMING DATA SEND
	const char *A_OK = "A OK";
	const char *B_OK = "B OK";
	// DATA NOT READY
	const char *A_NOTRDY = "A NOTRDY";
	const char *B_NOTRDY = "B NOTRDY";
	// COMMON
	const char *FFT = "FFT";
	const char *FAIL = "FAIL";
	const char *WHOAMI = "WHOAMI";
	const char *DEVID = "MPU6050";
	// When data is ready sampling task sends A DATRDY or B DATRDY
	// When FFT is done calculating, fft task sends A FFTOK or B FFTOK

	while (1)
	{
		if (xQueueReceive(queue_enqueued_msg_processing, &enqueued_message, portMAX_DELAY))
		{
			if (enqueued_message.msg_ptr != NULL)
			{
				// WHOAMI
				if (memcmp(enqueued_message.msg_ptr, WHOAMI, (strlen(WHOAMI))) == 0)
				{
					uart_write_bytes(UART_NUM, DEVID, strlen(DEVID));
				}
				// A START
				else if (memcmp(enqueued_message.msg_ptr, A_START, (strlen(A_START))) == 0)
				{
					if (xSemaphoreTake(semphr_sampling_request_a, pdMS_TO_TICKS(10)) == pdTRUE)
					{
						uart_write_bytes(UART_NUM, A_SAMPLING, strlen(A_SAMPLING));
						data_ready_a = false;
						fft_ready_a = false;
						if (!sampling_a && !sampling_b)
						{
							sampling_a = true;
							xTaskNotifyGive(handl_mpu_sampling_begin);
						}
						else
						{
							sampling_a = true;
						}
					}
					else if (!sampling_a && !data_ready_a)
					{
						xSemaphoreGive(semphr_sampling_request_a);
					}
					else
					{
						uart_write_bytes(UART_NUM, A_BUSY, strlen(A_BUSY));
					}
				}
				// B START
				else if (memcmp(enqueued_message.msg_ptr, B_START, (strlen(B_START))) == 0)
				{
					if (xSemaphoreTake(semphr_sampling_request_b, pdMS_TO_TICKS(10)) == pdTRUE)
					{
						uart_write_bytes(UART_NUM, B_SAMPLING, strlen(B_SAMPLING));
						data_ready_b = false;
						fft_ready_b = false;
						if (!sampling_a && !sampling_b)
						{
							sampling_b = true;
							xTaskNotifyGive(handl_mpu_sampling_begin);
						}
						else
							sampling_b = true;
					}
					else if (!sampling_b && !data_ready_b)
					{
						xSemaphoreGive(semphr_sampling_request_b);
					}
					else
					{
						uart_write_bytes(UART_NUM, B_BUSY, strlen(B_BUSY));
					}
				}
				// A SEND
				else if (memcmp(enqueued_message.msg_ptr, A_SEND, (strlen(A_SEND))) == 0)
				{
					if (data_ready_a)
					{
						
						if (fft_ready_a)
						{
							uart_write_bytes(UART_NUM, A_OK, strlen(A_OK));
							xTaskNotifyGive(handl_uart_fft_components);
						}
						else if (xQueueSend(queue_fft_calculation, &fft_queue_msg_a, 0) == pdTRUE)
						{
							uart_write_bytes(UART_NUM, A_OK, strlen(A_OK));
							uart_write_bytes(UART_NUM, FFT, strlen(FFT));
						}
						else
						{
							uart_write_bytes(UART_NUM, FFT, strlen(FFT));
							uart_write_bytes(UART_NUM, FAIL, strlen(FAIL));
						}
					}
					else
					{
						uart_write_bytes(UART_NUM, A_NOTRDY, strlen(A_NOTRDY));
					}
				}
				// B SEND
				else if (memcmp(enqueued_message.msg_ptr, B_SEND, (strlen(B_SEND))) == 0)
				{
					if (data_ready_b)
					{
						if (fft_ready_b)
						{
							uart_write_bytes(UART_NUM, B_OK, strlen(B_OK));
							xTaskNotifyGive(handl_uart_fft_components);
						}
						else if (xQueueSend(queue_fft_calculation, &fft_queue_msg_b, 0) == pdTRUE)
						{
							uart_write_bytes(UART_NUM, B_OK, strlen(B_OK));
							uart_write_bytes(UART_NUM, FFT, strlen(FFT));
						}
						else
						{
							uart_write_bytes(UART_NUM, FFT, strlen(FFT));
							uart_write_bytes(UART_NUM, FAIL, strlen(FAIL));
						}
					}
					else
					{
						uart_write_bytes(UART_NUM, B_NOTRDY, strlen(B_NOTRDY));
					}
				}
				else
				{
					uart_write_bytes(UART_NUM, "?>", strlen("?>"));
					uart_write_bytes(UART_NUM, enqueued_message.msg_ptr, enqueued_message.msg_size);
				}
				free(enqueued_message.msg_ptr);
			}
			else
			{
				ESP_LOGE(TAG, "Null pointer passed");
			}
		}
	}
}