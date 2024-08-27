#include "program_tasks.h"

TaskHandle_t notif_init;
TaskHandle_t notif_data_sampling;
TaskHandle_t notif_fft_calculation;
TaskHandle_t notif_send_fft_components;
TaskHandle_t notif_send_data_samples;

float *data_sampled;
indexed_float_type *indexed_magnitudes;
float *fft_complex_arr;

void task_initialization(void *params)
{
	const char *TAG = "TSK INIT";
	// esp_log_level_set(TAG, ESP_LOG_INFO);

	int error_code = 0;

	// Allocate memory for data_sampled in DRAM with 8-bit and 32-bit alignment
	data_sampled = (float *)heap_caps_malloc(N_SAMPLES * sizeof(float), MALLOC_CAP_SPIRAM);
	if (!data_sampled)
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

	// Create mpu sampling and fft tasks
	if (xTaskCreatePinnedToCore(task_mpu6050_data_sampling, "Data sampling task", TASK_MPU_SAMPLING_STACK_SIZE, NULL, 10, &notif_data_sampling, tskNO_AFFINITY) != pdPASS)
	{
		ESP_LOGE(TAG, "Failed to create mpu data sampling task");
		vTaskDelete(NULL);
	}
    if (xTaskCreate(task_fft_calculation, "FFT calculation task", TASK_FFT_CALC_STACK_SIZE, NULL, 10, &notif_fft_calculation) != pdPASS)
	{
		ESP_LOGE(TAG, "Failed to create fft calculation task");
		vTaskDelete(NULL);
	}
	if (xTaskCreate(task_fft_send_components, "Send FFT components task", TASK_SEND_FFT_STACK_SIZE, NULL, 10, &notif_send_fft_components) != pdPASS)
	{
		ESP_LOGE(TAG, "Failed to create fft components uart transmission task");
		vTaskDelete(NULL);

	}
    if (xTaskCreate(task_send_data_samples, "Send data samples task", TASK_SEND_DATA_SAMPLES_STACK_SIZE, NULL, 10, &notif_send_data_samples) != pdPASS)
	{
		ESP_LOGE(TAG, "Failed to create fft components uart transmission task");
		vTaskDelete(NULL);

	}

	// Init UART with ISR queue
	if ((error_code = uart_init_with_isr_queue(&uart_config, UART_NUM, UART_TXD, UART_RXD, UART_TX_BUFF_SIZE, UART_RX_BUFF_SIZE, &uart_event_queue_handle, UART_EVENT_QUEUE_SIZE, 0)) != 0)
	{
		ESP_LOGE(TAG, "Failed to init uart with isr queue, error code %d", error_code);
		vTaskDelete(NULL);
	}
	// Create UART ISR monitoring and message handling tasks
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
	esp_log_level_set(TAG, ESP_LOG_ERROR);
	UBaseType_t old_free_heap = 0;

	size_t index = 0;
	TickType_t last_wake_time;
	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		last_wake_time = xTaskGetTickCount();
		// while (index < ((uint32_t)N_SAMPLES/1000)*1000)
		while (index < N_SAMPLES)
		{
			if (!mpu_data_read_extract_accel(&i2c_buffer_t, &mpu_data_t))
			{
				ESP_LOGE(TAG, "Error reading MPU6050 data.");
				continue;
			}
			mpu_data_substract_err(&mpu_data_t, true);
			mpu_data_to_fs(&mpu_data_t, true);

			vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1)); // Sampling frequency 1 ms
			// copy value to the array
			memcpy(&data_sampled[index], &mpu_data_t.accel_gyro_g[0], sizeof(float)); // X-axis
			last_wake_time = xTaskGetTickCount();
			index++;
		}
		index = 0;

		if (DEBUG_STACKS == 1)
		{
			UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
			ESP_LOGI(TAG, "Free stack size: %u B", stack_hwm);
			ESP_LOGI(TAG, "Stack in use: %u of %u B", (TASK_MPU_SAMPLING_STACK_SIZE - stack_hwm), TASK_MPU_SAMPLING_STACK_SIZE);

			ESP_LOGI(TAG, "Sampling complete");

			UBaseType_t free_heap = xPortGetFreeHeapSize();
			ESP_LOGI(TAG, "Free heap size: %u B (old %u)", free_heap, old_free_heap);
			old_free_heap = free_heap;
		}

		xTaskNotifyGive(notif_fft_calculation);
	}
}

void task_fft_calculation(void *params)
{
	const char *TAG = "TSK FFT CALC";
	esp_log_level_set(TAG, ESP_LOG_ERROR);

	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// Copy sampled data to the complex array. Re parts only, im all to zero.
		fft_prepare_complex_arr(data_sampled, fft_complex_arr, N_SAMPLES);
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

		// ESP_LOGI(TAG, "FFT calculations complete");
		xTaskNotifyGive(notif_send_fft_components);
	}
}

void task_fft_send_components(void *params)
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
		xTaskNotifyGive(notif_send_data_samples);
	}
}

void task_send_data_samples(void *params)
{
	const char *TAG = "TSK SEND DATA";
	esp_log_level_set(TAG, ESP_LOG_ERROR);

	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		uart_write_bytes(UART_NUM, "\n", 1);
		uart_write_bytes(UART_NUM, "\xfe\xfe\xfe\xfe\xff", 5);
		printf("\ndata*");
		for (size_t i = 0; i < N_SAMPLES; i++)
		{
			// printf("%.4f, ", data_sampled[i]);
			uart_write_bytes(UART_NUM, (const char *)&data_sampled[i], sizeof(float));
		}
		printf("*data\n");
		uart_write_bytes(UART_NUM, "\xff\xfe\xfe\xfe\xfe", 5);

		if (DEBUG_STACKS == 1)
		{
			UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
			ESP_LOGI(TAG, "Free stack size: %u B", stack_hwm);
			ESP_LOGI(TAG, "Stack in use: %u of %u B", (TASK_SEND_DATA_SAMPLES_STACK_SIZE - stack_hwm), TASK_SEND_DATA_SAMPLES_STACK_SIZE);
			ESP_LOGI(TAG, "Data samples sent");
		}
	}
}