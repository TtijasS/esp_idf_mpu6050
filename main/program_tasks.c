#include "program_tasks.h"

TaskHandle_t notif_init;
TaskHandle_t notif_data_sampling;
TaskHandle_t notif_fft_calculation;
TaskHandle_t notif_send_fft_components;
TaskHandle_t notif_send_data_samples;

float data_sampled_x[N_SAMPLES];
__attribute__((aligned(16))) float fft_complex_arr[N_SAMPLES * 2];					 // Real and imaginary part of sampled data [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
__attribute__((aligned(16))) indexed_float_type indexed_magnitudes[MAGNITUDES_SIZE]; // FFT output for magnitudes

void task_initialization(void *params)
{
	const char *TAG = "TSK INIT FN";
	esp_log_level_set(TAG, ESP_LOG_INFO);

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	// Init I2C and UART
	i2c_init();
	// Configure MPU6050
	mpu_initial_setup(&i2c_buffer_t);
	// Configure UART
	uart_init();
	// Init FFT memory
	fft_init();

	ESP_LOGI(TAG, "Init complete");

	xTaskNotifyGive(notif_data_sampling);
	// UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
	// ESP_LOGI(TAG, "Free stack size: %u B", stack_hwm);
	// ESP_LOGI(TAG, "Stack in use: %u of %u B", (TASK_INIT_STACK_SIZE - stack_hwm), TASK_INIT_STACK_SIZE);
	vTaskDelete(NULL);
}

void task_mpu6050_data_sampling(void *params)
{
	const char *TAG = "TSK DATA SAMPL";
	esp_log_level_set(TAG, ESP_LOG_INFO);
	UBaseType_t old_free_heap = 0;

	size_t index = 0;
	TickType_t last_wake_time;
	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		last_wake_time = xTaskGetTickCount();
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
			memcpy(&data_sampled_x[index], &mpu_data_t.accel_gyro_g[0], sizeof(float)); // X-axis
			// memcpy(&data_sampled_y[index], &mpu_data_t.accel_gyro_g[1], sizeof(float)); // Y-axis
			// memcpy(&data_sampled_z[index], &mpu_data_t.accel_gyro_g[2], sizeof(float)); // Z-axis
			last_wake_time = xTaskGetTickCount();
			index++;
		}
		index = 0;
		// UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
		// ESP_LOGI(TAG, "Free stack size: %u B", stack_hwm);
		// ESP_LOGI(TAG, "Stack in use: %u of %u B", (TASK_MPU_SAMPLING_STACK_SIZE - stack_hwm), TASK_MPU_SAMPLING_STACK_SIZE);

		// ESP_LOGI(TAG, "Sampling complete");

		// UBaseType_t free_heap = xPortGetFreeHeapSize();
		// ESP_LOGI(TAG, "Free heap size: %u B (old %u)", free_heap, old_free_heap);
		// old_free_heap = free_heap;
		xTaskNotifyGive(notif_fft_calculation);
	}
}

void task_fft_calculation(void *params)
{
	const char *TAG = "TSK FFT CALC";
	esp_log_level_set(TAG, ESP_LOG_INFO);
	
	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// Copy sampled data to the complex array. Re parts only, im all to zero.
		fft_prepare_complex_arr(data_sampled_x, fft_complex_arr, N_SAMPLES);
		// ESP_LOGI(TAG, "Window prepared and data merged to fft_components");

		fft_calculate_re_im(fft_complex_arr, N_SAMPLES);
		// // ESP_LOGI(TAG, "FFT calculated");

		fft_calculate_magnitudes(indexed_magnitudes, fft_complex_arr, MAGNITUDES_SIZE);

		// fft_sort_magnitudes(indexed_magnitudes, MAGNITUDES_SIZE);

		// UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
		// ESP_LOGI(TAG, "Free stack size: %u B", stack_hwm);
		// ESP_LOGI(TAG, "Stack in use: %u of %u B", (TASK_FFT_CALC_STACK_SIZE - stack_hwm), TASK_FFT_CALC_STACK_SIZE);

		// ESP_LOGI(TAG, "FFT calculations complete");
		xTaskNotifyGive(notif_send_fft_components);
	}
}

void task_fft_send_components(void *params)
{
	const char *TAG = "T FFT SEND COMP";
	esp_log_level_set(TAG, ESP_LOG_INFO);

	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		uint32_t n_ms_components = fft_percentile_n_components(0, N_SAMPLES);

		int error_code = fft_send_ms_components_over_uart(fft_complex_arr, indexed_magnitudes, N_SAMPLES, n_ms_components);
		if (error_code != 0)
			ESP_LOGE(TAG, "Error %d", error_code);

		// UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
		// ESP_LOGI(TAG, "Free stack size: %u B", stack_hwm);
		// ESP_LOGI(TAG, "Stack in use: %u of %u B", (TASK_SEND_FFT_STACK_SIZE - stack_hwm), TASK_SEND_FFT_STACK_SIZE);

		// ESP_LOGI(TAG, "FFT components sent");
		xTaskNotifyGive(notif_send_data_samples);
	}
}

void task_send_data_samples(void *params)
{
	const char *TAG = "TSK SEND DATA";
	esp_log_level_set(TAG, ESP_LOG_INFO);

	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		uart_write_bytes(uart_num, "\n", 1);
		uart_write_bytes(uart_num, "\xfe\xfe\xfe\xfe\xff", 5);
		// printf("\ndata*");
		for (size_t i = 0; i < N_SAMPLES; i++)
		{
			// printf("%.4f, ", data_sampled_x[i]);
			uart_write_bytes(uart_num, (const char *)&data_sampled_x[i], sizeof(float));
		}
		// printf("*data\n");
		uart_write_bytes(uart_num, "\xff\xfe\xfe\xfe\xfe", 5);

		// UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
		// ESP_LOGI(TAG, "Free stack size: %u B", stack_hwm);
		// ESP_LOGI(TAG, "Stack in use: %u of %u B", (TASK_SEND_DATA_SAMPLES_STACK_SIZE - stack_hwm), TASK_SEND_DATA_SAMPLES_STACK_SIZE);
		// ESP_LOGI(TAG, "Data samples sent");
		// xTaskNotifyGive(notif_data_sampling);
	}
}