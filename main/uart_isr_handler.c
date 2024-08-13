#include "uart_isr_handler.h"

QueueHandle_t uart_event_queue_handle;
int tmp_message_size = 0;
uint8_t *tmp_message_buffer = NULL;

QueueHandle_t queue_msg_handle;

uart_config_t uart_config = {
	.baud_rate = UART_BAUD,
	.data_bits = UART_DATA_8_BITS,
	.parity = UART_PARITY_DISABLE,
	.stop_bits = UART_STOP_BITS_1,
	.flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

/**
 * @brief Initi uart with isr queue logging
 *
 * @param uart_config uart config file
 * @param port_num valid uart port number
 * @param gpio_tx tx pin
 * @param gpio_rx rx pin
 * @param rx_buff_size rx buffer size
 * @param tx_buff_size tx buffer size
 * @param isr_queue_handle isr interrupt handle
 * @param isr_queue_size isr queue size (number of interrupt events to store)
 * @param intr_alloc_flags ESP_INTR_FLAG_*
 */
void uart_init_with_isr_queue(uart_config_t *uart_config, uart_port_t port_num, int gpio_tx, int gpio_rx, int tx_buff_size, int rx_buff_size, QueueHandle_t *isr_queue_handle, int isr_queue_size, int intr_alloc_flags)
{
	// Create the ISR queue
	*isr_queue_handle = xQueueCreate(isr_queue_size, sizeof(uart_event_t));
	if (*isr_queue_handle == NULL)
	{
		ESP_LOGE("UART_INIT", "Failed to create ISR queue");
		return;
	}

	ESP_ERROR_CHECK(uart_param_config(port_num, uart_config));
	ESP_ERROR_CHECK(uart_driver_install(port_num, rx_buff_size, tx_buff_size, isr_queue_size, &uart_event_queue_handle, intr_alloc_flags));
	ESP_ERROR_CHECK(uart_set_pin(port_num, gpio_tx, gpio_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

/**
 * @brief Encapsulated message start flag handling
 *
 * Confirm the structure of encapsulated messages start flag.
 *
 * @param uart_num uart port number
 * @param encap_start_buf uint8_t encapsulation start flag buffer
 * @param pattern_index starting index of the detected pattern
 * @return 0 OK
 * @return -1 failed malloc
 * @return -2 not enough bytes in RX buffer
 */
int uart_encapsulation_start_flag_handler(uart_port_t uart_num, int pattern_index)
{
	const char *TAG = "ENCAP START";

	int message_size = pattern_index + ENCAPS_FLAG_SIZE;
	uint8_t *tmp_buffer = (uint8_t *)malloc(message_size * sizeof(uint8_t));
	if (tmp_buffer == NULL)
	{
		ESP_LOGE(TAG, "Failed to malloc tmp_buffer");
		return -2;
	}

	if (uart_read_bytes(uart_num, tmp_buffer, message_size, pdMS_TO_TICKS(100)) != message_size)
	{
		ESP_LOGE(TAG, "Wrong message size");
		free(tmp_buffer);
		return -3;
	}

	int return_value = memcmp(&tmp_buffer[pattern_index], ENCAPS_START_PAT, ENCAPS_FLAG_SIZE);
	free(tmp_buffer); tmp_buffer = NULL;

	return return_value;
}

/**
 * @brief Encapsulated message end flag handling
 *
 * Confirm the position and structure of the encapsulated messages end flag
 *
 * @param uart_num uart port number
 * @param encap_stop_buf uint8_t encapsulation end flag buffer
 * @param pattern_index starting index of the detected pattern
 * @return 0 OK
 * @return -1 wrong pattern indice
 * @return -2 not enough bytes in RX buffer
 */
int uart_encapsulation_end_flag_handler(uart_port_t uart_num, int pattern_index)
{
	uint8_t *tmp_buffer = (uint8_t *)malloc(ENCAPS_FLAG_SIZE*sizeof(uint8_t));

	const char *TAG = "ENCAP STOP";

	if (pattern_index != 1)
	{
		ESP_LOGE(TAG, "Pat indice != -1");
		return -2;
	}

	if (uart_read_bytes(uart_num, tmp_buffer, ENCAPS_FLAG_SIZE, pdMS_TO_TICKS(100)) != ENCAPS_FLAG_SIZE)
	{
		ESP_LOGE(TAG, "Incorrect message size");
		return -3;
	}

	int return_value = memcmp(tmp_buffer, ENCAPS_END_PAT, ENCAPS_FLAG_SIZE);
	free(tmp_buffer); tmp_buffer = NULL;
	return return_value;
}

/**
 * @brief Store the encapsulated message
 *
 * @param uart_num uart port number
 * @param message_buf uint8_t message buffer array
 * @param message_size message buffer size
 * @return 0 OK
 * @return -1 null operator passed
 * @return -2 message_size < 0
 * @return -3 not enough bytes in RX buffer
 */
int uart_encapsulated_message_handler(uart_port_t uart_num, uint8_t *message_buf, int message_size)
{
	/*
	This function simply reads the encapsulated message that should sit between two encapsulation flags.
	*/
	const char *TAG = "ENCAP MSG";
	if (message_buf == NULL)
	{
		ESP_LOGE(TAG, "Null operators passed to the function");
		return -1;
	}
	if (message_size < 0)
		return -2;

	if (uart_read_bytes(uart_num, message_buf, message_size, pdMS_TO_TICKS(100)) != message_size)
	{
		ESP_LOGE(TAG, "Wrong message size");
		return -3;
	}
	return 0;
}

/**
 * @brief Handles the state of the received message and manages the UART communication flow
 *
 * Function checks if the successfull start and stop flags have been received and if the received message was complete.
 *
 * @param uart_num uart port number
 * @param encap_state encapsulation state counter (nothing happened = 0, start flag received = 1, stop flag received = 2)
 * @param pattern_index index of the detected pattern
 * @param encap_start_buf debug buffer for checking how the received start flag looks like
 * @param encap_stop_buf debuf buffer for checking how the stop flag looks like
 * @param message_buf received message buffer
 * @param message_size received message size
 * @return 1 start flag ok message received successfully
 * @return 0 message received ok
 * @return -1 bad start flag
 * @return -2 message size < 1
 * @return -3 failed to malloc message buffer
 * @return -4 failed to to read the message from RX buffer
 * @return -5 bad stop flag
 * @return -6 failed malloc message buffer for queue message
 * @return -5 failed to send the message copy to queue
 */
int uart_encapsulation_handler(uart_port_t uart_num, int *encap_state, int *pattern_index)
{
	const char *TAG = "ENCAP HANDLER";
	uint8_t *tmp_message_buf = NULL;
	int tmp_message_size = 0;
	int error_code = 0; // To store the error code and then return it

	// Parse START FLAG
	if (*encap_state == 0)
	{
		if (uart_encapsulation_start_flag_handler(UART_NUM, *pattern_index) == 0)
		{
			(*encap_state)++;
			return 1; // Start flag OK, return immediately
		}
		else
		{
			ESP_LOGE(TAG, "Bad start flag");
			error_code = -1; // Error code for bad start flag
			goto cleanup;
		}
	}
	else if (*encap_state == 1)
	{
		tmp_message_size = *pattern_index - 1;

		if (tmp_message_size < 1)
		{
			ESP_LOGE(TAG, "Message size < 1");
			error_code = -2; // Error code for invalid message size
			goto cleanup;
		}

		tmp_message_buf = (uint8_t *)malloc(tmp_message_size * sizeof(uint8_t));
		if (tmp_message_buf == NULL)
		{
			ESP_LOGE(TAG, "Failed to malloc tmp data buffer");
			error_code = -3; // Error code for memory allocation failure
			goto cleanup;
		}

		if (uart_encapsulated_message_handler(UART_NUM, tmp_message_buf, tmp_message_size) == 0)
		{
			(*encap_state)++;
			*pattern_index -= tmp_message_size;
		}
		else
		{
			ESP_LOGE(TAG, "Failed to receive message");
			error_code = -4; // Error code for failed message receipt
			goto cleanup;
		}
	}

	if ((*encap_state == 2) && (uart_encapsulation_end_flag_handler(UART_NUM, *pattern_index) == 0))
	{
		TaskQueueMessage_type message_to_queue = {
			.msg_ptr = (uint8_t *)malloc(tmp_message_size * sizeof(uint8_t)),
			.msg_size = tmp_message_size};

		if (message_to_queue.msg_ptr == NULL)
		{
			error_code = -6; // Error code for queue message memory allocation failure
			goto cleanup;
		}
		memcpy(message_to_queue.msg_ptr, tmp_message_buf, tmp_message_size * sizeof(uint8_t));

		if (xQueueSend(queue_msg_handle, &message_to_queue, portMAX_DELAY) != pdTRUE)
		{
			free(message_to_queue.msg_ptr);
			error_code = -7; // Error code for queue send failure
			goto cleanup;
		}
		free(tmp_message_buf); // Free only if successful
		tmp_message_buf = NULL;
		*encap_state = 0; // Move state forward only on success
		uart_pattern_queue_reset(UART_NUM, UART_PAT_QUEUE_SIZE);
		return 0; // Successful execution
	}
	else
	{
		ESP_LOGE(TAG, "Failed stop flag");
		error_code = -5; // Error code for failed stop flag
		goto cleanup;
	}

cleanup:
	if (tmp_message_buf != NULL)
	{
		free(tmp_message_buf);
	}
	*encap_state = 0;
	uart_pattern_queue_reset(UART_NUM, UART_PAT_QUEUE_SIZE);
	return error_code;
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
		if (xQueueReceive(uart_event_queue_handle, (void *)&uart_event, portMAX_DELAY) == pdTRUE)
		{
			// Clear the buffer
			ESP_LOGI(TAG, "[EVENT]: %d, [SIZE]: %d", uart_event.type, uart_event.size);

			switch (uart_event.type)
			{
			/**
			 * @brief Pattern detection case
			 *
			 * Multiple identical characters of specific type detected in a row.
			 * Number of chars is defined with macro UART_PATTERN_SIZE
			 *
			 */
			case UART_PATTERN_DET:
			{
				const char *descriptor = "[UART_PAT]: "; /*!<  message description*/

				int pattern_index = uart_pattern_pop_pos(UART_NUM);
				ESP_LOGI(descriptor, "Pat index: %d", pattern_index);
				if (pattern_index == -1)
				{
					ESP_LOGE(TAG, "Pattern index -1");
					break;
				}

				error_flag = uart_encapsulation_handler(UART_NUM, &encapsulation_counter, &pattern_index);

				if (error_flag == 0)
				{
					ESP_LOGI(TAG, "Successfull message: %d", error_flag);
				}
				else
				{
					ESP_LOGE(TAG, "Error flag: %d", error_flag);
				}
				break;
			}
			default:
			{
				ESP_LOGI(TAG, "Default switch state");
				break;
			}
			}
		}
	}
	ESP_LOGW(TAG, "KILLING THE UART ISR TASK");
	vTaskDelete(NULL);
}

void task_receive_message(void *params)
{
	const char *TAG = "RECEIVE MSG";
	TaskQueueMessage_type enqueued_message;

	while (1)
	{
		if (xQueueReceive(queue_msg_handle, &enqueued_message, portMAX_DELAY))
		{
			if (enqueued_message.msg_ptr != NULL)
			{

				ESP_LOGI(TAG, "Sending message");
				// Write the message
				if(uart_write_bytes(UART_NUM, enqueued_message.msg_ptr, enqueued_message.msg_size) == -1)
				{
					ESP_LOGE(TAG, "Failed to write bytes");
				}

				// Free the message memory
				free(enqueued_message.msg_ptr);
			}
			else
			{
				ESP_LOGE(TAG, "Null pointer passed");
			}
		}
	}
}