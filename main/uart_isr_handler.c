#include "uart_isr_handler.h"


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
 * @return 0 OK
 * @return -1 failed to create isre queue handle
 * @return -2 failed to config uart params
 * @return -3 failed to install uart driver
 * @return -4 failed to set uart pins
 */
int uart_init_with_isr_queue(uart_config_t *uart_config, uart_port_t port_num, int gpio_tx, int gpio_rx, int tx_buff_size, int rx_buff_size, QueueHandle_t *isr_queue_handle, int isr_queue_size, int intr_alloc_flags)
{
	const char *TAG = "UART INIT WITH Q";
	esp_log_level_set(TAG, ESP_LOG_ERROR);

	int error_code = 0; // used for debugging
	// Create the ISR queue
	*isr_queue_handle = xQueueCreate(isr_queue_size, sizeof(uart_event_t));
	if (*isr_queue_handle == NULL)
	{
		return -1;
	}
	if ((error_code = uart_param_config(port_num, uart_config)) != 0)
	{
		return -2;
	}
	if ((error_code = uart_driver_install(port_num, rx_buff_size, tx_buff_size, isr_queue_size, isr_queue_handle, intr_alloc_flags)) != 0)
	{
		return -3;
	}
	if ((error_code = uart_set_pin(port_num, gpio_tx, gpio_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) != 0)
	{
		return -4;
	}
	return 0;
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

	int message_size = pattern_index + ENCAP_FLAG_SIZE;
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

	int return_value = memcmp(&tmp_buffer[pattern_index], ENCAP_START_PAT, ENCAP_FLAG_SIZE);
	free(tmp_buffer);
	tmp_buffer = NULL;

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
	uint8_t *tmp_buffer = (uint8_t *)malloc(ENCAP_FLAG_SIZE * sizeof(uint8_t));

	const char *TAG = "ENCAP STOP";

	if (pattern_index != 1)
	{
		ESP_LOGE(TAG, "Pat indice != -1");
		return -2;
	}

	if (uart_read_bytes(uart_num, tmp_buffer, ENCAP_FLAG_SIZE, pdMS_TO_TICKS(100)) != ENCAP_FLAG_SIZE)
	{
		ESP_LOGE(TAG, "Incorrect message size");
		return -3;
	}

	int return_value = memcmp(tmp_buffer, ENCAP_END_PAT, ENCAP_FLAG_SIZE);
	free(tmp_buffer);
	tmp_buffer = NULL;
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

int message_send_to_queue(uint8_t *message, size_t message_size)
{
	const char *TAG = "MSG PREP SEND TO Q";

	if (message == NULL)
	{
		ESP_LOGE(TAG, "Message null pointer passed");
		return -1;
	}

	TaskQueueMessage_type message_to_queue = {
		.msg_ptr = (uint8_t *)malloc(message_size * sizeof(uint8_t)),
		.msg_size = message_size};

	if (message_to_queue.msg_ptr == NULL)
	{
		ESP_LOGE(TAG, "Failed to malloc queued message buffer");
		return -2;
	}
	// Copy message to the new address
	memcpy(message_to_queue.msg_ptr, message, message_size * sizeof(uint8_t));

	// Send the message to queue
	if (xQueueSend(queue_enqueued_msg_processing, &message_to_queue, portMAX_DELAY) != pdTRUE)
	{
		free(message_to_queue.msg_ptr);
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
 * @return -5 failed malloc message buffer for queue message
 * @return -6 failed to send the message copy to queue
 * @return -7 bad stop flag
 */
int uart_encapsulation_handler(uart_port_t uart_num, int *encap_state, int *pattern_index)
{
	enum encap_states
	{
		START_FLAG_CONFIRMATION,
		END_FLAG_CONFIRMATION,
	};
	const char *TAG = "ENCAP HANDLER";
	uint8_t *tmp_message_buf = NULL;
	int tmp_message_size = 0; // To store corrected message size
	int error_code = 0;		  // To store the error code and then return it

	switch (*encap_state)
	{
	case START_FLAG_CONFIRMATION:
		/**
		 * @brief Read and check the START FLAG
		 */
		if (uart_encapsulation_start_flag_handler(UART_NUM, *pattern_index) == 0)
		{
			(*encap_state)++;
			return 1; // Start flag OK, return immediately
		}
		else
		{
			ESP_LOGE(TAG, "Bad start flag");
			error_code = -1; // Error code for bad start flag
			goto memcleanup;
		}
		break;

	case END_FLAG_CONFIRMATION:
		tmp_message_size = *pattern_index - 1;

		if (tmp_message_size < 1)
		{
			ESP_LOGE(TAG, "Message size %d is less than 1", tmp_message_size);
			error_code = -2; // Error code for invalid message size
			goto memcleanup;
		}

		tmp_message_buf = (uint8_t *)malloc(tmp_message_size * sizeof(uint8_t));
		if (tmp_message_buf == NULL)
		{
			ESP_LOGE(TAG, "Failed to malloc tmp message buffer");
			error_code = -3; // Error code for memory allocation failure
			goto memcleanup;
		}

		/**
		 * @brief Read the MESSAGE bytes
		 */
		if (uart_encapsulated_message_handler(UART_NUM, tmp_message_buf, tmp_message_size) == 0)
		{
			*pattern_index -= tmp_message_size;
		}
		else
		{
			ESP_LOGE(TAG, "Failed to receive message");
			error_code = -4; // Error code for failed message receipt
			goto memcleanup;
		}

		/**
		 * @brief Read and check STOP FLAG
		 */
		if (uart_encapsulation_end_flag_handler(UART_NUM, *pattern_index) == 0)
		{
			if (message_send_to_queue(tmp_message_buf, tmp_message_size) != 0)
			{
				ESP_LOGE(TAG, "Failed to send message to queue");
			}
		}
		else
		{
			ESP_LOGE(TAG, "Bad stop flag");
			error_code = -7; // Error code for failed stop flag
		}
		break;

	default:
		break;
	}
memcleanup:
	if (tmp_message_buf != NULL)
	{
		free(tmp_message_buf);
	}
	*encap_state = 0;
	uart_pattern_queue_reset(UART_NUM, UART_PAT_QUEUE_SIZE);
	return error_code;
}

