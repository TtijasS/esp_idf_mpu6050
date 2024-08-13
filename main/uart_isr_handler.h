#ifndef UART_ISR_HANDLER_H
#define UART_ISR_HANDLER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_BAUD 115200
#define UART_NUM UART_NUM_0
#define UART_RX_BUFF_SIZE 1024*4
#define UART_TX_BUFF_SIZE 0
#define UART_EVENT_QUEUE_SIZE 10 /*!< Number of UART ISR events queued*/
#define UART_PAT_QUEUE_SIZE 8 /*!< Number of queued pattern index positions*/
#define UART_TXD 43
#define UART_RXD 44

#define TASK_STACK_SIZE 1024
#define ENCAPS_START_PAT "+++++++*"
#define ENCAPS_END_PAT "*+++++++"
#define UART_PATTERN_SIZE (strlen((const char*)ENCAPS_START_PAT) - 1)
#define ENCAPS_FLAG_SIZE strlen((const char*)ENCAPS_START_PAT)

extern QueueHandle_t uart_event_queue_handle;
extern QueueHandle_t queue_msg_handle;

typedef struct TaskQueueMessage_type
{
	size_t msg_size;
	uint8_t *msg_ptr;
	
}TaskQueueMessage_type;

// configure uart
extern uart_config_t uart_config;

// init uart
void uart_init_with_isr_queue(uart_config_t *uart_config, uart_port_t port_num, int gpio_tx, int gpio_rx, int tx_buff_size, int rx_buff_size, QueueHandle_t *isr_queue_handle, int isr_queue_size, int intr_alloc_flags);

void task_uart_isr_monitoring(void *);
void task_receive_message(void *);
#endif // UART_ISR_HANDLER_H