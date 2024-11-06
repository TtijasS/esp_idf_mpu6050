#ifndef UART_ISR_HANDLER_H
#define UART_ISR_HANDLER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "app_tasks.h"

#define UART_BAUD 460800
#define UART_NUM UART_NUM_0
#define UART_RX_BUFF_SIZE 1024
#define UART_TX_BUFF_SIZE 0
#define UART_EVENT_QUEUE_SIZE 10 /*!< Number of UART ISR events queued*/
#define UART_PAT_QUEUE_SIZE 8 /*!< Number of queued pattern index positions*/
#define UART_TXD 43
#define UART_RXD 44

#define ENCAP_START_PAT "++*"
#define ENCAP_END_PAT "*++"
#define UART_PATTERN_SIZE (strlen((const char*)ENCAP_START_PAT) - 1)
#define ENCAP_FLAG_SIZE strlen((const char*)ENCAP_START_PAT)

// Uart config struct
extern uart_config_t uart_config;

// init uart
int myuart_init_with_isr_queue(uart_config_t *uart_config, uart_port_t port_num, int gpio_tx, int gpio_rx, int tx_buff_size, int rx_buff_size, QueueHandle_t *isr_queue_handle, int isr_queue_size, int intr_alloc_flags);
int myuart_encapsulation_start_flag_handler(uart_port_t uart_num, int pattern_index);
int myuart_encapsulation_end_flag_handler(uart_port_t uart_num, int pattern_index);
int myuart_encapsulated_message_handler(uart_port_t uart_num, uint8_t *message_buf, int message_size);
int myuart_message_send_to_queue(uint8_t *message, size_t message_size);
int myuart_encapsulation_handler(uart_port_t uart_num, int *encap_state, int *pattern_index);


#endif // UART_ISR_HANDLER_H