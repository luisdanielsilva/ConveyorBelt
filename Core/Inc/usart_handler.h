#ifndef USART_HANDLER_H
#define USART_HANDLER_H

/* USER CODE BEGIN Includes */
#include <main.h>
#include <usart.h>
#include <string.h>
#include <stdio.h>
#include "stdint.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Prototypes */
void enqueue_command(char* cmd);
int dequeue_command(char* cmd);
void clear_command_queue(void);
void print_command_queue(void);

void start_uart_receive(void);
void send_uart(const char* msg);
/* USER CODE END Prototypes */

/* USER CODE BEGIN Private defines */
#define QUEUE_SIZE 5

typedef struct {
    char commands[QUEUE_SIZE][20];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} CommandQueue;

extern CommandQueue cmd_queue;
extern uint8_t speed_percentage;
/* USER CODE END Private defines */


#endif // USART_HANDLER_H