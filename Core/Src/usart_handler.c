#include "usart_handler.h"


/* USER CODE BEGIN PV */
CommandQueue cmd_queue = {{{0}}, 0, 0, 0};

char rx_buffer[20];
uint8_t rx_index = 0;

uint8_t speed_percentage = 0; // Speed percentage (0-100)
/* USER CODE END PV */


/* USER CODE BEGIN 0 */
void enqueue_command(char* cmd) {
    if (cmd_queue.count < QUEUE_SIZE) {
        strcpy(cmd_queue.commands[cmd_queue.head], cmd);
        cmd_queue.head = (cmd_queue.head + 1) % QUEUE_SIZE;
        cmd_queue.count++;
    } else {
        send_uart("Command queue full\r\n");
    }
}

int dequeue_command(char* cmd) {
    if (cmd_queue.count > 0) {
        strcpy(cmd, cmd_queue.commands[cmd_queue.tail]);
        cmd_queue.tail = (cmd_queue.tail + 1) % QUEUE_SIZE;
        cmd_queue.count--;
        return 1;
    }
    return 0;
}

void clear_command_queue(void) {
    cmd_queue.head = 0;
    cmd_queue.tail = 0;
    cmd_queue.count = 0;
    for (uint8_t i = 0; i < QUEUE_SIZE; i++) {
        cmd_queue.commands[i][0] = '\0';
    }
}

void print_command_queue(void) {
    char msg[100];
    if (cmd_queue.count == 0) {
        sprintf(msg, "Command Queue: EMPTY\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    } else {
        sprintf(msg, "Command Queue: %d commands, Head: %d, Tail: %d\r\n", 
                cmd_queue.count, cmd_queue.head, cmd_queue.tail);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        for (uint8_t i = 0; i < cmd_queue.count; i++) {
            uint8_t index = (cmd_queue.tail + i) % QUEUE_SIZE;
            sprintf(msg, "Queue[%d]: %s\r\n", i, cmd_queue.commands[index]);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        }
    }
}

/* Send UART message */
void send_uart(const char* msg) {
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

/* Start UART receive interrupt */
void start_uart_receive(void) {
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[rx_index], 1);
}

/* void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (rx_buffer[rx_index] == '\n') {
            rx_buffer[rx_index] = '\0';
            enqueue_command(rx_buffer);
// DEBUG
//            char msg[50];
//            sprintf(msg,  " Enqueued SERIAL Command: %s\r\n", rx_buffer);
//            send_uart(msg);
// END DEBUG
            rx_index = 0;
        } else {
            rx_index = (rx_index < sizeof(rx_buffer) - 1) ? rx_index + 1 : 0;
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[rx_index], 1);
    }
} */

/* void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (rx_index == 0 && (rx_buffer[0] != 'S' && rx_buffer[0] != 'T' && 
                              rx_buffer[0] != 'D' && rx_buffer[0] != 'E')) {
            rx_index = 0; // Reset if first char invalid
        } else if (rx_index == 1 && rx_buffer[rx_index] == '\r') {
            rx_index++; // Expect LF next
        } else if (rx_index == 2 && rx_buffer[rx_index] == '\n') {
            rx_buffer[1] = '\0'; // Terminate after command
            enqueue_command(rx_buffer);
// DEBUG
            char msg[50];
            sprintf(msg, "Enqueued SERIAL Command: %s\r\n", rx_buffer);
            send_uart(msg); 
// END DEBUG
            rx_index = 0; // Reset after processing
        } else {
            rx_index = (rx_index < 2) ? rx_index + 1 : 0; // Reset on invalid or overflow
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[rx_index], 1);
    }
} */

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (rx_index == 0 && (rx_buffer[0] != 'S' && rx_buffer[0] != 'T' && 
                              rx_buffer[0] != 'D' && rx_buffer[0] != 'E' && 
                              rx_buffer[0] != 'P')) {
            rx_index = 0; // Reset if first char invalid
        } else if (rx_index == 1 && rx_buffer[0] == 'P' && (rx_buffer[1] < '0' || rx_buffer[1] > '9')) {
            rx_index = 0; // Reset if P not followed by digit
        } else if (rx_index == 2 && rx_buffer[0] == 'P' && (rx_buffer[2] < '0' || rx_buffer[2] > '9')) {
            rx_index = 0; // Reset if second char not digit
        } else if (rx_index == 3 && rx_buffer[0] == 'P' && rx_buffer[3] == '\r') {
            rx_index++; // Expect LF next
        } else if (rx_index == 4 && rx_buffer[0] == 'P' && rx_buffer[4] == '\n') {
            // Parse P command (e.g., "P30\r\n")
            uint8_t speed = (rx_buffer[1] - '0') * 10 + (rx_buffer[2] - '0');
            speed_percentage = (speed + 5) / 10 * 10; // Round to nearest 10%
            if (speed_percentage <= 100) {
                char msg[50];
                sprintf(msg, "Speed Set: %d%%\r\n", speed_percentage);
                send_uart(msg);
            }
            rx_index = 0; // Reset after processing
        } else if (rx_index == 1 && rx_buffer[1] == '\r') {
            rx_index++; // Expect LF for single-char commands
        } else if (rx_index == 2 && rx_buffer[0] != 'P' && rx_buffer[2] == '\n') {
            rx_buffer[1] = '\0'; // Terminate after command
            enqueue_command(rx_buffer);
            char msg[50];
            sprintf(msg, "Enqueued SERIAL Command: %s\r\n", rx_buffer);
            send_uart(msg);
            rx_index = 0; // Reset after processing
        } else {
            rx_index = (rx_index < 4) ? rx_index + 1 : 0; // Reset on invalid or overflow
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_buffer[rx_index], 1);
    }
}









/* USER CODE END 0*/