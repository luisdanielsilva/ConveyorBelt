/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart_handler.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h> // For abs()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STOPPED,
    RUNNING_FORWARD,
    RUNNING_REVERSE,
    PAUSED,
    FAULT
} ConveyorState;

typedef struct {
    ConveyorState state;
    ConveyorState (*callback)(void);  // Function pointer for state behavior
} StateMachine;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t last_adc_value = 0;


//uint32_t last_debounce_time = 0;
const uint32_t DEBOUNCE_DELAY = 50;  // 50ms debounce
const uint32_t SPEED_UPDATE_INTERVAL = 500;  // 500ms for speed updates
const uint32_t ADC_CHANGE_THRESHOLD = 82;  // ~2% of 4095 (12-bit ADC)

uint8_t direction_forward = 1;  // 1 for forward, 0 for reverse
uint8_t dir_button_pressed = 0; // Track DIR button state for press-and-release

uint8_t usart_start_pressed = 0; // Simulate START button state
uint8_t usart_stop_pressed = 0;  // Simulate STOP button state
uint8_t usart_sensor_triggered = 0; // Simulate SENSOR state


#define DEBOUNCE_DELAY 50 // 50ms debounce period
#define STABLE_COUNT 3    // Require 3 stable readings

#define COMMAND_SUPPRESS_DELAY 100 // 100ms to suppress redundant commands

/*
uint8_t start_stable_count = 0;
uint8_t stop_stable_count = 0;
uint8_t dir_stable_count = 0;
uint8_t sensor_stable_count = 0;
uint8_t start_last_state = GPIO_PIN_RESET;
uint8_t stop_last_state = GPIO_PIN_RESET;
uint8_t dir_last_state = GPIO_PIN_RESET;
uint8_t sensor_last_state = GPIO_PIN_RESET;
*/
uint32_t last_start_debounce_time = 0;
uint32_t last_stop_debounce_time = 0;
uint32_t last_dir_debounce_time = 0;
uint32_t last_sensor_debounce_time = 0;

uint32_t last_start_command_time = 0; // Track last START command
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
ConveyorState stopped_state(void);
ConveyorState running_forward_state(void);
ConveyorState running_reverse_state(void);
ConveyorState paused_state(void);
ConveyorState fault_state(void);

StateMachine state_machine[] = {
    {STOPPED, stopped_state},
    {RUNNING_FORWARD, running_forward_state},
    {RUNNING_REVERSE, running_reverse_state},
    {PAUSED, paused_state},
    {FAULT, fault_state}
};

ConveyorState current_state = STOPPED;
ConveyorState previous_running_state = RUNNING_FORWARD;  // Track last running direction

void update_state(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ConveyorState stopped_state(void) {
    if (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin) == GPIO_PIN_SET || usart_start_pressed) {
        usart_start_pressed = 0; // Reset USART flag
        ConveyorState next = direction_forward ? RUNNING_FORWARD : RUNNING_REVERSE;
        previous_running_state = next;
         char debug_msg[70];
        sprintf(debug_msg, "STOPPED: Transition to %s (direction_forward=%d)\r\n", 
                next == RUNNING_FORWARD ? "RUNNING_FORWARD" : "RUNNING_REVERSE", direction_forward);
        send_uart(debug_msg);
        return next;
    }
    // DEBUG
    usart_sensor_triggered = 0;
    // DEBUG END
    return STOPPED;
}

ConveyorState running_forward_state(void) {
    if (HAL_GPIO_ReadPin(STOP_GPIO_Port, STOP_Pin) == GPIO_PIN_SET || usart_stop_pressed) {
        usart_stop_pressed = 0;
        return STOPPED;
    } else if (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin) == GPIO_PIN_SET || usart_start_pressed) {
        usart_start_pressed = 0;
        return PAUSED;
    } else if (HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin) == GPIO_PIN_SET || usart_sensor_triggered) {
        usart_sensor_triggered = 0;
        return FAULT;
    } else if (!direction_forward) {
        previous_running_state = RUNNING_REVERSE;
        return RUNNING_REVERSE;
    }
    return RUNNING_FORWARD;
}

ConveyorState running_reverse_state(void) {
    if (HAL_GPIO_ReadPin(STOP_GPIO_Port, STOP_Pin) == GPIO_PIN_SET || usart_stop_pressed) {
        usart_stop_pressed = 0;
        return STOPPED;
    } else if (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin) == GPIO_PIN_SET || usart_start_pressed) {
        usart_start_pressed = 0;
        return PAUSED;
    } else if (HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin) == GPIO_PIN_SET || usart_sensor_triggered) {
        usart_sensor_triggered = 0;
        return FAULT;
    } else if (direction_forward) {
        previous_running_state = RUNNING_FORWARD;
        return RUNNING_FORWARD;
    }
    return RUNNING_REVERSE;
}

ConveyorState paused_state(void) {
    if (HAL_GPIO_ReadPin(STOP_GPIO_Port, STOP_Pin) == GPIO_PIN_SET || usart_stop_pressed) {
        usart_stop_pressed = 0;
        return STOPPED;
    } else if (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin) == GPIO_PIN_SET || usart_start_pressed) {
        usart_start_pressed = 0;
        return direction_forward ? RUNNING_FORWARD : RUNNING_REVERSE;
    } else if (HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin) == GPIO_PIN_SET || usart_sensor_triggered) {
        usart_sensor_triggered = 0;
        return FAULT;
    }
    return PAUSED;
}

ConveyorState fault_state(void) {
    if (HAL_GPIO_ReadPin(STOP_GPIO_Port, STOP_Pin) == GPIO_PIN_SET || usart_stop_pressed) {
        usart_stop_pressed = 0;
        return STOPPED;
    }
    usart_sensor_triggered = 0;
    return FAULT;
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    

    if (GPIO_Pin == START_Pin) {
        if (HAL_GetTick() - last_start_debounce_time < DEBOUNCE_DELAY) {
            return;
        }
        last_start_debounce_time = HAL_GetTick();
        // Check pin states and send UART messages
        if (GPIO_Pin == START_Pin && HAL_GPIO_ReadPin(GPIOA, START_Pin) == GPIO_PIN_SET) {
            clear_command_queue(); // Clear queue on START press
            //send_uart("START: PRESSED\r\n");
            usart_start_pressed = 1; // Set flag directly instead of enqueuing
            last_start_command_time = HAL_GetTick();
            update_state(); // Process state change immediately
            //enqueue_command("S");
        } else if (GPIO_Pin == START_Pin) {
            //send_uart("START: RELEASED\r\n");           // the release can be removed later
        }
    }

    if (GPIO_Pin == STOP_Pin) {
        if (HAL_GetTick() - last_stop_debounce_time < DEBOUNCE_DELAY) {
            return;
        }
        last_stop_debounce_time = HAL_GetTick();
        if (GPIO_Pin == STOP_Pin && HAL_GPIO_ReadPin(GPIOB, STOP_Pin) == GPIO_PIN_SET) {
            // DEBUG
            usart_stop_pressed = 0;
            // DEBUG END
            clear_command_queue(); // Clear queue on STOP press
            //send_uart("STOP: PRESSED\r\n");
            enqueue_command("T");
        } else if (GPIO_Pin == STOP_Pin) {
            //send_uart("STOP: RELEASED\r\n");
        }
    }

   /*  if (GPIO_Pin == DIR_Pin) {
        if (HAL_GetTick() - last_dir_debounce_time < DEBOUNCE_DELAY) {
            return;
        }
        last_dir_debounce_time = HAL_GetTick();
        
        if (HAL_GPIO_ReadPin(DIR_GPIO_Port, DIR_Pin) == GPIO_PIN_SET) {
            dir_button_pressed = 1; // Mark button as pressed
            send_uart("DIR: PRESSED\r\n");
        } else if (dir_button_pressed) {
            clear_command_queue(); // Clear queue on DIR press
            direction_forward = !direction_forward; // Toggle direction on release
            send_uart(direction_forward ? "DIR: TOGGLED TO FORWARD\r\n" : "DIR: TOGGLED TO REVERSE\r\n");
            enqueue_command("D");
            //dir_button_pressed = 0; // Reset press state
            update_state(); // Process state change immediately
        }
    } */

/*     if (GPIO_Pin == DIR_Pin) {
        if (HAL_GetTick() - last_dir_debounce_time < DEBOUNCE_DELAY) {
            return;
        }
        last_dir_debounce_time = HAL_GetTick();
        GPIO_PinState dir_state = HAL_GPIO_ReadPin(GPIOA, DIR_Pin);
        char debug_msg[50];
        sprintf(debug_msg, "DIR Pin State: %d\r\n", dir_state);
        send_uart(debug_msg);
        if (dir_state == GPIO_PIN_SET) {
            dir_button_pressed = 1;
            //send_uart("DIR: PRESSED\r\n");
            //print_command_queue(); // Print queue after press
        } else if (dir_button_pressed && dir_state == GPIO_PIN_RESET) {
            //print_command_queue(); // Print queue before release
            clear_command_queue();
            direction_forward = !direction_forward;
            sprintf(debug_msg, "DIR: TOGGLED TO %s (direction_forward=%d)\r\n", 
                    direction_forward ? "FORWARD" : "REVERSE", direction_forward);
            send_uart(debug_msg);
            dir_button_pressed = 0;
            update_state(); // Apply toggle immediately
        }
    } */

    if (GPIO_Pin == DIR_Pin) {
        if (HAL_GetTick() - last_dir_debounce_time < DEBOUNCE_DELAY) {
            return;
        }
        last_dir_debounce_time = HAL_GetTick();
        /* GPIO_PinState dir_state = HAL_GPIO_ReadPin(GPIOA, DIR_Pin);
        char debug_msg[50];
        sprintf(debug_msg, "DIR Pin State: %d\r\n", dir_state);
        send_uart(debug_msg); */
        if (HAL_GPIO_ReadPin(DIR_GPIO_Port, DIR_Pin) == GPIO_PIN_SET) {
            dir_button_pressed = 1;
            //send_uart("DIR: PRESSED\r\n");
            //print_command_queue(); // Print queue after press
        } else if (dir_button_pressed) {
            //print_command_queue(); // Print queue before release
            clear_command_queue();
            direction_forward = !direction_forward;
            char debug_msg[50];
            sprintf(debug_msg, "DIR: TOGGLED TO %s (direction_forward=%d)\r\n", 
                    direction_forward ? "FORWARD" : "REVERSE", direction_forward);
            send_uart(debug_msg);
            dir_button_pressed = 0;

            update_state(); // Apply toggle immediately
        }
    }





    if (GPIO_Pin == SENSOR_Pin) {
        if (HAL_GetTick() - last_sensor_debounce_time < DEBOUNCE_DELAY) {
            return;
        }
        last_sensor_debounce_time = HAL_GetTick();
    if (GPIO_Pin == SENSOR_Pin && HAL_GPIO_ReadPin(GPIOA, SENSOR_Pin) == GPIO_PIN_SET) {
 // DEBUG
        usart_sensor_triggered = 0;
// DEBUG END
        clear_command_queue(); // Clear queue on SENSOR trigger
        send_uart("SENSOR: TRIGGERED\r\n");
        enqueue_command("E");
    } 
    }
    //update_state();  // Update state machine after handling inputs

    if (GPIO_Pin != START_Pin) { // only for non-START button presses.
        update_state();
    }
}



// ADDED FOR INTERRUPT-DRIVEN ADC HANDLING
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        uint32_t adc_value = HAL_ADC_GetValue(hadc);
        
        /* // Check for significant change (±2%)
        if (abs((int32_t)adc_value - (int32_t)last_adc_value) > ADC_CHANGE_THRESHOLD) {
            uint32_t speed_percent = (adc_value * 100) / 4095;  // Scale to 0-100%
            char msg[50];
            sprintf(msg, "SPEED: %lu%%\r\n", speed_percent);
            send_uart(msg);
            last_adc_value = adc_value;
        } */

         // Check for significant change (±2%)
        if (abs((int32_t)adc_value - (int32_t)last_adc_value) > ADC_CHANGE_THRESHOLD) {
            uint32_t raw_percentage = (adc_value * 100) / 4095; // Scale to 0–100%
            speed_percentage = (raw_percentage + 5) / 10 * 10; // Round to nearest 10%
            if (speed_percentage > 100) speed_percentage = 100; // Clamp to 100%
            char msg[50];
            sprintf(msg, "SPEED: %d%%\r\n", speed_percentage);
            send_uart(msg);
            last_adc_value = adc_value;
        }
        
        // Restart the next conversion
        HAL_ADC_Start_IT(&hadc1);
    }
}

static uint32_t last_sensor_time = 0; // For debouncing SENSOR

void update_state(void) {
    for (uint8_t i = 0; i < sizeof(state_machine) / sizeof(state_machine[0]); i++) {
        if (state_machine[i].state == current_state) {
            ConveyorState new_state = state_machine[i].callback();
            if (new_state != current_state) {
                current_state = new_state;
                char msg[50];
                const char* state_str;
                switch (current_state) {
                    case STOPPED: state_str = "STOPPED"; break;
                    case RUNNING_FORWARD: state_str = "RUNNING_FORWARD"; break;
                    case RUNNING_REVERSE: state_str = "RUNNING_REVERSE"; break;
                    case PAUSED: state_str = "PAUSED"; break;
                    case FAULT: state_str = "FAULT"; break;
                    default: state_str = "UNKNOWN"; break;
                }
                sprintf(msg, "State -> %s\r\n", state_str);
                send_uart(msg);
            }
            break;
        }
    }

   /*  // Debug current state
    char msg[50];
    const char* state_str;
    switch (current_state) {
        case STOPPED: state_str = "STOPPED"; break;
        case RUNNING_FORWARD: state_str = "RUNNING_FORWARD"; break;
        case RUNNING_REVERSE: state_str = "RUNNING_REVERSE"; break;
        case PAUSED: state_str = "PAUSED"; break;
        case FAULT: state_str = "FAULT"; break;
        default: state_str = "UNKNOWN"; break;
    }
    sprintf(msg, "Current State: %s\r\n", state_str);
    send_uart(msg); */

    // Check SENSOR (e.g., PC13) with debouncing
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET && 
                HAL_GetTick() - last_sensor_time >= 50) { // 50ms debounce
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED on
                char msg[50];
                sprintf(msg, "SENSOR Triggered\r\n");
                send_uart(msg);
                last_sensor_time = HAL_GetTick();
            } else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED off
            }

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  clear_command_queue(); // Clear queue on startup
  send_uart("Open Source Conveyor Started\r\n");
  start_uart_receive();  // Start UART receive interrupt
  HAL_ADC_Start_IT(&hadc1);  // Start ADC in interrupt mode
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { /* USER CODE BEGIN 3 */
 
   char cmd[20];
   if (dequeue_command(cmd)) {
    char msg[50];
    if (strcmp(cmd, "D") == 0) {
          sprintf(msg, "Processed Command: %s (UART)\r\n", cmd);
        send_uart(msg);
        direction_forward = !direction_forward;
        char debug_msg[50];
        sprintf(debug_msg, "DIR: TOGGLED TO %s (direction_forward=%d)\r\n", 
                direction_forward ? "FORWARD" : "REVERSE", direction_forward);
        send_uart(debug_msg);
        update_state();
       
    } else if (strcmp(cmd, "S") == 0) {
        if (HAL_GetTick() - last_start_command_time >= COMMAND_SUPPRESS_DELAY) {
            sprintf(msg, "Processed Command: %s (UART)\r\n", cmd);
            send_uart(msg);
            usart_start_pressed = 1;
            last_start_command_time = HAL_GetTick();
            update_state();
            }
    } else if (strcmp(cmd, "T") == 0) {
        sprintf(msg, "Processed Command: %s (UART)\r\n", cmd);
        send_uart(msg);
        usart_stop_pressed = 1; // Simulate STOP press
        update_state();
    } else if (strcmp(cmd, "E") == 0) {
        sprintf(msg, "Processed Command: %s (UART)\r\n", cmd);
        send_uart(msg);
        usart_sensor_triggered = 1; // Simulate SENSOR trigger
        update_state();
    }
    clear_command_queue(); // Clear queue after processing
}


  HAL_Delay(100);
  /* USER CODE END 3 */
  /* USER CODE END WHILE */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

// New SystemClock_Config with Over-Drive for 180MHz.
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitStruct.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while (1);
    }

    /** Activate the Over-Drive mode
    */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        while (1);
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        while (1);
    }
}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
