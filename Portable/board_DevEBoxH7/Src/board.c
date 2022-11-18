/*

The MIT License (MIT)

Copyright (c) 2022 Ryan Edwards

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "usbd_gs_can.h"
#include "can.h"
#include "led.h"

#define TASK_MY_PROGRAM_STACK_SIZE (512 / sizeof(portSTACK_TYPE))
#define TASK_MY_PROGRAM_PRIORITY (tskIDLE_PRIORITY + 3)

extern void main_usbd_gs_can_set_channel_cb(USBD_HandleTypeDef *hUSB);

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
LED_HandleTypeDef hled1;

extern USBD_GS_CAN_HandleTypeDef hGS_CAN;
extern TIM_HandleTypeDef htim2;
extern QueueHandle_t queue_to_hostHandle;

static TaskHandle_t xCreatedMyProgramTask;
static bool host_channel_is_active;

static void task_my_program(void *argument);

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/** Configure pins
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_K1_Pin */
  GPIO_InitStruct.Pin = USER_BTN_K1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BTN_K1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BTN_K2_Pin */
  GPIO_InitStruct.Pin = USER_BTN_K2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BTN_K2_GPIO_Port, &GPIO_InitStruct);
}

static void task_my_program(void *argument)
{
  UNUSED(argument);
  
  uint8_t prev_pressed_state = GPIO_PIN_SET;

  /* Infinite loop */
  for(;;) {
    /* if GPIO is pressed then send*/
    if (HAL_GPIO_ReadPin(USER_BTN_K1_GPIO_Port, USER_BTN_K1_Pin) == GPIO_PIN_RESET && prev_pressed_state != GPIO_PIN_RESET) {
      prev_pressed_state = GPIO_PIN_RESET;
      struct GS_HOST_FRAME frame = {0};
      frame.can_dlc = 8;
      frame.can_id = 0x123;
      frame.channel = 0;
      frame.echo_id = 0xFFFFFFFF;
      frame.flags = 0;
      frame.data[0] = 0x11;
      frame.data[1] = 0x22;
      frame.data[2] = 0x34;
      frame.data[3] = 0x44;
      frame.data[4] = 0x55;
      frame.data[5] = 0x66;
      frame.data[6] = 0x77;
      frame.data[7] = 0x88;
      frame.timestamp_us = __HAL_TIM_GET_COUNTER(&htim2);
      if (host_channel_is_active) {
        xQueueSendToBack(queue_to_hostHandle, &frame, 0);
      }
    }
    
    if(HAL_GPIO_ReadPin(USER_BTN_K1_GPIO_Port, USER_BTN_K1_Pin) == GPIO_PIN_SET) {
      prev_pressed_state = GPIO_PIN_SET;
    }

    vTaskDelay(1);
  }
}

void main_init_cb(void)
{
  hGS_CAN.channels[0] = &hfdcan1;
  hGS_CAN.channels[1] = &hfdcan2;
  can_init(hGS_CAN.channels[0], FDCAN1);
  can_init(hGS_CAN.channels[1], FDCAN2);
  led_init(&hled1, LED1_GPIO_Port, LED1_Pin, LED_MODE_INACTIVE, LED_ACTIVE_LOW);
  host_channel_is_active = false;
}

void main_rtos_init_cb(void)
{
  xTaskCreate(task_my_program, "MyProgTask", TASK_MY_PROGRAM_STACK_SIZE, NULL,
              TASK_MY_PROGRAM_PRIORITY, &xCreatedMyProgramTask);  
}

void main_task_cb(void)
{
  /* update all the LEDs */
 led_update(&hled1);
}

void can_on_enable_cb(uint8_t channel)
{
  UNUSED(channel);
  led_set_active(&hled1);
  host_channel_is_active = true;
}

void can_on_disable_cb(uint8_t channel)
{
  UNUSED(channel);
  led_set_inactive(&hled1);
  host_channel_is_active = false;
}

void can_on_tx_cb(uint8_t channel, struct GS_HOST_FRAME *frame)
{
  UNUSED(channel);
  UNUSED(frame);
  led_indicate_rxtx(&hled1);
}

void can_on_rx_cb(uint8_t channel, struct GS_HOST_FRAME *frame)
{
  UNUSED(channel);
  UNUSED(frame);
  led_indicate_rxtx(&hled1);
}

void can_identify_cb(uint32_t do_identify)
{
  if (do_identify) {
    led_blink(&hled1, 1000);
  }
  else {
    led_restore_prev_mode(&hled1);
  }
}