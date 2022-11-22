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
#include "usbd_gs_can.h"
#include "can.h"
#include "led.h"

LED_HandleTypeDef hled1;
LED_HandleTypeDef hled2;

CAN_HandleTypeDef hcan;

extern USBD_GS_CAN_HandleTypeDef hGS_CAN;

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/** @brief Function to init all of the LEDs that this board supports
 *  @param None
 *  @retval None
 */
void main_init_cb(void)
{
  /* carry over the LED blink from original firmware */
  for (uint8_t i=0; i<10; i++)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(50);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	}
  
  hGS_CAN.channels[0] = &hcan;
  can_init(hGS_CAN.channels[0], CAN);

  led_init(&hled1, LED1_GPIO_Port, LED1_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
  led_init(&hled2, LED2_GPIO_Port, LED2_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);

}

/** @brief Function to periodically update any features on the board from the main task
 *  @param None
 *  @retval None
 */
void main_task_cb(void)
{
  /* update all the LEDs */
  led_update(&hled1);
  led_update(&hled2);
}

/** @brief Function called when the CAN is enabled for this channel
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval None
 */
void can_on_enable_cb(uint8_t channel)
{
  UNUSED(channel);
  led_set_active(&hled2);
}

/** @brief Function called when the CAN is disabled for this channel
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval None
 */
void can_on_disable_cb(uint8_t channel)
{
  UNUSED(channel);
  led_set_inactive(&hled2);
}

/** @brief Function called when a CAN frame is send on this channel
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval None
 */
void can_on_tx_cb(uint8_t channel, struct gs_host_frame *frame)
{
  UNUSED(channel);
  UNUSED(frame);
  led_indicate_rxtx(&hled1);
}

/** @brief Function called when a CAN frame is received on this channel
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval None
 */
void can_on_rx_cb(uint8_t channel, struct gs_host_frame *frame)
{
  UNUSED(channel);
  UNUSED(frame);
  led_indicate_rxtx(&hled1);
}