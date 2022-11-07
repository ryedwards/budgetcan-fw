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
#include "can.h"
#include "led.h"

LED_HandleTypeDef hled1;

FDCAN_HandleTypeDef hfdcan1;

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

   /*Configure GPIO pin : LED1_Pin */
   GPIO_InitStruct.Pin = LED1_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);
}

/** @brief Function to init all of the LEDs that this board supports
 *  @param None
 *  @retval None
 */
void board_led_init(void)
{
  led_init(&hled1, LED1_GPIO_Port, LED1_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
}

/** @brief Function to init all of the CAN channels this board supports
 *  @param None
 *  @retval None
 */
void board_can_init(void)
{
  can_init(&hfdcan1, FDCAN1);
}

/** @brief Function to assign the CAN HW pointers to the channel index in the USB handle
 *  @param USBD_HandleTypeDef *hUSB - The handle for the USB where will will set up the CAN pointer
 *  @retval None
 */
void board_usbd_gs_can_set_channel(USBD_HandleTypeDef *hUSB)
{
  USBD_GS_CAN_SetChannel(hUSB, 0, &hfdcan1);
}

/** @brief Function to periodically update the LED states - should be called as fast as possible
 *  @param None
 *  @retval None
 */
void board_led_update(void)
{
  /* update all the LEDs */
 led_update(&hled1);
}

/** @brief Function called when the CAN is enabled for this channel
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval None
 */
void board_on_can_enable_cb(uint8_t channel)
{
  UNUSED(channel);
}

/** @brief Function called when the CAN is disabled for this channel
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval None
 */
void board_on_can_disable_cb(uint8_t channel)
{
  UNUSED(channel);
}

/** @brief Function called when a CAN frame is send on this channel
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval None
 */
void board_on_can_tx_cb(uint8_t channel)
{
  UNUSED(channel);
  led_indicate_rxtx(&hled1);
}

/** @brief Function called when a CAN frame is received on this channel
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval None
 */
void board_on_can_rx_cb(uint8_t channel)
{
  UNUSED(channel);
  led_indicate_rxtx(&hled1);
}

/** @brief Function called to set the CAN termination resistor ON of OFF
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @param GPIO_PinState state - The requested state of the pin
 *  @retval None
 */
void board_set_can_term(uint8_t channel, GPIO_PinState state)
{
  UNUSED(channel);
}

/** @brief Function called to get the state of the CAN termination resistor
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval The current state of the CAN termination pin
 */
GPIO_PinState board_get_can_term(uint8_t channel)
{
  UNUSED(channel);
  return 0;
}
