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
#include "task.h"
#include "can.h"
#include "lin.h"
#include "led.h"

#define TASK_LIN_STACK_SIZE (512 / sizeof(portSTACK_TYPE))
#define TASK_LIN_STACK_PRIORITY (tskIDLE_PRIORITY + 1)

extern void main_usbd_gs_can_set_channel_cb(USBD_HandleTypeDef *hUSB);

LED_HandleTypeDef hled1;
LED_HandleTypeDef hled2;
LED_HandleTypeDef hled3;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

LIN_HandleTypeDef hlin1;

UART_HandleTypeDef huart1;

static TaskHandle_t xCreatedLINTask;

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FDCAN1_SLEEP_EN_Pin|FDCAN1_TERM_EN_Pin|FDCAN2_SLEEP_EN_Pin|FDCAN2_TERM_EN_Pin
                          |SWCAN_M0_Pin|SWCAN_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIN1_NSLP_GPIO_Port, LIN1_NSLP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FDCAN1_SLEEP_EN_Pin FDCAN1_TERM_EN_Pin FDCAN2_SLEEP_EN_Pin FDCAN2_TERM_EN_Pin
                           SWCAN_M0_Pin SWCAN_M1_Pin */
  GPIO_InitStruct.Pin = FDCAN1_SLEEP_EN_Pin|FDCAN1_TERM_EN_Pin|FDCAN2_SLEEP_EN_Pin|FDCAN2_TERM_EN_Pin
                          |SWCAN_M0_Pin|SWCAN_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LIN1_NSLP_Pin */
  GPIO_InitStruct.Pin = LIN1_NSLP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIN1_NSLP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 10417;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_LIN_Init(&huart1, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** @brief UART RX IRQ for this board
 *  @param None
 *  @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    lin_handle_uart_rx_IRQ(&hlin1);
    //set up for next RX for USART1
    HAL_UART_Receive_IT(&huart1, hlin1.UartRxBuffer, 1);
  }
}

/** @brief Function to run the LIN task
 *  @param None
 *  @retval None
 */
void task_lin(void *argument)
{
  UNUSED(argument);
  /* Infinite loop */
  for(;;) {
    /* LIN transmits in blocking mode so put into this task */
    lin_handler_task(&hlin1);

    vTaskDelay(1);
  }
}

/** @brief Function to init any features specific to this board
 *  @param None
 *  @retval None
 */
void main_init_cb(void)
{
  can_init(&hfdcan1, FDCAN1);
  can_init(&hfdcan2, FDCAN2);

  led_init(&hled1, LED1_GPIO_Port, LED1_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
  led_init(&hled2, LED2_GPIO_Port, LED2_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
  led_init(&hled3, LED3_GPIO_Port, LED3_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);

  MX_USART1_UART_Init();
  lin_init(&hlin1, LIN1_CHANNEL, &huart1);
  HAL_GPIO_WritePin(LIN1_NSLP_GPIO_Port, LIN1_NSLP_Pin, GPIO_PIN_SET);
  xTaskCreate(task_lin, "LIN Task", TASK_LIN_STACK_SIZE, NULL,
                TASK_LIN_STACK_PRIORITY, &xCreatedLINTask);
}

/** @brief Function to assign the CAN HW pointers to the channel index in the USB handle
 *  @param USBD_HandleTypeDef *hUSB - The handle for the USB where will will set up the CAN pointer
 *  @retval None
 */
void main_usbd_gs_can_set_channel_cb(USBD_HandleTypeDef *hUSB)
{
  USBD_GS_CAN_SetChannel(hUSB, 0, &hfdcan1);
  USBD_GS_CAN_SetChannel(hUSB, 1, &hfdcan2);
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
  led_update(&hled3);
}

/** @brief Function called when the CAN is enabled for this channel
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval None
 */
void can_on_enable_cb(uint8_t channel)
{
  if (channel == 0) {
    HAL_GPIO_WritePin(FDCAN1_SLEEP_EN_GPIO_Port, FDCAN1_SLEEP_EN_Pin, GPIO_PIN_RESET);
  }
  if (channel == 1) {
    HAL_GPIO_WritePin(FDCAN2_SLEEP_EN_GPIO_Port, FDCAN2_SLEEP_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SWCAN_M0_GPIO_Port, SWCAN_M0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SWCAN_M1_GPIO_Port, SWCAN_M1_Pin, GPIO_PIN_SET);
  }
  led_set_active(&hled3);
}

/** @brief Function called when the CAN is disabled for this channel
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval None
 */
void can_on_disable_cb(uint8_t channel)
{
  if (channel == 0) {
    HAL_GPIO_WritePin(FDCAN1_SLEEP_EN_GPIO_Port, FDCAN1_SLEEP_EN_Pin, GPIO_PIN_SET);
  }
  if (channel == 1) {
    HAL_GPIO_WritePin(FDCAN2_SLEEP_EN_GPIO_Port, FDCAN2_SLEEP_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SWCAN_M0_GPIO_Port, SWCAN_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SWCAN_M1_GPIO_Port, SWCAN_M1_Pin, GPIO_PIN_RESET);
  }
  led_set_inactive(&hled3);
}

/** @brief Function called when a CAN frame is send on this channel
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval None
 */
void can_on_tx_cb(uint8_t channel, struct GS_HOST_FRAME *frame)
{
  UNUSED(channel);
  UNUSED(frame);
  led_indicate_rxtx(&hled1);
}

/** @brief Function called when a CAN frame is received on this channel
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval None
 */
void can_on_rx_cb(uint8_t channel, struct GS_HOST_FRAME *frame)
{
  UNUSED(channel);
  UNUSED(frame);
  led_indicate_rxtx(&hled2);
}

/** @brief Function called to set the CAN termination resistor ON of OFF
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @param GPIO_PinState state - The requested state of the pin
 *  @retval None
 */
void can_set_term_cb(uint8_t channel, GPIO_PinState state)
{
  if (channel == 0) {
    HAL_GPIO_WritePin(FDCAN1_TERM_EN_GPIO_Port, FDCAN1_TERM_EN_Pin, state);
  }
  if (channel == 1) {
    HAL_GPIO_WritePin(FDCAN2_TERM_EN_GPIO_Port, FDCAN2_TERM_EN_Pin, state);
  }
}

/** @brief Function called to get the state of the CAN termination resistor
 *  @param uint8_t channel - The CAN channel (0 based)
 *  @retval The current state of the CAN termination pin
 */
GPIO_PinState can_get_term_cb(uint8_t channel)
{
  if (channel == 0) {
    return HAL_GPIO_ReadPin(FDCAN1_TERM_EN_GPIO_Port, FDCAN1_TERM_EN_Pin);
  }
  if (channel == 1) {
    return HAL_GPIO_ReadPin(FDCAN2_TERM_EN_GPIO_Port, FDCAN2_TERM_EN_Pin);
  }
  return 0;
}

