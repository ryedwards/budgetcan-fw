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
#include "usbd_gs_can.h"
#include "can.h"
#include "lin.h"
#include "led.h"

#define TASK_LIN_STACK_SIZE		(configMINIMAL_STACK_SIZE)
#define TASK_LIN_STACK_PRIORITY (tskIDLE_PRIORITY + 3)

LED_HandleTypeDef hled1;
LED_HandleTypeDef hled2;
LED_HandleTypeDef hled3;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

extern USBD_GS_CAN_HandleTypeDef hGS_CAN;

LIN_HandleTypeDef hlin1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

static TaskHandle_t xCreatedLINTask;

static void task_lin(void *argument);

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, FDCAN1_SLEEP_EN_Pin|FDCAN1_TERM_EN_Pin|FDCAN2_SLEEP_EN_Pin|FDCAN2_TERM_EN_Pin
					  |FDCAN3_SLEEP_EN_Pin|FDCAN3_TERM_EN_Pin|LIN1_MASTER_EN_Pin|FDCAN1_VIO_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LIN1_nSLEEP_GPIO_Port, LIN1_nSLEEP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
	GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : FDCAN1_SLEEP_EN_Pin FDCAN1_TERM_EN_Pin FDCAN2_SLEEP_EN_Pin FDCAN2_TERM_EN_Pin
	                         FDCAN3_SLEEP_EN_Pin FDCAN3_TERM_EN_Pin LIN1_MASTER_EN_Pin FDCAN1_VIO_EN_Pin */
	GPIO_InitStruct.Pin = FDCAN1_SLEEP_EN_Pin|FDCAN1_TERM_EN_Pin|FDCAN2_SLEEP_EN_Pin|FDCAN2_TERM_EN_Pin
						  |FDCAN3_SLEEP_EN_Pin|FDCAN3_TERM_EN_Pin|LIN1_MASTER_EN_Pin|FDCAN1_VIO_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : TPS2116_STAT_Pin */
	GPIO_InitStruct.Pin = TPS2116_STAT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TPS2116_STAT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LIN1_nSLEEP_Pin */
	GPIO_InitStruct.Pin = LIN1_nSLEEP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LIN1_nSLEEP_GPIO_Port, &GPIO_InitStruct);

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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/** @brief UART RX IRQ for this board
 *  @param None
 *  @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		//TODO: Set this up for multichannel LIN
		lin_rx_IRQ_handler(&hlin1);
		//set up for next RX for USART1
		HAL_UART_Receive_IT(hlin1.huart, hlin1.UartRxBuffer, 1);
	}
}

/** @brief UART TX IRQ for this board
 *  @param None
 *  @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
}

/** @brief Function to run the LIN task
 *  @param None
 *  @retval None
 */
static void task_lin(void *argument)
{
	UNUSED(argument);
	/* Infinite loop */
	for (;;) {
		/* LIN transmits in blocking mode so put into this task */
		lin_handler_task(&hlin1);
		vTaskDelay(pdMS_TO_TICKS(0));
	}
}

LIN_HandleTypeDef* lin_get_handle(uint32_t msg_id)
{
	if ((msg_id & 0x1FFFFE80) == 0x1FFFFE80) {
		return &hlin1;
	}
	else {
		return NULL;
	}
}

void main_init_cb(void)
{

	for (uint8_t i=0; i<20; i++)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(50);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	}

	hGS_CAN.channels[0] = &hfdcan1;
	hGS_CAN.channels[1] = &hfdcan2;
	hGS_CAN.channels[2] = &hfdcan3;
	can_init(hGS_CAN.channels[0], FDCAN1);
	can_init(hGS_CAN.channels[1], FDCAN2);
	can_init(hGS_CAN.channels[2], FDCAN3);
	HAL_GPIO_WritePin(FDCAN1_VIO_EN_GPIO_Port, FDCAN1_VIO_EN_Pin, GPIO_PIN_SET);

	led_init(&hled1, LED1_GPIO_Port, LED1_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
	led_init(&hled2, LED2_GPIO_Port, LED2_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
	led_init(&hled3, LED3_GPIO_Port, LED3_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);

	/* init specific to LIN */
	MX_USART1_UART_Init();
	xTaskCreate(task_lin, "LIN Task", TASK_LIN_STACK_SIZE, NULL,
				TASK_LIN_STACK_PRIORITY, &xCreatedLINTask);
	lin_init(&hlin1, LIN1_CHANNEL, &huart1);
	HAL_GPIO_WritePin(LIN1_nSLEEP_GPIO_Port,	LIN1_nSLEEP_Pin,	GPIO_PIN_SET);
	HAL_GPIO_WritePin(LIN1_MASTER_EN_GPIO_Port, LIN1_MASTER_EN_Pin, GPIO_PIN_SET);

	/* Debug serial port */
	MX_USART2_UART_Init();
}

void main_task_cb(void)
{
	/* update all the LEDs */
	led_update(&hled1);
	led_update(&hled2);
	led_update(&hled3);
}

void can_on_enable_cb(uint8_t channel)
{
	if (channel == 0) {
		HAL_GPIO_WritePin(FDCAN1_SLEEP_EN_GPIO_Port, FDCAN1_SLEEP_EN_Pin, GPIO_PIN_RESET);
	}
	if (channel == 1) {
		HAL_GPIO_WritePin(FDCAN2_SLEEP_EN_GPIO_Port, FDCAN2_SLEEP_EN_Pin, GPIO_PIN_RESET);
	}
	if (channel == 2) {
		HAL_GPIO_WritePin(FDCAN3_SLEEP_EN_GPIO_Port, FDCAN3_SLEEP_EN_Pin, GPIO_PIN_RESET);
	}
	led_set_active(&hled3);
}

void can_on_disable_cb(uint8_t channel)
{
	if (channel == 0) {
		HAL_GPIO_WritePin(FDCAN1_SLEEP_EN_GPIO_Port, FDCAN1_SLEEP_EN_Pin, GPIO_PIN_SET);
	}
	if (channel == 1) {
		HAL_GPIO_WritePin(FDCAN2_SLEEP_EN_GPIO_Port, FDCAN2_SLEEP_EN_Pin, GPIO_PIN_SET);
	}
	if (channel == 2) {
		HAL_GPIO_WritePin(FDCAN3_SLEEP_EN_GPIO_Port, FDCAN3_SLEEP_EN_Pin, GPIO_PIN_SET);
	}
	led_set_inactive(&hled3);
}

void can_on_tx_cb(uint8_t channel, struct gs_host_frame *frame)
{
	UNUSED(channel);
	UNUSED(frame);
	led_indicate_rxtx(&hled1);
}

void can_on_rx_cb(uint8_t channel, struct gs_host_frame *frame)
{
	UNUSED(channel);
	UNUSED(frame);
	led_indicate_rxtx(&hled2);
}

void can_set_term_cb(uint8_t channel, GPIO_PinState state)
{
	if (channel == 0) {
		HAL_GPIO_WritePin(FDCAN1_TERM_EN_GPIO_Port, FDCAN1_TERM_EN_Pin, state);
	}
	if (channel == 1) {
		HAL_GPIO_WritePin(FDCAN2_TERM_EN_GPIO_Port, FDCAN2_TERM_EN_Pin, state);
	}
	if (channel == 2) {
		HAL_GPIO_WritePin(FDCAN3_TERM_EN_GPIO_Port, FDCAN3_TERM_EN_Pin, state);
	}
}

GPIO_PinState can_get_term_cb(uint8_t channel)
{
	if (channel == 0) {
		return HAL_GPIO_ReadPin(FDCAN1_TERM_EN_GPIO_Port, FDCAN1_TERM_EN_Pin);
	}
	if (channel == 1) {
		return HAL_GPIO_ReadPin(FDCAN2_TERM_EN_GPIO_Port, FDCAN2_TERM_EN_Pin);
	}
	if (channel == 2) {
		return HAL_GPIO_ReadPin(FDCAN3_TERM_EN_GPIO_Port, FDCAN3_TERM_EN_Pin);
	}
	return 0;
}

void can_identify_cb(uint32_t do_identify)
{
	if (do_identify) {
		led_blink_start(&hled3, 250);
	}
	else {
		led_blink_stop(&hled3);
	}
}

