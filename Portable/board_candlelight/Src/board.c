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
#include "stream_buffer.h"
#include "usbd_gs_can.h"
#include "can.h"
#include "led.h"

#define IS_IRQ_MODE()             ( (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0)

#define STREAM_BUFFER_UART1_RX_SIZEBYTES     100U
#define STREAM_BUFFER_UART1_RX_TRIGGERLEVEL  10U
#define STREAM_BUFFER_UART1_TX_SIZEBYTES     100U
#define STREAM_BUFFER_UART1_TX_TRIGGERLEVEL  10U

static StreamBufferHandle_t stream_buffer_uart1_rx;
static StreamBufferHandle_t stream_buffer_uart1_tx;

LED_HandleTypeDef hledrx;
LED_HandleTypeDef hledtx;

UART_HandleTypeDef huart1;
CAN_HandleTypeDef hcan;

uint8_t uart1_rx_buffer[1] = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_nSTANDBY_GPIO_Port, CAN_nSTANDBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RX_Pin|LED_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CAN_nSTANDBY_Pin */
  GPIO_InitStruct.Pin = CAN_nSTANDBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_nSTANDBY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RX_Pin LED_TX_Pin */
  GPIO_InitStruct.Pin = LED_RX_Pin|LED_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * UART IRQ CALLBACKS
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t uart1_tx_buffer[16]; /* only send up to 16 bytes at a time */
  if (huart->Instance == USART1) {
    if (!xStreamBufferIsEmpty(stream_buffer_uart1_tx)) {
      /* there is data in the stream buffer that needs to be TX'd */
      uint8_t num_bytes = xStreamBufferReceiveFromISR(stream_buffer_uart1_tx, uart1_tx_buffer, sizeof(uart1_tx_buffer), NULL);
      HAL_UART_Transmit_IT(huart, uart1_tx_buffer, num_bytes);
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* only read 1 byte at a time since we don't have a defined data type */
  if (huart->Instance == USART1) {
    xStreamBufferSendFromISR(stream_buffer_uart1_rx, uart1_rx_buffer, 1, NULL);
    /* Set up for the next RX byte */
    HAL_UART_Receive_IT(huart, &uart1_rx_buffer[0], 1);
  }
}

int __io_putchar(int ch)
{
  if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)&ch, 1) == HAL_BUSY) {
    if (IS_IRQ_MODE()) {
      xStreamBufferSendFromISR(stream_buffer_uart1_tx,(uint8_t *)&ch, 1, NULL);
    }
    else {
      xStreamBufferSend(stream_buffer_uart1_tx,(uint8_t *)&ch, 1, 0);
    }
  }
  return ch;
}

void main_init_cb(void)
{
  MX_USART1_UART_Init();

  /* carry over the LED blink from original firmware */
  for (uint8_t i=0; i<10; i++)
	{
		HAL_GPIO_TogglePin(LED_RX_GPIO_Port, LED_RX_Pin);
		HAL_Delay(50);
		HAL_GPIO_TogglePin(LED_TX_GPIO_Port, LED_TX_Pin);
	}
  
  hGS_CAN.channels[0] = &hcan;
  can_init(hGS_CAN.channels[0], CAN);

  led_init(&hledrx, LED_RX_GPIO_Port, LED_RX_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
  led_init(&hledtx, LED_TX_GPIO_Port, LED_TX_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);

  HAL_UART_Receive_IT(&huart1, &uart1_rx_buffer[0], 1);
}

void main_rtos_init_cb(void)
{
  stream_buffer_uart1_rx = xStreamBufferCreate(STREAM_BUFFER_UART1_RX_SIZEBYTES,
                                                  STREAM_BUFFER_UART1_RX_TRIGGERLEVEL);
  stream_buffer_uart1_tx = xStreamBufferCreate(STREAM_BUFFER_UART1_TX_SIZEBYTES,
                                                STREAM_BUFFER_UART1_TX_TRIGGERLEVEL);
}

void main_task_cb(void)
{
  /* update all the LEDs */
  led_update(&hledrx);
  led_update(&hledtx);
}

void can_on_enable_cb(uint8_t channel)
{
  UNUSED(channel);
  HAL_GPIO_WritePin(CAN_nSTANDBY_GPIO_Port, CAN_nSTANDBY_Pin, GPIO_PIN_RESET);
  printf("CAN enabled");
}

void can_on_disable_cb(uint8_t channel)
{
  UNUSED(channel);
  HAL_GPIO_WritePin(CAN_nSTANDBY_GPIO_Port, CAN_nSTANDBY_Pin, GPIO_PIN_SET);
  printf("CAN diabled");
}

void can_on_tx_cb(uint8_t channel, struct GS_HOST_FRAME *frame)
{
  UNUSED(channel);
  UNUSED(frame);
  led_indicate_rxtx(&hledtx);
}

void can_on_rx_cb(uint8_t channel, struct GS_HOST_FRAME *frame)
{
  UNUSED(channel);
  UNUSED(frame);
  led_indicate_rxtx(&hledrx);
}