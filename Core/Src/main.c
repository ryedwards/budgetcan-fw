/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "usbd_def.h"
#include "usbd_desc.h"
#include "usbd_core.h"
#include "board.h"
#include "usbd_gs_can.h"
#include "can.h"
#include "lin.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RESET_TO_BOOTLOADER_MAGIC_CODE 0xDEADBEEF

#define TASK_MAIN_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define TASK_MAIN_STACK_PRIORITY (tskIDLE_PRIORITY + 4)

#define TASK_Q_TO_HOST_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define TASK_Q_TO_HOST_STACK_PRIORITY (tskIDLE_PRIORITY + 3)

#define TASK_Q_FROM_HOST_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define TASK_Q_FROM_HOST_STACK_PRIORITY (tskIDLE_PRIORITY + 3)

#if !defined(BOARD_TIM2_PRESCALER)
  #define BOARD_TIM2_PRESCALER    0U
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static TaskHandle_t xCreatedMainTask;
static TaskHandle_t xCreatedQtoHostTask;
static TaskHandle_t xCreatedQfromHostTask;

#if defined (USE_MULTICHANNEL_QUEUE)
static QueueSetHandle_t xFromHostQueueSet;
static QueueSetMemberHandle_t xActivatedMember;
#endif

USBD_HandleTypeDef hUSB;
USBD_GS_CAN_HandleTypeDef hGS_CAN;

uint32_t dfu_reset_to_bootloader_magic;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
extern void SystemClock_Config(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void task_main(void *argument);
static void task_queue_to_host(void *argument);
static void task_queue_from_host(void *argument);
static void dfu_run_bootloader(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Function to allow the board to init any features */
  main_init_cb();

  /* Initialize the USB device */
  USBD_Init(&hUSB, (USBD_DescriptorsTypeDef*)&FS_Desc, DEVICE_FS);
  USBD_RegisterClass(&hUSB, &USBD_GS_CAN);
  USBD_GS_CAN_Init(&hUSB, &hGS_CAN);
  USBD_Start(&hUSB);

  /* Init the RTOS tasks */
  main_rtos_init_cb();

  xTaskCreate(task_main, "Main Task", TASK_MAIN_STACK_SIZE, NULL,
              TASK_MAIN_STACK_PRIORITY, &xCreatedMainTask);
  xTaskCreate(task_queue_to_host, "QtoHost", TASK_Q_TO_HOST_STACK_SIZE, NULL,
              TASK_Q_TO_HOST_STACK_PRIORITY, &xCreatedQtoHostTask);
  xTaskCreate(task_queue_from_host, "QfromHost", TASK_Q_FROM_HOST_STACK_SIZE, NULL,
              TASK_Q_FROM_HOST_STACK_PRIORITY, &xCreatedQfromHostTask);

  /* Init the RTOS streams and queues */
#if defined (USE_MULTICHANNEL_QUEUE)
  xFromHostQueueSet = xQueueCreateSet(QUEUE_SIZE_DEV_TO_HOST * CAN_NUM_CHANNELS);
  for(uint8_t i=0; i<CAN_NUM_CHANNELS; i++) {
    hGS_CAN.queue_from_hostHandle[i] = xQueueCreate(QUEUE_SIZE_HOST_TO_DEV, GS_HOST_FRAME_SIZE);
    xQueueAddToSet(hGS_CAN.queue_from_hostHandle[i], xFromHostQueueSet);
  }
#else
  hGS_CAN.queue_from_hostHandle = xQueueCreate(QUEUE_SIZE_HOST_TO_DEV, GS_HOST_FRAME_SIZE);
#endif
  hGS_CAN.queue_to_hostHandle = xQueueCreate(QUEUE_SIZE_DEV_TO_HOST, GS_HOST_FRAME_SIZE);

  /* Start scheduler */
  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = BOARD_TIM2_PRESCALER;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END TIM2_Init 2 */

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Function implementing the task_main thread.
  * @param  argument: Not used
  * @retval None
  */
void task_main(void *argument)
{
  UNUSED(argument);

  /* Infinite loop */
  for(;;)
  {
    main_task_cb();

    /* check for DFU update flag and kick to bootloader if set */
    if (USBD_GS_CAN_DfuDetachRequested(&hUSB)) {
      dfu_run_bootloader();
    }

    /* sleep 1ms to allow for idle */
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/**
  * @brief  Function implementing thread for handling messages FROM host.
  * @param  argument: Not used
  * @retval None
  */
void task_queue_from_host(void *argument)
{
  UNUSED(argument);
  struct gs_host_frame_object frame_object;

  /* Infinite loop */
  for(;;)
  {
#if defined (USE_MULTICHANNEL_QUEUE)
    /* wait for a queue member to become active */
    xActivatedMember = xQueueSelectFromSet(xFromHostQueueSet, portMAX_DELAY);
    xQueueReceive(xActivatedMember, &frame_object.frame, 0); 
#else
    xQueueReceive(hGS_CAN.queue_from_hostHandle, &frame_object.frame, portMAX_DELAY);
#endif

#if defined(LIN_FEATURE_ENABLED)
    if (IS_LIN_FRAME(frame_object.frame.can_id)) {
      lin_process_frame(&frame_object.frame);
      continue; /* just loop again so this frame isn't sent on CAN bus */
    }
#endif /* LIN_FEATURE_ENABLED */
    if (can_send(hGS_CAN.channels[frame_object.frame.channel], &frame_object.frame)) {
      /* Echo sent frame back to host */
      frame_object.frame.reserved = 0x0;
      xQueueSendToBack(hGS_CAN.queue_to_hostHandle, &frame_object.frame, 0);
      can_on_tx_cb(frame_object.frame.channel, &frame_object.frame);
    }
    else {
#if defined (USE_MULTICHANNEL_QUEUE)
        xQueueSendToFront(hGS_CAN.queue_from_hostHandle[frame_object.frame.channel], &frame_object.frame, 0);
#else
        xQueueSendToFront(hGS_CAN.queue_from_hostHandle, &frame_object.frame, 0);
#endif
    } 
  }
  vTaskDelay(pdMS_TO_TICKS(0));  
}


/**
  * @brief  Function implementing thread for handling messages TO host.
  * @param  argument: Not used
  * @retval None
  */
void task_queue_to_host(void *argument)
{
  UNUSED(argument);
  struct gs_host_frame_object frame_object;
    
    /* Infinite loop */
  for(;;)
  { 
    /* Check the queue to see if we have data TO the host to handle */
    if (xQueueReceive(hGS_CAN.queue_to_hostHandle, &frame_object.frame, portMAX_DELAY) == pdPASS) {
      if (USBD_GS_CAN_SendFrame(&hUSB, &frame_object.frame) == USBD_OK) {
        can_on_rx_cb(frame_object.frame.channel, &frame_object.frame);
      }
      else {
        /* throw the message back onto the queue */
        if (uxQueueSpacesAvailable(hGS_CAN.queue_to_hostHandle) == 0) {
          /* the pipe to the host is full - delay longer to allow for catchup if needed */
          vTaskDelay(pdMS_TO_TICKS(10));
        }
        else {
          xQueueSendToFront(hGS_CAN.queue_to_hostHandle, &frame_object.frame, 0);
          vTaskDelay(pdMS_TO_TICKS(0));
        }
      }
    }
  }
}

/*
 * Functions for handling the transition to bootloader
 */
void __initialize_hardware_early(void)
{
  if (dfu_reset_to_bootloader_magic == RESET_TO_BOOTLOADER_MAGIC_CODE) {
    dfu_reset_to_bootloader_magic = 0;
    typedef void (*pFunction)(void);
    uint32_t JumpAddress = *(__IO uint32_t*) (BOARD_SYSMEM_RESET_VECTOR + 4);
    pFunction Jump_To_Boot = (pFunction) JumpAddress;
    __set_MSP(*(__IO uint32_t*) BOARD_SYSMEM_RESET_VECTOR);
    Jump_To_Boot();
    while (42);
    }

    else {
        /* Do nothing - fall through and continue normal init */
    }
}

void dfu_run_bootloader(void)
{
  dfu_reset_to_bootloader_magic = RESET_TO_BOOTLOADER_MAGIC_CODE;
  NVIC_SystemReset();
}

/* weak function calls to allow callbacks to be optionally included */

/** @brief Callback function called during main init
 *  @param None
 *  @retval None
 */
__weak void main_init_cb(void)
{
}
/** @brief Callback function called during main rtos init
 *  @param None
 *  @retval None
 */
__weak void main_rtos_init_cb(void)
{
}
/** @brief Callback function called periodically during the main task
 *  @param None
 *  @retval None
 */
__weak void main_task_cb(void)
{
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
