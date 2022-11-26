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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

#define BOARD_SYSMEM_RESET_VECTOR    0x1FFFC400

#define USBD_VID                     0x1D50
#define USBD_LANGID_STRING           1033
#define USBD_MANUFACTURER_STRING     (uint8_t*) "cannette.io"
#define USBD_PID_FS                  0x606F
#define USBD_PRODUCT_STRING_FS       (uint8_t*) "cannette"
#define USBD_CONFIGURATION_STRING    (uint8_t*) "gs_usb"
#define USBD_INTERFACE_STRING        (uint8_t*) "gs_usb interface"
#define DFU_INTERFACE_STRING        (uint8_t*) "cannette DFU interface"

#define CAN_SJW_INIT        1
#define CAN_BRP_INIT        6
#define CAN_TS1_INIT        13
#define CAN_TS2_INIT        2

#define CAN_TIME_TRG_MODE_INIT      DISABLE
#define CAN_AUTO_BUS_OFF_INIT       ENABLE
#define CAN_AUTO_WAKE_UP_INIT       DISABLE
#define CAN_AUTO_RETX_INIT          ENABLE
#define CAN_RX_FIFO_LCKD_INIT       DISABLE
#define CAN_TX_FIFO_PRI_INIT        ENABLE
#define CAN_MODE_INIT               CAN_MODE_NORMAL

#define CAN_NUM_CHANNELS  1
#define CAN_CLOCK_SPEED   48000000

#define QUEUE_SIZE_HOST_TO_DEV    32
#define QUEUE_SIZE_DEV_TO_HOST    32

#define nSI86_EN_Pin GPIO_PIN_13
#define nSI86_EN_GPIO_Port GPIOC
#define CAN_nSTANDBY_Pin GPIO_PIN_14
#define CAN_nSTANDBY_GPIO_Port GPIOC
#define DCDC_EN_Pin GPIO_PIN_15
#define DCDC_EN_GPIO_Port GPIOC
#define SENSE_5V_Pin GPIO_PIN_5
#define SENSE_5V_GPIO_Port GPIOA
#define LED_TX_Pin GPIO_PIN_8
#define LED_TX_GPIO_Port GPIOA
#define LED_RX_Pin GPIO_PIN_9
#define LED_RX_GPIO_Port GPIOA

void SystemClock_Config(void);
void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__ BOARD_H__ */

