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

#define BOARD_SYSMEM_RESET_VECTOR    0x1FFF0000

#define USBD_VID                     0x1D50
#define USBD_LANGID_STRING           1033
#define USBD_MANUFACTURER_STRING     (uint8_t*) "canable.io"
#define USBD_PID_FS                  0x606F
#define USBD_PRODUCT_STRING_FS       (uint8_t*) "canablev2"
#define USBD_CONFIGURATION_STRING    (uint8_t*) "gs_usb"
#define USBD_INTERFACE_STRING        (uint8_t*) "gs_usb interface"
#define DFU_INTERFACE_STRING        (uint8_t*) "canablev2 DFU interface"

#define FDCAN_SJW_INIT        1
#define FDCAN_BRP_INIT        8
#define FDCAN_TS1_INIT        13
#define FDCAN_TS2_INIT        2

#define FDCAN_DATA_SJW_INIT   4
#define FDCAN_DATA_BRP_INIT   2
#define FDCAN_DATA_TS1_INIT   15
#define DFCAN_DATA_TS2_INIT   4

#define CANFD_SUPPORT     1
#define CAN_NUM_CHANNELS  1
#define CAN_CLOCK_SPEED   48000000

#define QUEUE_SIZE_HOST_TO_DEV    32
#define QUEUE_SIZE_DEV_TO_HOST    32

#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA

void SystemClock_Config(void);
void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__ BOARD_H__ */

