/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair
Copyright (c) 2022 Ryan Edwards (changes for STM32G0/G4 and CAN-FD)

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
#ifndef __USBD_GS_CAN_H
#define __USBD_GS_CAN_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdbool.h>
#include "main.h"
#include "board.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "usbd_def.h"
#include "gs_usb.h"
#include "can.h"

/* Exported defines -----------------------------------------------------------*/
#define CAN_DATA_MAX_PACKET_SIZE 64     /* Endpoint IN & OUT Packet size */
#define CAN_CMD_PACKET_SIZE		 72     /* Control Endpoint Packet size */
#define USB_CAN_CONFIG_DESC_SIZ	 50
#define USBD_GS_CAN_VENDOR_CODE	 0x20
#define DFU_INTERFACE_NUM		 1
#define DFU_INTERFACE_STR_INDEX	 0xE0

extern USBD_ClassTypeDef USBD_GS_CAN;


#if defined(CANFD_FEATURE_ENABLED)
#if !defined(FDCAN1)
#error This CAN type does not support CANFD!!!
#endif
#define GS_HOST_FRAME_SIZE		   struct_size((struct gs_host_frame *)NULL, canfd_ts, 1)
#define GS_HOST_CLASSIC_FRAME_SIZE struct_size((struct gs_host_frame *)NULL, classic_can_ts, 1)
#else
#define GS_HOST_FRAME_SIZE		   struct_size((struct gs_host_frame *)NULL, classic_can_ts, 1)
#endif

struct gs_host_frame_object {
	union {
		uint8_t _buf[GS_HOST_FRAME_SIZE];
		struct gs_host_frame frame;
	};
};

/* Exported types ------------------------------------------------------------*/
typedef struct {
	uint8_t ep0_buf[CAN_CMD_PACKET_SIZE];
	__IO uint32_t TxState;
	USBD_SetupReqTypedef last_setup_request;
	struct gs_host_frame_object from_host_frame;
	CAN_HANDLE_TYPEDEF *channels[CAN_NUM_CHANNELS];
	QueueHandle_t queue_from_hostHandle;
	QueueHandle_t queue_to_hostHandle;
	bool dfu_detach_requested;
	bool pad_pkts_to_max_pkt_size;
	bool timestamps_enabled;
	uint32_t sof_timestamp_us;
} USBD_GS_CAN_HandleTypeDef __attribute__ ((aligned (4)));

/* Exported functions --------------------------------------------------------*/
uint8_t USBD_GS_CAN_Init(USBD_HandleTypeDef *pdev, USBD_GS_CAN_HandleTypeDef *hcan);
uint8_t USBD_GS_CAN_GetChannelNumber(USBD_HandleTypeDef *pdev, CAN_HANDLE_TYPEDEF* handle);
bool USBD_GS_CAN_CustomDeviceRequest(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
bool USBD_GS_CAN_CustomInterfaceRequest(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
bool USBD_GS_CAN_DfuDetachRequested(USBD_HandleTypeDef *pdev);
uint8_t USBD_GS_CAN_SendFrame(USBD_HandleTypeDef *pdev, struct gs_host_frame *frame);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_GS_CAN_H */
