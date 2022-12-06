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

#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "board.h"
#include "can.h"
#include "led.h"
#include "usbd_gs_can.h"

static uint32_t can_last_err_status;

extern TIM_HandleTypeDef htim2;
extern USBD_HandleTypeDef hUSB;
extern USBD_GS_CAN_HandleTypeDef hGS_CAN;

static bool can_parse_error_status(uint32_t err, uint32_t last_err, CAN_HANDLE_TYPEDEF *hcan, struct gs_host_frame *frame);

/** @brief Function to initialize the FDCAN handle
 *  @param FDCAN_HandleTypeDef *hcan - The pointer to the handle.
 *  @param FDCAN_GlobalTypeDef *instance - pointer to the instance name.
 *  @retval None
 */
void can_init(CAN_HANDLE_TYPEDEF *hcan, CAN_TYPEDEF *instance)
{
#if defined(CAN)
	hcan->Instance = instance;
	hcan->Init.TimeTriggeredMode = CAN_TIME_TRG_MODE_INIT;
	hcan->Init.AutoBusOff = CAN_AUTO_BUS_OFF_INIT;
	hcan->Init.AutoWakeUp = CAN_AUTO_WAKE_UP_INIT;
	hcan->Init.AutoRetransmission = CAN_AUTO_RETX_INIT;
	hcan->Init.ReceiveFifoLocked = CAN_RX_FIFO_LCKD_INIT;
	hcan->Init.TransmitFifoPriority = CAN_TX_FIFO_PRI_INIT;
	hcan->Init.Mode = CAN_MODE_INIT;

	/* all values for the bxCAN init are -1 and shifted */
	hcan->Init.SyncJumpWidth = ((CAN_SJW_INIT)-1) << CAN_BTR_SJW_Pos;
	hcan->Init.Prescaler = ((CAN_BRP_INIT)-1);
	hcan->Init.TimeSeg1 = ((CAN_TS1_INIT)-1) << CAN_BTR_TS1_Pos;
	hcan->Init.TimeSeg2 = ((CAN_TS2_INIT)-1) << CAN_BTR_TS2_Pos;

	HAL_CAN_Init(hcan);

#elif defined(FDCAN1)
	hcan->Instance = instance;
	hcan->Init.FrameFormat = FDCAN_FRAME_FMT_INIT;
	hcan->Init.Mode = FDCAN_MODE_INIT;
	hcan->Init.AutoRetransmission = FDCAN_AUTO_RETX_INIT;
	hcan->Init.TransmitPause = FDCAN_AUTO_TX_PAUSE_INIT;
	hcan->Init.ProtocolException = FDCAN_PROT_EXCPTN_INIT;
	hcan->Init.NominalPrescaler = FDCAN_BRP_INIT;
	hcan->Init.NominalSyncJumpWidth = FDCAN_SJW_INIT;
	hcan->Init.NominalTimeSeg1 = FDCAN_TS1_INIT;
	hcan->Init.NominalTimeSeg2 = FDCAN_TS2_INIT;
	hcan->Init.DataPrescaler = FDCAN_DATA_BRP_INIT;
	hcan->Init.DataSyncJumpWidth = FDCAN_DATA_SJW_INIT;
	hcan->Init.DataTimeSeg1 = FDCAN_DATA_TS1_INIT;
	hcan->Init.DataTimeSeg2 = FDCAN_DATA_TS2_INIT;
	hcan->Init.StdFiltersNbr = FDCAN_STD_FLTR_NUM_INIT;
	hcan->Init.ExtFiltersNbr = FDCAN_EXT_FLTR_NUM_INIT;
	hcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION_INIT;
#if defined(STM32H7)
/* The STM32H7xx has a slightly different FDCAN implementation
  The clock divider was removed? */
	hcan->Init.MessageRAMOffset = FDCAN_MSG_RAM_OFST_INIT;
	hcan->Init.RxFifo0ElmtsNbr = FDCAN_RX_FIFO0_ELEM_NUM_INIT;
	hcan->Init.RxFifo0ElmtSize = FDCAN_RX_FIFO0_ELEM_SZ_INIT;
	hcan->Init.RxFifo1ElmtsNbr = FDCAN_RX_FIFO1_ELEM_NUM_INIT;
	hcan->Init.RxFifo1ElmtSize = FDCAN_RX_FIFO1_ELEM_SZ_INIT;
	hcan->Init.RxBuffersNbr = FDCAN_RX_BUFF_NUM_INIT;
	hcan->Init.RxBufferSize = FDCAN_RX_BUFF_SZ_INIT;
	hcan->Init.TxEventsNbr = FDCAN_TX_EVNT_NUM_INIT;
	hcan->Init.TxBuffersNbr = FDCAN_TX_BUFF_NUM_INIT;
	hcan->Init.TxFifoQueueElmtsNbr = FDCAN_TX_FIFO_ELEM_NUM_INIT;
	hcan->Init.TxElmtSize = FDCAN_TX_FIFO_ELEM_SZ_INIT;
#else
	hcan->Init.ClockDivider = FDCAN_CLOCK_DIV_INIT;
#endif
	HAL_FDCAN_Init(hcan);
#endif
}

/** @brief Function to set the CAN bit timing registers - will not set until can_enable() is executed
 *  @param FDCAN_HandleTypeDef *hcan - The pointer to the handle.
 *  @param uint16_t brp - The prescale value
 *  @param uint16_t phase_seg1 - The time phase segment 1 value
 *  @param uint16_t phase_seg2 - The time phase segment 2 value
 *  @param uint8_t sjw - The sync jump width value
 *  @retval None
 */
void can_set_bittiming(CAN_HANDLE_TYPEDEF *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
	if (  (brp > 0) && (brp <= 1024)
	   && (phase_seg1 > 0) && (phase_seg1 <= 16)
	   && (phase_seg2 > 0) && (phase_seg2 <= 8)
	   && (sjw > 0) && (sjw <= 4))
	{
#if defined(CAN)
		hcan->Init.SyncJumpWidth = (sjw-1) << CAN_BTR_SJW_Pos;
		hcan->Init.TimeSeg1 = (phase_seg1-1) << CAN_BTR_TS1_Pos;
		hcan->Init.TimeSeg2 = (phase_seg2-1) << CAN_BTR_TS2_Pos;;
		hcan->Init.Prescaler = brp;
#elif defined(FDCAN1)
		hcan->Init.NominalPrescaler = brp;
		hcan->Init.NominalTimeSeg1 = phase_seg1;
		hcan->Init.NominalTimeSeg2 = phase_seg2;
		hcan->Init.NominalSyncJumpWidth = sjw;
#endif
	}
}

/** @brief Function to set the CAN-FD (data) bit timing registers - will not set until can_enable() is executed
 *  @param FDCAN_HandleTypeDef *hcan - The pointer to the handle.
 *  @param uint16_t brp - The prescale value (data)
 *  @param uint16_t phase_seg1 - The time phase segment 1 value (data)
 *  @param uint16_t phase_seg2 - The time phase segment 2 value (data)
 *  @param uint8_t sjw - The sync jump width value (data)
 *  @retval None
 */
void can_set_data_bittiming(CAN_HANDLE_TYPEDEF *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
	if (  (brp > 0) && (brp <= 1024)
	   && (phase_seg1 > 0) && (phase_seg1 <= 16)
	   && (phase_seg2 > 0) && (phase_seg2 <= 8)
	   && (sjw > 0) && (sjw <= 4))
	{
#if defined(CAN)
		UNUSED(hcan);
		UNUSED(brp);
		UNUSED(phase_seg1);
		UNUSED(phase_seg2);
		UNUSED(sjw);
#elif defined(FDCAN1)
		hcan->Init.DataPrescaler = brp;
		hcan->Init.DataTimeSeg1 = phase_seg1;
		hcan->Init.DataTimeSeg2 = phase_seg2;
		hcan->Init.DataSyncJumpWidth = sjw;
#endif
	}
}

/** @brief Function to enable the CAN channel
 *  @param FDCAN_HandleTypeDef *hcan - The pointer to the handle.
 *  @param bool loop_back - flag to indicate CAN mode should be in loopback.
 *  @param bool listen_only - flag to indicate CAN mode should be in monitor.
 *  @param bool one_shot - flag to indicate CAN mode should be in one shot.
 *  @param bool can_mode_fd - flag to indicate CAN supports CAN-FD.
 *  @retval None
 */
void can_enable(CAN_HANDLE_TYPEDEF *hcan, bool loop_back, bool listen_only, bool one_shot, bool can_mode_fd)
{
#if defined(CAN)
	UNUSED(can_mode_fd);

	hcan->Init.AutoRetransmission = one_shot ? DISABLE : ENABLE;
	hcan->Init.Mode = CAN_MODE_NORMAL;
	if (listen_only) hcan->Init.Mode |= CAN_MODE_SILENT;
	if (loop_back) hcan->Init.Mode |= CAN_MODE_LOOPBACK;

	HAL_CAN_Init(hcan);

	// Configure reception filter to Rx FIFO 0
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(hcan, &sFilterConfig);

	HAL_CAN_Start(hcan);

	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING |
								 CAN_IT_ERROR_WARNING |
								 CAN_IT_ERROR_PASSIVE |
								 CAN_IT_BUSOFF |
								 CAN_IT_LAST_ERROR_CODE |
								 CAN_IT_ERROR);
#elif defined(FDCAN1)
	FDCAN_FilterTypeDef sFilterConfig;

	hcan->Init.AutoRetransmission = one_shot ? DISABLE : ENABLE;
	if (loop_back && listen_only) hcan->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
	else if (loop_back) hcan->Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
	else if (listen_only) hcan->Init.Mode = FDCAN_MODE_BUS_MONITORING;
	else hcan->Init.Mode = FDCAN_MODE_NORMAL;
	hcan->Init.FrameFormat = can_mode_fd ? FDCAN_FRAME_FD_BRS : FDCAN_FRAME_CLASSIC;

	HAL_FDCAN_Init(hcan);

	/* Configure reception filter to Rx FIFO 0 on both FDCAN instances */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
	sFilterConfig.FilterID1 = 0x000;
	sFilterConfig.FilterID2 = 0x7FF;

	HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig);

	/* Configure global filter on both FDCAN instances:
	   Filter all remote frames with STD and EXT ID
	   Reject non matching frames with STD ID and EXT ID */
	HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

	// Start CAN using HAL
	HAL_FDCAN_Start(hcan);

	HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
								   FDCAN_IT_ERROR_PASSIVE |
								   FDCAN_IT_ERROR_LOGGING_OVERFLOW |
								   FDCAN_IT_RESERVED_ADDRESS_ACCESS |
								   FDCAN_IT_DATA_PROTOCOL_ERROR |
								   FDCAN_IT_ARB_PROTOCOL_ERROR |
								   FDCAN_IT_RAM_WATCHDOG |
								   FDCAN_IT_BUS_OFF |
								   FDCAN_IT_ERROR_WARNING, 0);
#endif
	can_on_enable_cb(USBD_GS_CAN_GetChannelNumber(&hUSB, hcan));
}

/** @brief Function to isable the CAN channel
 *  @param FDCAN_HandleTypeDef *hcan - The pointer to the handle.
 *  @retval None
 */
void can_disable(CAN_HANDLE_TYPEDEF *hcan)
{
#if defined(CAN)
	HAL_CAN_Stop(hcan);
	HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING |
								   CAN_IT_ERROR_WARNING |
								   CAN_IT_ERROR_PASSIVE |
								   CAN_IT_BUSOFF |
								   CAN_IT_LAST_ERROR_CODE |
								   CAN_IT_ERROR);
#elif defined(FDCAN1)
	//Stop can using HAL
	HAL_FDCAN_Stop(hcan);
	HAL_FDCAN_DeactivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
									 FDCAN_IT_ERROR_PASSIVE |
									 FDCAN_IT_ERROR_LOGGING_OVERFLOW |
									 FDCAN_IT_RESERVED_ADDRESS_ACCESS |
									 FDCAN_IT_DATA_PROTOCOL_ERROR |
									 FDCAN_IT_ARB_PROTOCOL_ERROR |
									 FDCAN_IT_RAM_WATCHDOG |
									 FDCAN_IT_BUS_OFF |
									 FDCAN_IT_ERROR_WARNING);
#endif
	can_on_disable_cb(USBD_GS_CAN_GetChannelNumber(&hUSB, hcan));
}

bool can_is_enabled(CAN_HANDLE_TYPEDEF *hcan)
{
#if defined(CAN)
	return hcan->State == HAL_CAN_STATE_LISTENING;
#elif defined(FDCAN1)
	return hcan->State == HAL_FDCAN_STATE_BUSY;
#endif
}

/** @brief Function to disable the CAN channel
 *  @param FDCAN_HandleTypeDef *hcan - The pointer to the handle.
 *  @param struct gs_host_frame *frame - The pointer to the host frame containing message data.
 *  @retval true if TX was successful, false if no successful.
 */
bool can_send(CAN_HANDLE_TYPEDEF *hcan, struct gs_host_frame *frame)
{
#if defined(CAN)
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;

	TxHeader.StdId = frame->can_id & 0x7FF;
	TxHeader.ExtId = frame->can_id & 0x1FFFFFFF;
	TxHeader.RTR = frame->can_id & CAN_RTR_FLAG ? CAN_RTR_REMOTE : CAN_RTR_DATA;
	TxHeader.IDE = frame->can_id & CAN_EFF_FLAG ? CAN_ID_EXT : CAN_ID_STD;
	TxHeader.DLC = frame->can_dlc;
	TxHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, (uint8_t*)frame->classic_can->data, &TxMailbox) != HAL_OK) {
		return false;
	}
	else {
		return true;
	}
#elif defined(FDCAN1)
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.DataLength = frame->can_dlc << 16;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	if (frame->can_id & CAN_RTR_FLAG) {
		TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
	}
	else {
		TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	}

	if (frame->can_id & CAN_EFF_FLAG) {
		TxHeader.IdType = FDCAN_EXTENDED_ID;
		TxHeader.Identifier = frame->can_id & 0x1FFFFFFF;
	}
	else {
		TxHeader.IdType = FDCAN_STANDARD_ID;
		TxHeader.Identifier = frame->can_id & 0x7FF;
	}

	if (frame->flags & GS_CAN_FLAG_FD) {
		TxHeader.FDFormat = FDCAN_FD_CAN;
		if (frame->flags & GS_CAN_FLAG_BRS) {
			TxHeader.BitRateSwitch = FDCAN_BRS_ON;
		}
		else {
			TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
		}
	}
	else {
		TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
		TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	}

	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, (uint8_t*)frame->classic_can->data) != HAL_OK) {
		return false;
	}
	else {
		return true;
	}
#endif
}
/** @brief Set the termination I/O
 *  @param uint8_t channel - CAN channel
 *  @param uint8_t value - 0=Reset, 1=Set
 *  @retval None
 */
void can_set_termination(uint8_t channel, uint8_t value)
{
	GPIO_PinState pin_state = value == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET;
	can_set_term_cb(channel, pin_state);
}

/** @brief Set the termination I/O
 *  @param uint8_t channel - CAN channel
 *  @param uint8_t value - 0=Reset, 1=Set
 *  @retval None
 */
uint8_t can_get_termination(uint8_t channel)
{
	return can_get_term_cb(channel);
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signalled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
#if defined(CAN)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	struct gs_host_frame_object frame_object;
	/* Get RX message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, (uint8_t*)&frame_object.frame.classic_can->data) != HAL_OK)
	{
		return;
	}

	frame_object.frame.channel = USBD_GS_CAN_GetChannelNumber(&hUSB, hcan);
	frame_object.frame.can_dlc = RxHeader.DLC;
	if (RxHeader.IDE == CAN_ID_EXT) {
		frame_object.frame.can_id = RxHeader.ExtId | CAN_EFF_FLAG;
	}
	else {
		frame_object.frame.can_id = RxHeader.StdId;
	}

	if (RxHeader.RTR == CAN_RTR_REMOTE) {
		frame_object.frame.can_id |= CAN_RTR_FLAG;
	}

	frame_object.frame.echo_id = 0xFFFFFFFF;
	frame_object.frame.reserved = 0;
	frame_object.frame.flags = 0;
	frame_object.frame.classic_can_ts->timestamp_us = __HAL_TIM_GET_COUNTER(&htim2);

	/* put this CAN message into the queue to send to host */
	xQueueSendToBackFromISR(hGS_CAN.queue_to_hostHandle, &frame_object.frame, NULL);
}
#elif defined(FDCAN1)
void HAL_FDCAN_RxFifo0Callback(CAN_HANDLE_TYPEDEF *hcan, uint32_t RxFifo0ITs)
{
	UNUSED(RxFifo0ITs);
	FDCAN_RxHeaderTypeDef RxHeader;
	struct gs_host_frame_object frame_object;

	if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &RxHeader, frame_object.frame.canfd->data) != HAL_OK) {
		return;
	}

	frame_object.frame.channel = USBD_GS_CAN_GetChannelNumber(&hUSB, hcan);
	frame_object.frame.can_id = RxHeader.Identifier;

	if (RxHeader.IdType == FDCAN_EXTENDED_ID) {
		frame_object.frame.can_id |= CAN_EFF_FLAG;
	}

	if (RxHeader.RxFrameType == FDCAN_REMOTE_FRAME) {
		frame_object.frame.can_id |= CAN_RTR_FLAG;
	}

	frame_object.frame.can_dlc = (RxHeader.DataLength & 0x000F0000) >> 16;

	frame_object.frame.echo_id = 0xFFFFFFFF; // not a echo frame
	frame_object.frame.reserved = 0;
	frame_object.frame.flags = 0;

	if (RxHeader.FDFormat == FDCAN_FD_CAN) {
		/* this is a CAN-FD frame */
		frame_object.frame.flags = GS_CAN_FLAG_FD;
		if (RxHeader.BitRateSwitch == FDCAN_BRS_ON) {
			frame_object.frame.flags |= GS_CAN_FLAG_BRS;
		}

	}

	frame_object.frame.canfd_ts->timestamp_us = __HAL_TIM_GET_COUNTER(&htim2);

	/* put this CAN message into the queue to send to host */
	xQueueSendToBackFromISR(hGS_CAN.queue_to_hostHandle, &frame_object.frame, NULL);
}
#endif

/**
  * @brief  Error status callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  ErrorStatusITs indicates which Error Status interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Error_Status_Interrupts.
  * @retval None
  */
#if defined(CAN)
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	struct gs_host_frame_object frame_object;
	uint32_t can_err_status = hcan->Instance->ESR;
	can_parse_error_status(can_err_status, can_last_err_status, hcan, &frame_object.frame);
	/* put this CAN message into the queue to send to host */
	xQueueSendToBackFromISR(hGS_CAN.queue_to_hostHandle, &frame_object.frame, NULL);
	can_last_err_status = can_err_status;

}
#elif defined(FDCAN1)
void HAL_FDCAN_ErrorStatusCallback(CAN_HANDLE_TYPEDEF *hcan, uint32_t ErrorStatusITs)
{
	UNUSED(ErrorStatusITs);
	struct gs_host_frame_object frame_object;
	uint32_t can_err_status = hcan->Instance->PSR;
	can_parse_error_status(can_err_status, can_last_err_status, hcan, &frame_object.frame);
	/* put this CAN message into the queue to send to host */
	xQueueSendToBackFromISR(hGS_CAN.queue_to_hostHandle, &frame_object.frame, NULL);
	can_last_err_status = can_err_status;
}
#endif

static bool status_is_active(uint32_t err)
{
#if defined(CAN)
	return !(err & (CAN_ESR_BOFF | CAN_ESR_EPVF));
#elif defined(FDCAN1)
	return !(err & (FDCAN_PSR_BO | FDCAN_PSR_EP));
#endif
}

/** @brief Function parse the error data returned from the error callback
 *  @param FDCAN_ProtocolStatusTypeDef *status - pointer to the status data
 *  @param struct gs_host_frame *frame - The pointer to the host frame containing message data.
 *  @retval None
 */
bool can_parse_error_status(uint32_t err, uint32_t last_err, CAN_HANDLE_TYPEDEF *hcan, struct gs_host_frame *frame)
{
	/* We build up the detailed error information at the same time as we decide
	 * whether there's anything worth sending. This variable tracks that final
	 * result. */
	bool should_send = false;

	frame->echo_id = 0xFFFFFFFF;
	frame->can_id  = CAN_ERR_FLAG;
	frame->can_dlc = CAN_ERR_DLC;
	frame->classic_can->data[0] = CAN_ERR_LOSTARB_UNSPEC;
	frame->classic_can->data[1] = CAN_ERR_CRTL_UNSPEC;
	frame->classic_can->data[2] = CAN_ERR_PROT_UNSPEC;
	frame->classic_can->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
	frame->classic_can->data[4] = CAN_ERR_TRX_UNSPEC;
	frame->classic_can->data[5] = 0;
	frame->classic_can->data[6] = 0;
	frame->classic_can->data[7] = 0;

	/* We transitioned from passive/bus-off to active, so report the edge. */
	if (!status_is_active(last_err) && status_is_active(err)) {
		frame->can_id |= CAN_ERR_CRTL;
		frame->classic_can->data[1] |= CAN_ERR_CRTL_ACTIVE;
		should_send = true;
	}

#if defined(CAN)
	UNUSED(hcan);
	if (err & CAN_ESR_BOFF) {
		if (!(last_err & CAN_ESR_BOFF)) {
			/* We transitioned to bus-off. */
			frame->can_id |= CAN_ERR_BUSOFF;
			should_send = true;
		}
		// - tec (overflowed) / rec (looping, likely used for recessive counting)
		//   are not valid in the bus-off state.
		// - The warning flags remains set, error passive will cleared.
		// - LEC errors will be reported, while the device isn't even allowed to send.
		//
		// Hence only report bus-off, ignore everything else.
		return should_send;
	}

	/* The Linux sja1000 driver puts these counters here. Seems like as good a
	 * place as any. */
	frame->classic_can->data[6] = (err>>16) & 0xFF;
	frame->classic_can->data[7] = (err>>24) & 0xFF;

	if (err & CAN_ESR_EPVF) {
		if (!(last_err & CAN_ESR_EPVF)) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->classic_can->data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
			should_send = true;
		}
	} else if (err & CAN_ESR_EWGF) {
		if (!(last_err & CAN_ESR_EWGF)) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->classic_can->data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
			should_send = true;
		}
	}

	uint8_t lec = (err>>4) & 0x07;
#elif defined(FDCAN1)
	if (err & FDCAN_PSR_BO) {
		if (!(last_err & FDCAN_PSR_BO)) {
			/* We transitioned to bus-off. */
			frame->can_id |= CAN_ERR_BUSOFF;
			should_send = true;
		}
	}

	/* The Linux sja1000 driver puts these counters here. Seems like as good a
	* place as any. */
	// TX error count
	frame->classic_can->data[6] = ((hcan->Instance->ECR & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos);
	// RX error count
	frame->classic_can->data[7] = ((hcan->Instance->ECR & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos);

	if (err & FDCAN_PSR_EP) {
		if (!(last_err & FDCAN_PSR_EP)) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->classic_can->data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
			should_send = true;
		}
	} else if (err & FDCAN_PSR_EW) {
		if (!(last_err & FDCAN_PSR_EW)) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->classic_can->data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
			should_send = true;
		}
	}

	uint8_t lec = err & FDCAN_PSR_LEC;
#endif

	switch (lec) {
		case 0x01: /* stuff error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can->data[2] |= CAN_ERR_PROT_STUFF;
			should_send = true;
			break;
		case 0x02: /* form error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can->data[2] |= CAN_ERR_PROT_FORM;
			should_send = true;
			break;
		case 0x03: /* ack error */
			frame->can_id |= CAN_ERR_ACK;
			should_send = true;
			break;
		case 0x04: /* bit recessive error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can->data[2] |= CAN_ERR_PROT_BIT1;
			should_send = true;
			break;
		case 0x05: /* bit dominant error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can->data[2] |= CAN_ERR_PROT_BIT0;
			should_send = true;
			break;
		case 0x06: /* CRC error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
			should_send = true;
			break;
		default: /* 0=no error, 7=no change */
			break;
	}
	return should_send;
}

/* weak function calls to allow callbacks to be optionally included */
/** @brief Callback function called when the CAN channel is enabled
 *  @param uint8_t channel - CAN channel that was acted upon
 *  @retval None
 */
__weak void can_on_enable_cb(uint8_t channel)
{
	UNUSED(channel);
}
/** @brief Callback function called when the CAN channel is disabled
 *  @param uint8_t channel - CAN channel that was acted upon
 *  @retval None
 */
__weak void can_on_disable_cb(uint8_t channel)
{
	UNUSED(channel);
}
/** @brief Callback function called when a message is succefully
 *         transmitted on the CAN bus
 *  @param uint8_t channel - CAN channel that was acted upon
 *  @param struct gs_host_frame *frame - data that was sent on the bus
 *  @retval None
 */
__weak void can_on_tx_cb(uint8_t channel, struct gs_host_frame *frame)
{
	UNUSED(channel);
	UNUSED(frame);
}
/** @brief Callback function called when a message is succefully
 *         received on the CAN bus
 *  @param uint8_t channel - CAN channel that was acted upon
 *  @param struct gs_host_frame *frame - data that was received on the bus
 *  @retval None
 */
__weak void can_on_rx_cb(uint8_t channel, struct gs_host_frame *frame)
{
	UNUSED(channel);
	UNUSED(frame);
}
/** @brief Callback function called the identify request is made by the host
 *  @param uint32_t do_identify - 0=identify off, 1=identify on
 *  @retval None
 */
__weak void can_identify_cb(uint32_t do_identify)
{
	UNUSED(do_identify);
}
/** @brief Callback function called to set the state of the
 *         CAN termination resistor
 *  @param uint8_t channel - CAN channel to act upon
 *  @param GPIO_PinState state - requested state of the pin
 *  @retval None
 */
__weak void can_set_term_cb(uint8_t channel, GPIO_PinState state)
{
	UNUSED(channel);
	UNUSED(state);
}
/** @brief Callback function called to get the state of the
 *         CAN termination resistor
 *  @retval GPIO_PinState state - current state of the pin
 */
__weak GPIO_PinState can_get_term_cb(uint8_t channel)
{
	UNUSED(channel);
	return 0;
}
