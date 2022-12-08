/*

The MIT License (MIT)

Copyright (c) 2022 R. Edwards

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

/**
******************************************************************************
* @file           : lin.c
* @brief          : LIN driver to interface with the STM32
******************************************************************************
*/



#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "lin.h"
#include "main.h"
#include "can.h"
#include "usbd_gs_can.h"
#include "board.h"

#if defined(LIN_FEATURE_ENABLED)
#define LIN_SYNC_BYTE		  0x55u
#define LIN_GET_PID_BIT(x,y) (((x) >> (y)) & 0x01u)
#define LIN_ID_MASK			  0x3Fu
#define LIN_RX_TIMEOUT_VALUE  3
#define LIN_RX_MAX_DATA_BYTES 9

extern USBD_GS_CAN_HandleTypeDef hGS_CAN;
extern TIM_HandleTypeDef htim2;

#if defined (LIN_GATEWAY_CAN_CH)
extern CAN_HANDLE_TYPEDEF LIN_GATEWAY_CAN_CH;
#endif

static void lin_state_machine_reset(LIN_HandleTypeDef* hlin);
static FlagStatus lin_hw_check_for_break(LIN_HandleTypeDef* hlin);
static uint8_t lin_data_layer_checksum(uint8_t pid, uint8_t length, const uint8_t* data_ptr);
static void lin_erase_slot_table(LIN_HandleTypeDef* hlin);
static void lin_disable_all_slots(LIN_HandleTypeDef* hlin);
static void lin_can_gateway_tx(LIN_HandleTypeDef* hlin);
static uint8_t lin_config(LIN_HandleTypeDef* hlin, uint32_t msg_id, uint8_t *data);


void lin_init(LIN_HandleTypeDef* hlin, uint8_t lin_instance, UART_HandleTypeDef* huart)
{
	/* erase the slot table to ensure no random data */
	lin_erase_slot_table(hlin);

	/* init the LIN handles */
	hlin->huart = huart;
	hlin->lin_instance = lin_instance;

	lin_state_machine_reset(hlin);

	HAL_UART_Receive_IT(hlin->huart, hlin->UartRxBuffer, 1);
}

static void lin_state_machine_reset(LIN_HandleTypeDef* hlin) 
{
	/* Reset the LIN state machine */
	vPortEnterCritical();
	hlin->lin_data_frame.pid = 0;
	memset(hlin->lin_data_frame.lin_data_buffer, 0x00, sizeof(hlin->lin_data_frame.lin_data_buffer));
	hlin->lin_flags.lin_rx_data_available = 0;
	hlin->lin_data_frame.data_index = 0;
	hlin->lin_state = LIN_IDLE_AWAIT_BREAK;
	vPortExitCritical();
}

void lin_transmit_master(LIN_HandleTypeDef* hlin, uint8_t pid)
{
	uint8_t sync_byte = 0x55;
	
	HAL_LIN_SendBreak(hlin->huart);

	if (hlin->lin_flags.lin_master_req_type == LIN_MASTER_REQ_BREAK_ONLY) {
		return;
	}

	while (HAL_UART_Transmit_IT(hlin->huart, &sync_byte, 1) != HAL_OK) {
		vTaskDelay(pdMS_TO_TICKS(0));
	}
	while (HAL_UART_Transmit_IT(hlin->huart, &pid, 1) != HAL_OK) {
		vTaskDelay(pdMS_TO_TICKS(0));
	}
	
	vPortEnterCritical();
	hlin->lin_data_frame.pid = pid;
	hlin->lin_data_frame.data_index = 0;
	if (hlin->lin_flags.lin_master_req_type == LIN_MASTER_REQ_HEADER_ONLY) {
		hlin->lin_state = LIN_MASTER_RX_PID;
		hlin->lin_rx_timeout_starttick = HAL_GetTick();
	}
	else if (hlin->lin_flags.lin_master_req_type == LIN_MASTER_REQ_MASTER_FRAME_TX) {
		hlin->lin_state = LIN_SLAVE_TX_DATA;
	}
	vPortExitCritical();
}

void lin_process_frame(struct gs_host_frame* frame)
{
	uint32_t msg_id = frame->can_id & 0x1FFFFFFF;
	
	LIN_HandleTypeDef* hlin = lin_get_handle(msg_id);

	if (hlin == NULL) {
		return;
	}
	/* if a config frame process the config */
	if (IS_LIN_CONFIG_FRAME(msg_id)) {
		vPortEnterCritical();
		lin_config(hlin, msg_id, frame->classic_can->data);
		vPortExitCritical();
	}
	else if (IS_LIN_MASTER_HEADER_FRAME(msg_id)) {
		/* header only signals a slave to respond */
		hlin->lin_state = LIN_MASTER_TX_HEADER;
		hlin->lin_flags.lin_master_req_type = LIN_MASTER_REQ_HEADER_ONLY;
		hlin->lin_data_frame.pid = frame->classic_can->data[0];
	}
	else if (IS_LIN_MASTER_FRAME(msg_id)) {
	/* if a master frame broadcast - send message */
		hlin->lin_state = LIN_MASTER_TX_HEADER;
		hlin->lin_flags.lin_master_req_type = LIN_MASTER_REQ_MASTER_FRAME_TX;
		hlin->lin_data_frame.pid = frame->classic_can->data[0];
		hlin->lin_data_frame.tx_msg_len = frame->classic_can->data[1];
		/* right now only 6 bytes as we don't have enough payload - need to switch to CANFD */
		memcpy(&hlin->lin_data_frame.lin_data_buffer, &frame->classic_can->data[2], 6);
	}
	
	/* since this came in as a frame from the host we have to echo it */
	frame->flags = 0x0;
	frame->reserved = 0x0;
	xQueueSendToFront(hGS_CAN.queue_to_hostHandle, frame, 0);
}

void lin_rx_IRQ_handler(LIN_HandleTypeDef* hlin)
{
	/* create a local copy for readability */
	uint8_t rxbyte = hlin->UartRxBuffer[0];
	/* LIN can be reset and state flags set outside of this routine as needed */
	switch (hlin->lin_state) {
		case LIN_IDLE_AWAIT_BREAK:
			hlin->lin_data_frame.data_index = 0;
			/* if flag is set to wait for sync and we get sync set flag to get ID */
			if (lin_hw_check_for_break(hlin) == SET  && rxbyte == LIN_SYNC_BYTE) {
				hlin->lin_rx_timeout_starttick = HAL_GetTick();
				hlin->lin_state = LIN_PID_RX;
			}
			break;

		case LIN_PID_RX:
			/* is this PID in the config table - loop to find out */
			/* based on the table this is either a master, slave, or monitor */
			hlin->lin_rx_timeout_starttick = HAL_GetTick();
			hlin->lin_data_frame.pid = rxbyte;
			for (uint8_t lin_slot_index = 0; lin_slot_index < LIN_MAX_SLOTS; lin_slot_index++) {
				if ((hlin->lin_data_frame.pid & LIN_ID_MASK) == hlin->lin_slot_table[lin_slot_index].PID  &&
					(hlin->lin_slot_table[lin_slot_index].lin_slot_flags.is_active)) {
					/* there is an instance in the table for this PID - handle based on the flags */
					hlin->lin_slot_table_index = lin_slot_index;
					if (hlin->lin_slot_table[lin_slot_index].lin_slot_flags.lin_node_action == LIN_MONITOR) {
						hlin->lin_state = LIN_MONITOR_RX_DATA;
					}
					else if (hlin->lin_slot_table[lin_slot_index].lin_slot_flags.lin_node_action == LIN_SLAVE) {
						hlin->lin_data_frame.tx_msg_len = hlin->lin_slot_table[lin_slot_index].len;
						memcpy(hlin->lin_data_frame.lin_data_buffer, 
								hlin->lin_slot_table[lin_slot_index].data,
								hlin->lin_slot_table[lin_slot_index].len);
						hlin->lin_state = LIN_SLAVE_TX_DATA;
					}
					else {
						/* there was a match but no matching action so reset */
						hlin->lin_state = LIN_IDLE_AWAIT_BREAK;
					}
					break; /* exit the for loop as we have found a matching item */
				}
			}
			break;

		case LIN_MASTER_RX_PID:
			hlin->lin_data_frame.pid = rxbyte;
			hlin->lin_state = LIN_MASTER_RX_DATA;
			break;

		case LIN_MONITOR_RX_DATA:
		case LIN_MASTER_RX_DATA:
			/* reset the timeout for this reception state */
			/* this is handled in the interrupt since data is read byte by byte */
			hlin->lin_rx_timeout_starttick = HAL_GetTick();
			hlin->lin_data_frame.lin_data_buffer[hlin->lin_data_frame.data_index] = rxbyte;
			if (hlin->lin_data_frame.data_index > LIN_RX_MAX_DATA_BYTES) {
				/* we've received the max # of bytes, flag it */
				hlin->lin_flags.lin_rx_data_available = 1;
			}
			else {
				hlin->lin_data_frame.data_index++;
			}
			break;

		default:
			break;
	}
}

void lin_handler_task(LIN_HandleTypeDef* hlin)
{
	uint8_t data_length;
	uint8_t checksum;
	uint8_t pid;

	switch (hlin->lin_state) {
		case LIN_MASTER_TX_HEADER:
			lin_transmit_master(hlin, hlin->lin_data_frame.pid);
			break;

		case LIN_MONITOR_RX_DATA:
		case LIN_MASTER_RX_DATA:
			if (((HAL_GetTick() - hlin->lin_rx_timeout_starttick) > LIN_RX_TIMEOUT_VALUE)
					|| (hlin->lin_flags.lin_rx_data_available)) {
				/* either the max number of bytes were received or the state machine timed out waiting for more data */
				hlin->lin_data_frame.data_index--; /* subtract the checksum from the data length */
				hlin->lin_data_frame.checksum = hlin->lin_data_frame.lin_data_buffer[hlin->lin_data_frame.data_index];
				if (hlin->lin_data_frame.checksum == lin_data_layer_checksum(hlin->lin_data_frame.pid,
																				hlin->lin_data_frame.data_index, /* subtract the PID from the data length */
																				hlin->lin_data_frame.lin_data_buffer)) {
					/* We have good data, take action on it */
					/* send this data on CAN */
					lin_can_gateway_tx(hlin);
				}
				else {
					/* checksum doesn't match - TODO: throw an error eventually */
				}

				/* Reset the LIN state machine */
				lin_state_machine_reset(hlin);
			}
			break;
			
		case LIN_SLAVE_TX_DATA:
			/* we have RX'd a PID that we now need to respond to */
			/* need the table index that we used to trigger the PID */
			/* create a copy of the data length for readability */
			data_length = hlin->lin_data_frame.tx_msg_len;
			pid = hlin->lin_data_frame.pid;
			memcpy(hlin->UartTxBuffer, hlin->lin_data_frame.lin_data_buffer, data_length);

			/* calculate the checksum */
			checksum = lin_data_layer_checksum(pid,
												data_length,
												hlin->UartTxBuffer);

			/* add the checksum to the end of the buffer */
			hlin->UartTxBuffer[data_length] = checksum;

			/* TX the data over the UART -- THIS IS BLOCKING */
			while (HAL_UART_Transmit_IT(hlin->huart, hlin->UartTxBuffer, data_length + 1) != HAL_OK) {
				vTaskDelay(pdMS_TO_TICKS(0));
			}

			/* Reset the LIN state machine */
			lin_state_machine_reset(hlin);
			break;
		
		case LIN_IDLE_AWAIT_BREAK:
			/* We are already in the pending state, no need to reset the state machine */
			break;

		default:
			if ((HAL_GetTick() - hlin->lin_rx_timeout_starttick) > LIN_RX_TIMEOUT_VALUE) {
				lin_state_machine_reset(hlin);
			}
			break;
	}		
}

static uint8_t lin_config(LIN_HandleTypeDef* hlin, uint32_t msg_id, uint8_t *data)
{
	if (msg_id == LIN_CONFIG_MSG_ID_DATA) {
		/* store the data into the config data store */
		memcpy(hlin->lin_config_data.lin_data.data, data, 8);
		return 1;
	}

	hlin->lin_config_data.lin_cmd_type = data[0];
	hlin->lin_config_data.lin_channel = data[1];
	hlin->lin_config_data.lin_table_index = data[2];
	hlin->lin_config_data.lin_data.lin_slot_flags.is_active = data[3];
	hlin->lin_config_data.lin_data.lin_slot_flags.lin_node_action = data[4];
	hlin->lin_config_data.lin_data.PID = data[5];
	hlin->lin_config_data.lin_data.len = data[6];

	switch (hlin->lin_config_data.lin_cmd_type) {
		case LIN_CMD_LOAD_SLOT:
			/* load the slot table with the passed data */
			hlin->lin_slot_table[hlin->lin_config_data.lin_table_index] = hlin->lin_config_data.lin_data;
			break;

		case LIN_CMD_READ_SLOT:
			/* read the data out of the slot table and send it back to the host */
			/* TODO: Figure out a good way to do this if it's really needed */
			break;

		case LIN_CMD_ENABLE_SLOT:
			/* set the flag to enable one slot */
			hlin->lin_slot_table[hlin->lin_config_data.lin_table_index].lin_slot_flags.is_active = 1;
			break;

		case LIN_CMD_DISABLE_SLOT:
			/* clear the flag to disable one slot */
			hlin->lin_slot_table[hlin->lin_config_data.lin_table_index].lin_slot_flags.is_active = 0;
			break;

		case LIN_CMD_DISABLE_ALL_SLOTS:
			lin_disable_all_slots(hlin);
			break;

		case LIN_CMD_ERASE_ALL_SLOTS:
			lin_erase_slot_table(hlin);
			break;

		case LIN_CMD_ENABLE_LIN:
			/* to do, need a lookup table to get the baud info */
			//static void lin_usart_enable(LIN_USART_HW[lin_ctrl_slot->LIN_Channel], NEED BAUD LOOKUP ETC.);
			break;

		case LIN_CMD_DISABLE_LIN:
			/* to do, need a lookup table to get the pin info */
			//lin_usart_disable(LIN_USART_HW[lin_ctrl_slot->LIN_Channel], NEED PIN LOOKUP);
			break;

		default:
			break;
	}

	return 1;
}

static void lin_erase_slot_table(LIN_HandleTypeDef* hlin)
{
	/* routine to zero out all data in the slot table */
	for (uint8_t lin_slot_index = 0; lin_slot_index < LIN_MAX_SLOTS; lin_slot_index++) {
		hlin->lin_slot_table[lin_slot_index].lin_slot_flags.is_active = 0;
		//lin_slot_table[usart_chan][lin_slot_index].lin_slot_flags.lin_node_action = US_LINMR_NACT_IGNORE;
		hlin->lin_slot_table[lin_slot_index].PID = 0x00;
		hlin->lin_slot_table[lin_slot_index].len = 0x00;
		for (uint8_t data_index = 0; data_index <  LIN_MAX_DATA_BYTES; data_index++) {
			hlin->lin_slot_table[lin_slot_index].data[data_index] = 0x00;
		}
	}
}

static void lin_disable_all_slots(LIN_HandleTypeDef* hlin)
{
	/* routine to disable all slots in the slot table */
	for (uint8_t lin_slot_index = 0; lin_slot_index < LIN_MAX_SLOTS; lin_slot_index++) {
		hlin->lin_slot_table[lin_slot_index].lin_slot_flags.is_active = 0;
	}
}

static FlagStatus lin_hw_check_for_break(LIN_HandleTypeDef* hlin)
{
	FlagStatus result = __HAL_UART_GET_FLAG(hlin->huart, UART_FLAG_LBDF);
	if (result == SET) {
		__HAL_UART_CLEAR_FLAG(hlin->huart, UART_CLEAR_LBDF);
	}

	return result;
}

static uint8_t lin_data_layer_checksum(uint8_t pid, uint8_t length, const uint8_t* data_ptr)
{
	uint16_t chk = pid;
	// loop over data bytes
	for (uint8_t i = 0; i < length; i++)
	{
		chk += (uint16_t) (data_ptr[i]);
		if (chk>255)
			chk -= 255;
	}
	chk = (uint8_t)(0xFF - ((uint8_t) chk)); // bitwise invert
	// return frame checksum
	return (uint8_t)chk;
}

static void lin_can_gateway_tx(LIN_HandleTypeDef* hlin)
{
#if defined(LIN_GATEWAY_MSG_ID)
  struct gs_host_frame_object frame_object;

	frame_object.frame.echo_id = 0xFFFFFFFF;
	frame_object.frame.reserved = 0;
	frame_object.frame.flags = 0;
	frame_object.frame.channel = 0;
	//frame_object.frame.classic_can_ts->timestamp_us = __HAL_TIM_GET_COUNTER(&htim2);

	/* zero out the buffer data for good measure */
	for (uint8_t i = 0 ; i < 8 ; i++) {
		frame_object.frame.classic_can->data[i]  = 0x00;
	}

	frame_object.frame.can_id = (LIN_GATEWAY_MSG_ID | CAN_EFF_FLAG);
	frame_object.frame.can_dlc = 8;

	frame_object.frame.classic_can->data[0] = hlin->lin_data_frame.pid & LIN_ID_MASK;
	frame_object.frame.classic_can->data[1] = hlin->lin_data_frame.data_index;

	/* can only use the first 6 bytes of data in the LIN frame due to CAN data limits */
	uint8_t lin_gateway_tx_len = hlin->lin_data_frame.data_index;
	if (lin_gateway_tx_len > 6) {
		lin_gateway_tx_len = 6;
	}

	for (uint8_t i = 0; i < lin_gateway_tx_len; i++) {
		frame_object.frame.classic_can->data[i + 2] = hlin->lin_data_frame.lin_data_buffer[i];
	}

	/* Set this frame just like a regular CAN frame - reuse the can_send routine */
	if (hGS_CAN.channels[0]->State == HAL_FDCAN_STATE_BUSY) {
		/* only send if CAN is active */
		xQueueSendToBack(hGS_CAN.queue_to_hostHandle, &frame_object.frame, 0);
		//can_send(&LIN_GATEWAY_CAN_CH, &frame_object.frame);
	}

#else
	UNUSED(hlin);
#endif
}

__weak LIN_HandleTypeDef* lin_get_handle(uint32_t msg_id)
{
	UNUSED(msg_id);
	return NULL;
}
#endif /* LIN_FEATURE_ENABLED */
