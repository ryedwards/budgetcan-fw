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
#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "gs_usb.h"
#include "usbd_gs_can.h"

/* Exported functions --------------------------------------------------------*/
void can_init(CAN_HANDLE_TYPEDEF *hcan, CAN_TYPEDEF *instance);
void can_set_bittiming(CAN_HANDLE_TYPEDEF *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw);
void can_set_data_bittiming(CAN_HANDLE_TYPEDEF *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw);
void can_enable(CAN_HANDLE_TYPEDEF *hcan, bool loop_back, bool listen_only, bool one_shot, bool can_mode_fd);
void can_disable(CAN_HANDLE_TYPEDEF *hcan);
bool can_is_enabled(CAN_HANDLE_TYPEDEF *hcan);
bool can_send(CAN_HANDLE_TYPEDEF *hcan, struct GS_HOST_FRAME *frame);
void can_set_termination(uint8_t channel, uint8_t value);
uint8_t can_get_termination(uint8_t channel);

void can_on_enable_cb(uint8_t channel);
void can_on_disable_cb(uint8_t channel);
void can_on_tx_cb(uint8_t channel);
void can_on_rx_cb(uint8_t channel);
void can_set_term_cb(uint8_t channel, GPIO_PinState state);
GPIO_PinState can_get_term_cb(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */
