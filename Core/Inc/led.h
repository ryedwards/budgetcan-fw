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

#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported defines -----------------------------------------------------------*/
#if !defined (LED_RXTX_ACTIVE_TIME_MS)
#define LED_RXTX_ACTIVE_TIME_MS     50U
#endif
#if !defined (LED_RXTX_INACTIVE_TIME_MS)
#define LED_RXTX_INACTIVE_TIME_MS   50U
#endif

#define LED_ACTIVE_LOW              0U
#define LED_ACTIVE_HIGH             1U

/* Exported types ------------------------------------------------------------*/
typedef enum {
  LED_MODE_INACTIVE = 0,
  LED_MODE_ACTIVE,
  LED_MODE_RXTX_ACTIVE,
  LED_MODE_RXTX_HOLDOFF,
  LED_MODE_BLINK
} led_mode_t;

typedef struct {
  GPIO_TypeDef *GPIOx;
  uint16_t GPIO_Pin;
  GPIO_PinState initial_state;
  led_mode_t led_mode;
  led_mode_t prev_led_mode;
  uint8_t led_active_level;
  uint32_t led_rxtx_start_tick;
  uint32_t blink_period_ms;
  uint32_t prev_blink_period_ms;
  uint32_t blink_toggle_start_tick;
} LED_HandleTypeDef;

/* Exported functions --------------------------------------------------------*/
void led_init(LED_HandleTypeDef* hled, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, led_mode_t led_mode, uint8_t led_active_level);
void led_update(LED_HandleTypeDef* hled);
led_mode_t led_get_mode(LED_HandleTypeDef* hled);
void led_set_active(LED_HandleTypeDef* hled);
void led_set_inactive(LED_HandleTypeDef* hled);
void led_indicate_rxtx(LED_HandleTypeDef* hled);
void led_blink(LED_HandleTypeDef* hled, uint32_t period_ms);
void led_restore_prev_mode(LED_HandleTypeDef* hled);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */
