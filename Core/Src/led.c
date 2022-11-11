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

#include "main.h"
#include "board.h"
#include "led.h"

/** @brief Function to initialize the LED handle
 *  @param LED_HandleTypeDef* hled - The pointer to the handle.
 *  @param GPIO_TypeDef *GPIOx - The pointer to the GPIO port.
 *  @param uint16_t GPIO_Pin - The pin number.
 *  @param led_mode_t led_mode - The desired initial mode.
 *  @param uint8_t led_active_level - Define if the LED is active HIGH or LOW.
 *  @retval None
 */
void led_init(LED_HandleTypeDef* hled, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, led_mode_t led_mode, uint8_t led_active_level)
{
  hled->GPIOx = GPIOx;
  hled->GPIO_Pin = GPIO_Pin;
  hled->led_mode = led_mode;
  hled->prev_led_mode = led_mode;
  hled->led_active_level = led_active_level;
  /* start with the LED inactive at init */
  HAL_GPIO_WritePin(hled->GPIOx, hled->GPIO_Pin, !led_active_level);

}

/** @brief Function to be called periodically to update the specific handle
 *  @param LED_HandleTypeDef* hled - The pointer to the handle.
 *  @retval None
 */
void led_update(LED_HandleTypeDef* hled)
{

  /* check this handle for it's current mode and update the output accordingly */
  switch (hled->led_mode) {
    case LED_MODE_INACTIVE:
      HAL_GPIO_WritePin(hled->GPIOx, hled->GPIO_Pin, !hled->led_active_level);
      break;
    case LED_MODE_ACTIVE:
      HAL_GPIO_WritePin(hled->GPIOx, hled->GPIO_Pin, hled->led_active_level);
      break;
    case LED_MODE_RXTX_ACTIVE:
      if ((HAL_GetTick() -  hled->led_rxtx_start_tick) > LED_RXTX_ACTIVE_TIME_MS) {
        /* Turn off LED and change the mode to OFF */
        HAL_GPIO_WritePin(hled->GPIOx, hled->GPIO_Pin, !hled->led_active_level);
        hled->led_mode = LED_MODE_RXTX_HOLDOFF;
        hled->led_rxtx_start_tick = HAL_GetTick();
      }
      break;
    case LED_MODE_RXTX_HOLDOFF:
      /* prevent LED from going solid ON for high traffic */
      if ((HAL_GetTick() - hled->led_rxtx_start_tick) > LED_RXTX_INACTIVE_TIME_MS) {
        /* Return the LED to it's previous state */
        hled->led_mode = hled->prev_led_mode;
      }
      break;
    case LED_MODE_BLINK:
      if ((HAL_GetTick() - hled->blink_toggle_start_tick) > (hled->blink_period_ms/2)){
        HAL_GPIO_TogglePin(hled->GPIOx, hled->GPIO_Pin);
        hled->blink_toggle_start_tick = HAL_GetTick();
      }
    default:
      break;
  }
}

/** @brief Function to read the LED current mode
 *  @param LED_HandleTypeDef* hled - The pointer to the handle.
 *  @retval led_mode_t - the enumeratred led mode
 */
led_mode_t led_get_mode(LED_HandleTypeDef* hled)
{
  return hled->led_mode;
}

/** @brief Function to set the LED into it's active mode
 *  @param LED_HandleTypeDef* hled - The pointer to the handle.
 *  @retval None
 */
void led_set_active(LED_HandleTypeDef* hled)
{
  /* set the mode flag - the handler will update the GPIO */
  hled->prev_led_mode = hled->led_mode;
  hled->led_mode = LED_MODE_ACTIVE;
}

/** @brief Function to set the LED into it's inactive mode
 *  @param LED_HandleTypeDef* hled - The pointer to the handle.
 *  @retval None
 */
void led_set_inactive(LED_HandleTypeDef* hled)
{
  /* set the mode flag - the handler will update the GPIO */
  hled->prev_led_mode = hled->led_mode;
  hled->led_mode = LED_MODE_INACTIVE;
}

/** @brief Function to indicate to the LED handler that an RX or TX event took place to pulse the output
 *  @param LED_HandleTypeDef* hled - The pointer to the handle.
 *  @retval None
 */
void led_indicate_rxtx(LED_HandleTypeDef* hled)
{
  /* check to see if we have already started to indicate an RXTX to prevent steady state LED on high message rates */
  if ((hled->led_mode != LED_MODE_RXTX_ACTIVE)  && (hled->led_mode != LED_MODE_RXTX_HOLDOFF)) {
    /* turn on the LED for a brief period of time to indicate a message was sent */
    HAL_GPIO_WritePin(hled->GPIOx, hled->GPIO_Pin, hled->led_active_level);
    hled->prev_led_mode = hled->led_mode;
    hled->led_mode = LED_MODE_RXTX_ACTIVE;
    hled->led_rxtx_start_tick = HAL_GetTick();
  }
}

/** @brief Function to initiate a blink pattern on the LED
 *  @param LED_HandleTypeDef* hled - The pointer to the handle.
 *  @param uint32_t period_ms - The period in ms that the LED should blink.
 *  @retval None
 */
void led_blink(LED_HandleTypeDef* hled, uint32_t period_ms)
{
  hled->prev_led_mode = hled->led_mode;
  hled->prev_blink_period_ms = hled->blink_period_ms;
  hled->blink_period_ms = period_ms;
  hled->led_mode = LED_MODE_BLINK;
  hled->blink_toggle_start_tick = HAL_GetTick();
}

/** @brief Function to restore the previous LED mode
 *  @param LED_HandleTypeDef* hled - The pointer to the handle.
 *  @retval None
 */
void led_restore_prev_mode(LED_HandleTypeDef* hled)
{
  hled->led_mode = hled->prev_led_mode;
  hled->blink_period_ms = hled->prev_blink_period_ms;
}

