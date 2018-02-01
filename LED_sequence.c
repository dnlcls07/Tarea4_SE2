/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    LED_sequence.c
 * @brief   Application entry point.
 */

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

#define BLUE_LED_PORT 21
#define RED_LED_PORT 22
#define GREEN_LED_PORT 24

#define RED_ON &led_machine[0]
#define BLUE_ON &led_machine[1]
#define GREEN_ON &led_machine[2]

typedef const struct{
  const struct led_state_t* next_state;
  const struct led_state_t* inv_state;
  GPIO_Type* GPIO_led_on;
  GPIO_Type* GPIO_led_off_0;
  GPIO_Type* GPIO_led_off_1;
  uint8_t led_on;
  uint8_t led_off_0;
  uint8_t led_off_1;
} led_state_t;

led_state_t led_machine[3] = {
    {BLUE_ON,GREEN_ON,GPIOB,GPIOB,GPIOE,RED_LED_PORT,BLUE_LED_PORT,GREEN_LED_PORT},
    {GREEN_ON,RED_ON,GPIOB,GPIOB,GPIOE,BLUE_LED_PORT,RED_LED_PORT,GREEN_LED_PORT},
    {RED_ON,BLUE_ON,GPIOE,GPIOB,GPIOB,GREEN_LED_PORT,BLUE_LED_PORT,RED_LED_PORT}
};

led_state_t* led_state = RED_ON;

uint8_t switch_flag = 0;
uint8_t hold_flag = 0;

void PORTA_IRQHandler(){
  PORT_ClearPinsInterruptFlags(PORTA, 1<<4);
  switch_flag = ~(switch_flag);
}

void PORTC_IRQHandler(){
  PORT_ClearPinsInterruptFlags(PORTC, 1<<6);
  hold_flag = ~(hold_flag);
  if (hold_flag)
    PIT_StopTimer (PIT, kPIT_Chnl_0);
  else
    PIT_StartTimer (PIT, kPIT_Chnl_0);
}

void PIT0_IRQHandler(){
  if (!switch_flag)
      led_state = led_state->next_state;
  else
    led_state = led_state->inv_state;
  GPIO_WritePinOutput (led_state->GPIO_led_on,led_state->led_on , 0);
  GPIO_WritePinOutput (led_state->GPIO_led_off_0,led_state->led_off_1 , 1);
  GPIO_WritePinOutput (led_state->GPIO_led_off_0,led_state->led_off_1 , 1);
}

/*
 * @brief   Application entry point.
 */
int
main (void)
{

  /* Init board hardware. */
  BOARD_InitBootPins ();
  BOARD_InitBootClocks ();
  BOARD_InitBootPeripherals ();
  /* Init FSL debug console. */
  BOARD_InitDebugConsole ();

  pit_config_t * pit_config;

  port_pin_config_t config_led =
    { kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
	kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
	kPORT_UnlockRegister, };

  gpio_pin_config_t led_config_gpio =
    { kGPIO_DigitalOutput, 1 };

  PORT_SetPinConfig (PORTB, 21, &config_led);	//Blue LED
  PORT_SetPinConfig (PORTB, 22, &config_led);	//Red LED
  PORT_SetPinConfig (PORTE, 26, &config_led);	//Green LED

  GPIO_PinInit (GPIOB, 21, &led_config_gpio);
  GPIO_PinInit (GPIOB, 22, &led_config_gpio);
  GPIO_PinInit (GPIOE, 26, &led_config_gpio);

  port_pin_config_t config_switch =
    { kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
	kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
	kPORT_UnlockRegister };

  gpio_pin_config_t switch_config_gpio =
    { kGPIO_DigitalInput, 1 };

  PORT_SetPinInterruptConfig (PORTA, 4, kPORT_InterruptFallingEdge);
  PORT_SetPinInterruptConfig (PORTC, 6, kPORT_InterruptFallingEdge);

  PORT_SetPinConfig (PORTA, 4, &config_switch);
  PORT_SetPinConfig (PORTC, 6, &config_switch);
  GPIO_PinInit (GPIOA, 4, &switch_config_gpio);
  GPIO_PinInit (GPIOC, 6, &switch_config_gpio);

  NVIC_EnableIRQ (PORTA_IRQn);
  NVIC_EnableIRQ (PORTC_IRQn);
  NVIC_EnableIRQ (PIT0_IRQn);

  PIT_GetDefaultConfig (pit_config);
  PIT_Init (PIT, pit_config);
  PIT_EnableInterrupts (PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
  PIT_SetTimerPeriod (PIT, kPIT_Chnl_0, BUS_CLK);

  PIT_StartTimer (PIT, kPIT_Chnl_0);

  /* Force the counter to be placed into memory. */
  volatile static int i = 0;
  /* Enter an infinite loop, just incrementing a counter. */
  while (1)
    {
      i++;
    }
  return 0;
}
