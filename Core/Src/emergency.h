/*
 * emergency.h
 *
 *  Created on: 21 ott 2020
 *      Author: Moreno
 */

#ifndef SRC_EMERGENCY_H_
#define SRC_EMERGENCY_H_
/* Always included */
#include "stm32f4xx_hal.h"

static void __attribute__((noinline)) emergency(){

	RCC->AHB1ENR |= 1; // Enables GPIOA clock
	GPIOA->MODER &= ~0x0C00;
	GPIOA->MODER |=  0x0400;

	/* Configure the SysTick @ ~1 Hz
	 SysTick should be already initialized by HAL_Init()
	 We reload and blink forever.
	 */
	SysTick->LOAD = 0xFFFFFF;	// set SysTick to the maximum of 24 bits
	SysTick->CTRL = 5;			// enable it, no interrupt, use system clock of 16 MHz
	SysTick->VAL = 0; 			// clear current VALUE register

	while (1){
		GPIOA->ODR = (SysTick->VAL >> 18) & 0x20;
	}

	return;
}

#endif /* SRC_EMERGENCY_H_ */
