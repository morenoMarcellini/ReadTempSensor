/*
 * stdlib.h
 *
 *  Created on: Sep 21, 2020
 *      Author: Moreno
 */

#ifndef STDLIBRARY_H_
#define STDLIBRARY_H_

#ifdef __cplusplus
extern "C" {
#endif
/* Always included */
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
}
#endif

void rampRCC();
void delayMs(int);
void rampGPIOA();
void ledOn();
void ledOff();
void blinkAtInit();
void blink();

/* for test cases*/
void addition();
void addition1(int);
void addition2(int, int);
void fp_addition1(float);
void dfp_addition1(double);

/* for serial port USART2 */
void USART2_init();
void USART2_write(int);
unsigned int USART2_printf(const char*);

/* TIMER and SysClock */
void emergency();
void blinkForever();
#endif /* STDLIB_H_ */
