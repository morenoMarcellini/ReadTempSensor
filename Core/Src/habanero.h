/*
 * stdlib.h
 *
 *  Created on: Sep 21, 2020
 *      Author: Moreno
 */

#ifndef HABANERO_H_
#define HABANERO_H_

#ifdef __cplusplus
extern "C" {
#endif
/* Always included */
#include "stm32f4xx_hal.h"
#include <math.h>
#ifdef __cplusplus
}
#endif

/* DEBUG */
uint32_t debug;

/* definition and so on */
typedef struct S_ {
		int32_t unit;
		int32_t decimal;
		int32_t null1;
		int32_t null2;
} Integer;

void rampRCC();
void delayMs(int);
void rampGPIOA();
void ledOn();
void ledOff();
void blinkAtInit();
void blink();

/* for test cases*/
void addition();
void addition1(uint32_t);
void addition2(int, int);
void fp_addition1(float);
void dfp_addition1(double);

/* for serial port USART2 */
void USART2_init(/* 9600 baud */);
void USART2_open(/* 9600 baud */);
void USART2_initTX(/* 9600 baud */);
void USART2_initRX(/* 9600 baud */);
void USART2_write(int32_t);
char USART2_read();
unsigned int USART2_printf(const char*);

/* TIMER and SysClock */
void emergency();
void blinkForever();

/* Diverse */
void floatToInt(float, Integer*);
#endif /* STDLIB_H_ */
