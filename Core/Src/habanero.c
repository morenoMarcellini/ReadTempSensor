/*
 * habanero.c:
 *
 * A collection of randomly dstributed functions and helpers
 *
 *  Created on: Sep 21, 2020
 *      Author: Moreno
 */

#include "habanero.h"
#include <stdio.h>
#include <string.h>
#include "emergency.h"

/* LED2 HELPERS */
inline void rampRCCA(){

	RCC->AHB1ENR |= 1; /* Enables GPIOA clock */

	return;
}

inline void rampRCCC(){

	RCC->AHB1ENR |= 0x4; // Enables GPIOC clock

	return;
}

inline void rampGPIOA(){

	RCC->AHB1ENR |= 1; 			// Enables GPIOA clock
	GPIOA->MODER &= ~0x0C00;
	GPIOA->MODER |=  0x0400;

	return;
}

inline void rampGPIOC(){

	RCC->AHB1ENR |= 0x4;		// Enables GPIOC clock
#if 0
	GPIOC->MODER &= ~0x300000;	// pin 10 reset
	GPIOC->MODER |=  0x100000;	// pin 10 general-purpose
#else
	GPIOC->MODER &= ~0x300000;	// pin 10 reset
	GPIOC->MODER |=  0x200000;	// pin 10 alternate function
#endif

#if 0
	GPIOC->MODER &= ~0xC00000;	// pin 11 reset
	GPIOC->MODER |=  0x400000;	// pin 11 general-purpose
#else
	GPIOC->MODER &= ~0xC00000;	// pin 11 reset
	GPIOC->MODER |=  0x800000;	// pin 11 alternate function
#endif
	return;
}

/* Functions for leds */
inline void ledOnExt10(){

	GPIOC->ODR |= 0x400;

	return;
}

inline void ledOffExt10(){

	GPIOC->ODR &= ~0x400;

	return;
}

inline void ledOnExt11(){
#if 0
	GPIOC->ODR |= 0x400;
#else
	GPIOC->BSRR |= 0x800;		// turns on bit on pin 11
#endif
	return;
}

inline void ledOffExt11(){
#if 0
	GPIOC->ODR &= ~0x400;
#else
	GPIOC->BSRR = 0x8000000;	// turns off bit on pin 11
#endif
	return;
}

inline void blinkAtInit(){

	for (uint32_t i=0; i < 5; i++){
		ledOn();
		addition1(60000001);
		ledOff();
	}

	return;
}

inline void blink(){

	uint32_t q = 123456;

	for (int i=0; i < 5; i++){
		ledOn();
		while (q!=0){
			q--;
		}
		q = 1234567;
		ledOff();
	}

	q = 1234567;
	ledOff();
	while (q!=0){
		q--;
	}
	return;
}

inline void blinkForever(){

	// rampRCCA();
	rampGPIOA();

	/* Configure the TIM2 to clock @ ~1 Hz when SYSCLK =  16 MHz */
	RCC->APB1ENR |= 1;		// we supply the clock on the APB1ENR bus
	TIM2->PSC = 1600-1;		// divide SYSCLK by 1600
	TIM2->ARR = 10000-1;	// count up to 10000
	TIM2->CNT = 0;			// reset the counter
	TIM2->CR1 = 0x1; 		// switch on the counter

	while (1){
		while (!(TIM2->SR & 0x1)){}; 	// wait that the counter reaches TIM2->ARR
		TIM2->SR &= ~0x1;				// reset the status register
		ledOn();
		while (!(TIM2->SR & 0x1)){}; 	// wait that the counter reaches TIM2->ARR
		TIM2->SR &= ~0x1;				// reset the status register
		ledOff();
	}

	return;
}

#if 0
inline void emergency(){

	// rampRCCA();
	rampGPIOA();

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
#endif

inline void delayMs(int ms){

	for (; ms > 0; ms--){
		for (int i=0; i < 12345; i++);
	}

	return;
}

inline void ledOn(){

	// rampRCCA();
	rampGPIOA();

	GPIOA->ODR |= 0x20;

	return;
}

inline void ledOff(){

	GPIOA->ODR &= ~0x20;

	return;
}


/* usefull functions if you need to delay */
inline void addition(){

	uint32_t a = SystemCoreClock;
	int32_t flag = 1;
	while(flag == 1){
		a--;
		if (a == 0) flag = a;
	}

	return;
}

void __attribute__((noinline)) addition1(uint32_t in1){

	for (uint32_t i=0; i < in1; i++){
		--in1;
	}

	return;
}

inline void addition2(int in1, int in2){

	int32_t flag = 1;
	if (in1 <= in2){
		while (flag){
			in1++;
			if (in1 == in2) flag = 0;
		}
	}
	else {
		while (flag){
			in2++;
			if (in1 == in2) flag = 0;
		}
	}

	return;
}
/* Floating point version */
inline void fp_addition(){

	int32_t flag = 1;
	float T;

	T = (float) SystemCoreClock;

	while (flag == 1){
		T = T - 1.0f;
		if (T <= 0.0f) flag = 0;
	}

	return;
}

inline void fp_addition1(float in1){

	int32_t flag = 1;
	while (flag == 1){
		in1 = in1 - 1.0f;
		if (in1 <= 0.0f) flag = 0;
	}

	return;
}

#if 0 /* Only single precision float */
inline void dfp_addition1(double in1){

	int32_t flag = 1;
	while (flag == 1){
		in1 = in1 - 1.0;
		if (in1 <= 0) flag = 0;
	}

	return;
}
#endif


/* USART2 Helper */
__attribute__(( weak )) void USART2_init(){

	RCC->AHB1ENR |= 1;			// GPIOA clock for led
	RCC->APB1ENR |= 0x20000;	// USART2 clock

	/* Configuration of PA2 for USART2_TX */
	/* Configure GPIOA */
	GPIOA->AFR[0] &= ~0x0F00;
	GPIOA->AFR[0] |=  0x0700;
	GPIOA->MODER &= ~0x0030;
	GPIOA->MODER |= 0x0020; // enable alternate function on PA2 = Pin GPIOA 2
	/* Configure USART2 */
	USART2->BRR = 0x0683;	// BAUD rate 9600 @16MHz
	USART2->CR1 = 0x0008;	// TX enabled @8 bits
	USART2->CR2 = 0x0000;	// 1 stop-bit
	USART2->CR3 = 0x0000; 	// no flow control
	USART2->CR1 |= 0x2000;	// USART2 finally enabled

	return;
}

__attribute__(( weak )) void USART2_initTX(){

	RCC->AHB1ENR |= 1;			// GPIOA clock for led
	RCC->APB1ENR |= 0x20000;	// USART2 clock

	/* Configuration of PA2 for USART2_TX */
	/* Configure GPIOA */
	GPIOA->AFR[0] &= ~0x0F00;	// erase bit
	GPIOA->AFR[0] |=  0x0700;	// set bit as alt7 for USART2
	GPIOA->MODER &= ~0x0030;
	GPIOA->MODER |= 0x0020; // enable alternate function on PA2 = Pin GPIOA 2
	/* Configure USART2 */
	USART2->BRR = 0x0683;	// BAUD rate 9600 @16MHz
	USART2->CR1 = 0x0008;	// TX enabled @8 bits
	USART2->CR2 = 0x0000;	// 1 stop-bit
	USART2->CR3 = 0x0000; 	// no flow control
	USART2->CR1 |= 0x2000;	// USART2 finally enabled

	return;
}

__attribute__(( weak )) void USART2_initRX(){

	RCC->AHB1ENR |= 1;			// Enable GPIOA clock
	RCC->APB1ENR |= 0x20000;	// Enable USART2 clock

	/* Configuration of PA3 for USART2_RX */
	/* Configure GPIOA */
	GPIOA->AFR[0] &= ~0xF000;	// erase bits
	GPIOA->AFR[0] |=  0x7000;	// set bits as alt7 for USART2
	GPIOA->MODER &= ~0x00C0;
	GPIOA->MODER |= 0x0080; // enable alternate function on PA3 = Pin GPIOA 3
	/* Configure USART2 */
	USART2->BRR = 0x0683;	// BAUD rate 9600 @16MHz
	USART2->CR1 = 0x0004;	// RX enabled @8 bits
	USART2->CR2 = 0x0000;	// 1 stop-bit
	USART2->CR3 = 0x0000; 	// no flow control
	USART2->CR1 |= 0x2000;	// USART2 finally enabled

	return;
}

static uint32_t calcBRR(Integer *brr, uint32_t baud, uint32_t clock){

		const float mult = 16.0f;
		float res;
		uint32_t tmp;

		tmp = clock >> 4;
		res = (float)(tmp) / ((float)baud);

		brr->unit = (int32_t)(res);
		brr->decimal = (int32_t)((res-(float)brr->unit)*mult + 0.5f);

		tmp = (brr->unit << 4) | brr->decimal;
		debug = tmp;
		return tmp;

}
__attribute__(( weak )) void USART2_open(){
	/* This function open USART2 in TX/RX.
	 * The final function should be able to set the correct
	 * BAUD rate as function of the bus clock */
	uint32_t bus, cpu_clock;
	uint32_t baud = 9600;
	Integer baud_q;

	// uint32_t HAL_RCC_GetSysClockFreq() return the cpu clock;
	// uint32_t HAL_RCC_GetPCLK1Freq();	return the APB1ENR bus clock
	cpu_clock = HAL_RCC_GetSysClockFreq();
	bus = HAL_RCC_GetPCLK1Freq();
	/* baud rate is computed as, in pag. 95:
	 * baud = PLCK1Freq / {[8 * (2 - OVER8 )] * USARTDIV}
	 * USARTDIV (USART2->BRR) = PLCK1Freq / (16 * baud) */
	calcBRR(&baud_q, baud, bus);

	RCC->AHB1ENR |= 1;			// GPIOA clock for led
	RCC->APB1ENR |= 0x20000;	// USART2 clock

	/* Configuration of PA2 for USART2_TX */
	/* Configure GPIOA */
	GPIOA->AFR[0] &= ~0xFF00;	// erase bits
	GPIOA->AFR[0] |=  0x7700;	// set bit as alt7 for USART2 TX/RX
	GPIOA->MODER &= ~0x00F0;
	GPIOA->MODER |= 0x00A0; // enable alternate function on PA2 = Pin GPIOA 2 and PA3 = Pin GPIOA 3
	/* Configure USART2 */
	USART2->BRR = 0x0683;	// BAUD rate 9600 @16MHz
	USART2->CR1 = 0x000C;	// TX/RX enabled @8 bits
	USART2->CR2 = 0x0000;	// 1 stop-bit
	USART2->CR3 = 0x0000; 	// no flow control
	USART2->CR1 |= 0x2000;	// USART2 finally enabled

#if 0
	char str[64];
	sprintf(str, "%ld %ld \r\n", baud_q.unit, baud_q.decimal);
	USART2_printf(str);
	sprintf(str, "%ld %ld \r\n", cpu_clock, bus);
	USART2_printf(str);
#endif

	return;
}

unsigned int USART2_printf(const char* str){

	uint32_t l = strlen(str);

	for (unsigned int i=0; i < l; i++){
		USART2_write((int)str[i]);
	}
	/*
	USART2_write((int)'\r');
	USART2_write((int)'\n');
	*/
	return l;
}

__attribute__(( weak )) void USART2_write(int32_t ch){

	while (!(USART2->SR & 0x80)) {}
	USART2->DR = (ch & 0xFF);

	return;
}

/* Read one character at the time */
char USART2_read(){

	while(!(USART2->SR & 0x20)) {};

	return USART2->DR;
}

void floatToInt(float f, Integer *ret){

	float mult = 1000.0f;

	ret->unit = (int32_t)(f);
	ret->decimal = (int32_t)((f-(float)ret->unit)*mult);

	return;
}

/* some stuff */
int rand(int* seed)
{
  seed[0] = seed[0]*0x343FD+0x269EC3;  // a=214013, b=2531011
  return (seed[0] >> 0x10) & 0x7FFF;
}
