/*
 * stdlib.c
 *
 *  Created on: Sep 21, 2020
 *      Author: Moreno
 */
#include <string.h>
#include "stdLibrary.h"

inline void rampRCC(){

	RCC->AHB1ENR |= 1; /* Enables GPIOA clock */

	return;
}

inline void rampGPIOA(){

	GPIOA->MODER &= ~0x0C00;
	GPIOA->MODER |=  0x0400;

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

	rampRCC();
	rampGPIOA();

	/* Configure the TIM2 to clock @ ~1 Hz when SYSCLK =  16 MHz */
	RCC->APB1ENR |= 1;	// we supply the clock on the APB1ENR bus
	TIM2->PSC = 1600-1;	// divide SYSCLK by 1600
	TIM2->ARR = 10000-1;	// count up to 10000
	TIM2->CNT = 0;	// reset the counter
	TIM2->CR1 = 0x1; 	// switch on the counter

	while (1){
		while (!(TIM2->SR & 0x1)){}; 	// wait that the counter reaches TIM2->ARR
		TIM2->SR &= ~0x1;	// reset the status register
		ledOn();
		while (!(TIM2->SR & 0x1)){}; 	// wait that the counter reaches TIM2->ARR
		TIM2->SR &= ~0x1;	// reset the status register
		ledOff();
	}

	return;
}

inline void emergency(){

	rampRCC();
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

inline void delayMs(int ms){

	for (; ms > 0; ms--){
		for (int i=0; i < 12345; i++);
	}

	return;
}

inline void ledOn(){

	rampRCC();
	rampGPIOA();

	GPIOA->ODR |= 0x20;

	return;
}

inline void ledOff(){

	GPIOA->ODR &= ~0x20;

	return;
}

inline void addition(){

	int a = 12345678;
	int flag = 1;
	while(flag == 1){
		a--;
		if (a == 0) flag = 0;
	}

	return;
}

inline void addition1(int in1){

	uint32_t flag = 1;
	while (flag == 1){
		in1--;
		if (in1 == 0) flag = 0;
	}

	return;
}

inline void fp_addition1(float in1){

	uint32_t flag = 1;
	while (flag == 1){
		in1 = in1 - 1.0;
		if (in1 <= 0) flag = 0;
	}

	return;
}

inline void dfp_addition1(double in1){

	uint32_t flag = 1;
	while (flag == 1){
		in1 = in1 - 1.0;
		if (in1 <= 0) flag = 0;
	}

	return;
}


inline void addition2(int in1, int in2){

	int flag = 1;
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


unsigned int USART2_printf(const char *str){

	unsigned int l = strlen(str);
	//short int *end;
	for (unsigned int i=0; i < l; i++){
		USART2_write((int)str[i]);
	}
	/*
	USART2_write((int)'\r');
	USART2_write((int)'\n');
	*/
	return l;
}

__attribute__(( weak )) void USART2_write(int ch){

	while (!(USART2->SR & 0x80)) {}
	USART2->DR = (ch & 0xFF);

	return;
}
