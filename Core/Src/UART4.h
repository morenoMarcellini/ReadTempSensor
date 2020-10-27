/*
 * USART2.h
 *
 *  Created on: 10 ott 2020
 *      Author: Moreno
 */

#ifndef SRC_UART4_H_
#define SRC_UART4_H_

/* Always included */
#include "stm32f4xx_hal.h"
#include "stdLibrary.h"
#include <math.h>
#include <string.h>

/* definition and so on */
typedef struct S_ {
		int32_t unit;
		int32_t decimal;
		int32_t null1;
		int32_t null2;
} Integer;


/* Global variable definition*/
int32_t UART4TX_Irq;
uint32_t debug;

/* for serial port UART4 */

void UART4_init();
void UART4_initTX();
void UART4_initRX();
static uint32_t calcBRR(Integer *brr, uint32_t baud, uint32_t clock);
void UART4_openTXRX(uint32_t);
void UART4_TXon();
void UART4_TXoff();
/* Internal function hidden to main */
void UART4_write(int32_t ch);
unsigned int UART4_printf(const char* str);
/* Read one character at the time */
char UART4_read();
void fakeUART4TX(char);

/* DMA version
 * DMA1, channel 4, stream 4, for UART4TX
 * DMA1, channel 4, stream 2, for UART4RX
 */
/* Set up a DMA TX from USART2
 * from manual (page 207) UART4_TX is connected to DMA1-S4 */
void UART4_DMA1_initTX();
void UART4_DMA1_write(unsigned int src, unsigned int dst, unsigned int len);
/* to use the same syntax as in pooling */
void UART4_DMA1_printf(unsigned int *src);
/* DMA1 Stream 4 interrupt handler */
void DMA1_Stream4_IRQHandler();
void UART4_IRQHandler();
void UART4_IRQ_initTX();
int  waitUART4TX_IRQ();

#if 0 /* Hidden function to user */
#endif

#endif /* SRC_UART4_H_ */
