/*
 * DMA-Collector.h
 *
 *  Created on: 8 ott 2020
 *      Author: Moreno
 */

#ifndef SRC_DMA_COLLECTOR_H_
#define SRC_DMA_COLLECTOR_H_

/* The next header is always included */
#include <stm32f446xx.h>

/* Local variable definition*/
static int32_t USART2TX_DMA1_Interrupt = 1;

/* Functions definition */
void DMA1_USART2TX_init();
void DMA1_USART2TX_write();
void DMA1_Stream6_IRQHandler();
void USART2_IRQHandler();
void USART2TX_Interrupt_init();
#endif /* SRC_DMA_COLLECTOR_H_ */
