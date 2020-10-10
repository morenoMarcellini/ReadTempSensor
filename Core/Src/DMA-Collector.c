/*
 * DMA-Collector.c
 *
 *  Created on: 8 ott 2020
 *      Author: Moreno
 */

#include <stm32f446xx.h>
#include "habanero.h"
#include "DMA-Collector.h"

/*	Initialize the DMA1, stream 6, where the USART2TX is connected */
void DMA1_USART2TX_init(){

	RCC->AHB1ENR |= 0x00200000; 		// clock to DMA controller
	DMA1->HISR = 0x003F0000;			// clear all the interrupt flags of stream 6
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);	// DMA interrupt enabled at NVIC

	return;
}

/* Set up a DMA TX from USART2
 * from manual (page 207) USART2TX is connected to DMA1-6 */

void DMA1_USART2TX_write(unsigned int src, unsigned int dst, unsigned int len){

	DMA1_Stream6->CR &= ~0x1; 		/* We first disable the DMA1-S6 */
	while (DMA1_Stream6->CR & 1){}; // We wait that DMA1-S6 is off

	DMA1->HIFCR = 0x003F0000; 		// we reset all the interrupt of DMA1-S6

	DMA1_Stream6->PAR = dst;		// PAR contains the address of destination
	DMA1_Stream6->M0AR = src;		// M0AR contains the address of source
	DMA1_Stream6->NDTR = len;		// NDTR contains the lenght of item to transfer, bytes in this case

	/* in the book CR is equivalent to DMA_SxCR */
	DMA1_Stream6->CR  = 0x08000000; // USART2TX is on channel 4 of stream 6
	DMA1_Stream6->CR |= 0x00000440;	// data size, mem incr, mem-to-periph.
	DMA1_Stream6->CR |= 0x16;		// raising TCIE, TEIE, DMEIE
	DMA1_Stream6->FCR = 0;			// no FIFO, direct mode
	DMA1_Stream6->CR |= 0x1;		// we finally restart the DMA
	/* For USART2TX */
	USART2->SR &= ~0x40;			// we reset the USART2 transmit complete interrupt flag
	USART2->CR3 |= 0x80;			// enable USART2TX transmitter DMA

	return;
}

/* DMA1 Stream 6 interrupt handler */
void DMA1_Stream6_IRQHandler(){

	if (DMA1->HIFCR & 0x000C0000){ /* error occurred */
		while (1){
			emergency();
		}
	}

	DMA1->HIFCR = 0x3F0000; 	// clean up all the interrupt for DMA1-S6
	DMA1_Stream6->CR &= ~0x10;	// disable DMA1-S6 TCIE

	return;
}

void USART2_IRQHandler(){

	USART2->SR &= ~0x40;		// Clear transmit complete interrupt flag
	USART2TX_DMA1_Interrupt = 1;

	return;
}

void USART2TX_Interrupt_init(){

	USART2->SR &= ~0x40; 	// clear TC flag
	USART2->CR1 |= 0x40;	// enable transmit complete interrupt

	NVIC_EnableIRQ(USART2_IRQn);	// USART2 Interrupt enable at NVIC

	return;
}
