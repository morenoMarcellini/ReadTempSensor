/*
 * USART2.c
 *
 *  Created on: 10 ott 2020
 *      Author: Moreno
 */

#include "UART4.h"

/* UART4 Helper
 *
 * UART4 is connected to pin PC10
 */

/* HELPERS */

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

int __attribute__((noinline)) waitUART4TX_IRQ(){

	return UART4TX_Irq;
}

/* This is used to debug */
void __attribute__(( weak )) UART4_init(){

	UART4_initTX();

	return;
}

void __attribute__(( weak,noinline )) UART4_initTX(){

	RCC->AHB1ENR |= 0x4; 		// Enables GPIOC clock
	RCC->APB1ENR |= 0x80000;	// Enables UART4 clock

	/* Configuration of PC10 for UART4_TX */
	/* Configure GPIOC */
	GPIOC->MODER &= ~0x300000;	// reset GPIOC MODER Pin 10
	GPIOC->MODER |=  0x200000;  // enable alternate function on PC10 = Pin 10 GPIOC
	// GPIOC->PUPDR |=  0x200000;	// pull down
	/* Configure Alternate function: */
	GPIOC->AFR[1] &= 0xF00;	// erase bits
	GPIOC->AFR[1] |= 0x800;	// set bits as alt8 for UART4_TX

	/* Configure UART4 */
	UART4->BRR = 0x0683;	// BAUD rate 9600 @16MHz
	UART4->CR1 = 0x0008;	// TX enabled @8 bits
	UART4->CR2 = 0x0000;	// 1 stop-bit
	UART4->CR3 = 0x0000; 	// no flow control
	UART4->CR1 |= 0x2000;	// UART4 finally enabled

	return;
}

void __attribute__(( weak, noinline ))  UART4_initRX(){

	RCC->AHB1ENR |= 0x4;		// GPIOC clock
	RCC->APB1ENR |= 0x80000;	// UART4 clock

	/* Configuration of PC11 for UART4_RX */
	/* Configure GPIOC */
	GPIOA->MODER &= ~0xC00000;	// erase bits
	GPIOA->MODER |= ~0x800000; 	// enable alternate function on PC11 = Pin GPIOC 11
	GPIOA->AFR[1] &= ~0xF000;	// erase bits
	GPIOA->AFR[1] |=  0x8000;	// set bits as alt8 for UART4_RX

	/* Configure UART4 */
	UART4->BRR = 0x0683;	// BAUD rate 9600 @16MHz
	UART4->CR1 = 0x0004;	// RX enabled @8 bits
	UART4->CR2 = 0x0000;	// 1 stop-bit
	UART4->CR3 = 0x0000; 	// no flow control
	UART4->CR1 |= 0x2000;	// UART4 finally enabled

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

void __attribute__(( weak, noinline )) UART4_openTXRX(uint32_t bbbb){
	/* This function open USART4 in TX/RX.
	 * The final function should be able to set the correct
	 * BAUD rate as function of the bus clock */
	uint32_t bus, cpu_clock;
	uint32_t baud = bbbb;
	Integer baud_q;

	baud = 9600;

	// uint32_t HAL_RCC_GetSysClockFreq() return the cpu clock;
	// uint32_t HAL_RCC_GetPCLK1Freq();	return the APB1ENR bus clock
	cpu_clock = HAL_RCC_GetSysClockFreq();
	bus = HAL_RCC_GetPCLK1Freq();
	/* baud rate is computed as, in pag. 95:
	 * baud = PLCK1Freq / {[8 * (2 - OVER8 )] * USARTDIV}
	 * USARTDIV (USART2->BRR) = PLCK1Freq / (16 * baud) */
	calcBRR(&baud_q, baud, bus);

	RCC->AHB1ENR |= 0x4; 		// Enables GPIOC clock
	RCC->APB1ENR |= 0x80000;	// Enables UART4 clock

	/* Configuration of PC10/PC11 for USART4_TX/RX */
	/* Configure GPIOC */
	GPIOC->MODER &= ~0xF00000;	// reset GPIOC MODER Pin 10/11
	GPIOC->MODER |=  0xA00000;  // enable alternate function on PC10/PC11
	GPIOC->AFR[1] &= ~0xFF00;	// erase bits
	GPIOC->AFR[1] |=  0x8800;	// set bit as alt7 for UART4 TX/RX

	/* Configure USART4 */
	UART4->BRR = 0x0683;	// BAUD rate 9600 @16MHz
	UART4->CR1 = 0x000C;	// TX/RX enabled @8 bits
	UART4->CR2 = 0x0000;	// 1 stop-bit
	UART4->CR3 = 0x0000; 	// no flow control
	UART4->CR1 |= 0x2000;	// UART4 finally enabled

#if 0
	char str[64];
	sprintf(str, "%ld %ld \r\n", baud_q.unit, baud_q.decimal);
	USART2_printf(str);
	sprintf(str, "%ld %ld \r\n", cpu_clock, bus);
	USART2_printf(str);
#endif

	return;
}

/* Internal function hidden to main */
void __attribute__(( weak )) UART4_write(int32_t ch){

	while (!(UART4->SR & 0x80)) {};
	UART4->DR = (ch & 0xFF);

	return;
}

unsigned int __attribute__((noinline)) UART4_printf(const char* str){

	uint32_t l = strlen(str);

	for (unsigned int i=0; i < l; i++){
		UART4_write((int)str[i]);
	}

	return l;
}

/* Read one character at the time */
char UART4_read(){

	while(!(UART4->SR & 0x20)) {};

	return UART4->DR;
}

inline void UART4_TXon(){

	UART4->CR1 |= 0x2000;	// UART4 finally enabled

	return;
}

inline void UART4_TXoff(){

	UART4->CR1 &= ~0x2000;	// UART4 finally disabled

	return;
}

void fakeUART4TX(char in){

	char t = 0x1, swp;
	uint32_t tick;

	tick = HAL_GetTick();

	for (int i=0; i<8; i++){
		/* to switch on */
		swp = in & t;
		if (swp == 1){
			GPIOC->BSRR = 0x800;
		}
		else {
			GPIOC->BSRR = 0x8000000;
		}
		in = in >> 1;
		while (tick == HAL_GetTick()) {};
		tick = HAL_GetTick();
	}
}

/*  DMA version
 *	DMA1, channel 4, stream 4, for UART4TX
 * 	DMA1, channel 4, stream 2, for UART4RX
 */

/* In order after UART4_initTX: 1. UART4_IRQ_initTX 2. UART4_DMA1_initTX */
void __attribute__((noinline)) UART4_IRQ_initTX(){

	UART4->SR &= ~0x40;	 	// clear TC = transmit complete flag
	UART4->CR1 |= 0x40;		// enable transmit complete interrupt

	NVIC_EnableIRQ(UART4_IRQn);	// UART4 Interrupt enable at NVIC

	return;
}

void __attribute__((noinline)) UART4_DMA1_initTX(){

	RCC->AHB1ENR |= 0x00200000; 		// clock to DMA controller
	DMA1->HISR = 0x3D;					// set all the interrupt flags of stream 4
	NVIC_EnableIRQ(DMA1_Stream4_IRQn);	// DMA interrupt enabled at NVIC

	return;
}

/* Set up a DMA TX from UART4
 * from manual (page 207) UART4TX is connected to DMA1-S4 */

void __attribute__((noinline)) UART4_DMA1_write(unsigned int src, unsigned int dst, unsigned int len){

	DMA1_Stream4->CR &= ~0x1; 		// We first disable the DMA1-S4
									// because we cannot write if bit EN !=0
	while (DMA1_Stream4->CR & 1){}; // We wait that DMA1-S4 is off

	DMA1->HIFCR = 0x3D;		 		// we reset all the interrupt of DMA1-S4

	DMA1_Stream4->PAR = dst;		// PAR contains the address of destination
	DMA1_Stream4->M0AR = src;		// M0AR contains the address of source
	DMA1_Stream4->NDTR = len;		// NDTR contains the lenght of item to transfer, bytes in this case

	/* in the book CR is equivalent to DMA_SxCR */
	DMA1_Stream4->CR  = 0x08000000; // UART4TX is on channel 4
	DMA1_Stream4->CR |= 0x00000440;	// data size, mem incr, mem-to-periph.
									// 8 bits     1 element mem->periph
	DMA1_Stream4->CR |= 0x16;		// raising TCIE, TEIE, DMEIE
	DMA1_Stream4->FCR = 0x0;		// no FIFO, direct mode
	DMA1_Stream4->CR |= 0x1;		// we finally restart the DMA
									// we set bit EN to 1
	/* For UART4TX */
	UART4->SR &= ~0x40;				// we reset the UART4 transmit complete interrupt flag
	UART4->CR3 |= 0x80;				// enable UART4TX transmitter DMA

	return;
}

/* to use the same syntax as in pooling */
void __attribute__((noinline)) UART4_DMA1_printf(unsigned int *src){

	unsigned int dst, len;

	dst = (unsigned int)&UART4->DR;
	len = strlen((char*)src);

	UART4_DMA1_write((unsigned int)src, dst, len);

	return;
}

/* IRQ Handlers */
/* DMA1 Stream 4 interrupt handler */
void __attribute__((noinline)) DMA1_Stream4_IRQHandler(){

	if (DMA1->HISR & 0x0D){	// error occurred
		while (1){
			RCC->AHB1ENR = 0x0; // Total switch off
		}
	}

	DMA1->HIFCR = 0x3D; 		// clean up all the interrupt for DMA1-S4
	DMA1_Stream4->CR &= ~0x10;	// disable DMA1-S4 TCIE

	return;
}

#if 0
void __attribute__((noinline)) UART4_IRQHandler(){
/* this IRQHandler is the simplest one
 * to catch the end of transmission: at the end of
 * file you find the generic one to catch the RX, and amount of trasmitted data
*/

		UART4->SR &= ~0x40;		// Clear transmit complete interrupt flag
		UART4TX_Irq = 1;		// Return for end-of-transfer

	return;
}
#endif

/*
 *
 * UART4_RX DMA1 Version
 *
 */

/* In order after UART4_initRX: 1. UART4_IRQ_initRX 2. UART4_DMA1_initRX */
void __attribute__((noinline)) UART4_IRQ_initRX(){

	UART4->SR &= ~0x20;	 	// clear RXNE bit = receive-complete flag
	UART4->CR1 |= 0x20;		// enable receive-complete interrupt
	UART4->CR1 |= 0x10;		// IDLE interrupt enable:
							// An USART interrupt is generated whenever IDLE=1 in the USART_SR register

	NVIC_EnableIRQ(UART4_IRQn);	// UART4 Interrupt enable at NVIC

	return;
}

void __attribute__((noinline)) UART4_DMA1_initRX(){

	RCC->AHB1ENR |= 0x00200000; 		// clock to DMA controller
	DMA1->LISR = 0xF00;					// set all the interrupt flags of stream 2
	NVIC_EnableIRQ(DMA1_Stream2_IRQn);	// DMA interrupt enabled at NVIC

	return;
}

/* Set up a DMA RX from UART4
 * from manual (page 207) UART4RX is connected to DMA1-S2 */

uint32_t __attribute__((noinline)) getDMA1_Stream2(){

	return DMA1_Stream2->CR;
}

void __attribute__((noinline)) UART4_DMA1_read(unsigned int src, unsigned int dst[2], unsigned int len){	//	TODO

	DMA1_Stream2->CR &= ~0x1; 		// We first disable the DMA1-S2
									// because we cannot write if bit EN (bit 1)!=0
	while (getDMA1_Stream2() & 1){};	// We wait that DMA1-S2 is off

	DMA1->LIFCR = 0xF00;		 	// we reset all the interrupt of DMA1-S2

	DMA1_Stream2->PAR = src;		// PAR contains the address of destination
	DMA1_Stream2->M0AR = dst[0];	// M0AR contains the address of source
	DMA1_Stream2->M1AR = dst[1];	// because we want to use double buffer, we need 2 dst
	DMA1_Stream2->NDTR = len;		// NDTR contains the length of item to transfer, bytes in this case
#if 0
	/* in the book CR is equivalent to DMA_SxCR */
	DMA1_Stream2->CR  = 0x04000000; // UART4RX is on channel 2
	DMA1_Stream2->CR |= 0x00004000;	// Double-buffer
	DMA1_Stream2->CR |= 0x00000000;	// data size 8 bits
	DMA1_Stream2->CR |= 0x00000400;	// mem incr 1
	DMA1_Stream2->CR |= 0x00000100; // circular mode
	DMA1_Stream2->CR |= 0x00000000;	// mem-to-periph.
	DMA1_Stream2->CR |= 0x0000001E;	// raising TCIE, HTIE, TEIE, DMEIE
#endif
	DMA1_Stream2->CR  = 0x0400451E;	// see above
	DMA1_Stream2->FCR = 0x0;		// no FIFO, direct mode
	DMA1_Stream2->CR |= 0x1;		// we finally restart the DMA
									// we set bit EN to 1
	/* For UART4RX */
	UART4->SR &= ~0x40;				// we reset the UART4 transmit complete interrupt flag
	UART4->CR3 |= 0x80;				// enable UART4RX transmitter DMA

	return;
}

/* to use the same syntax as in pooling */
void __attribute__((noinline)) UART4_DMA1_scanf(unsigned int *src){	//	NOT NEEDED
#if 0
	unsigned int dst, len;

	dst = (unsigned int)&UART4->DR;
	len = strlen((char*)src);

	UART4_DMA1_read((unsigned int)src, dst, len);
#endif
	return;
}

/* IRQ Handlers */
/* DMA1 Stream 2 interrupt handler */
void __attribute__((noinline)) DMA1_Stream2_IRQHandler(){

	if (DMA1->LISR & 0xD0000){	// error occurred
		while (1){
			RCC->AHB1ENR = 0x0; // Total switch off
		}
	}
	if (DMA1->LISR & 0x100000 ){	// full transfer done
		UART4TX_Irq = 1;			// return end-of-transfer
	}

	DMA1->LIFCR = 0x7D0000; 	// clean up all the interrupt for DMA1-S2
	DMA1_Stream2->CR &= ~0x10;	// disable DMA1-S2 TCIE

	return;
}

void UART4_RX_check(){

	while (DMA1_Stream2->NDTR > 0) {};	// full DMA is transferred

#if 0
/* case from
https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx/blob/master/projects/usart_rx_idle_line_irq_F4/Src/main.c
*/
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /* We are in "linear" mode */
            /* Process data directly by subtracting "pointers" */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            /* We are in "overflow" mode */
            /* First process data to the end of buffer */
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            /* Check and continue with beginning of buffer */
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
    }
    old_pos = pos;                              /* Save current position as old */

    /* Check and manually update if we reached end of buffer */
    if (old_pos == ARRAY_LEN(usart_rx_dma_buffer)) {
        old_pos = 0;
    }
#endif
	return;
}

#if 1
void __attribute__((noinline)) UART4_IRQHandler(){

		if (UART4->CR1 & 0x10){	// we catch IDLE flag
			UART4_RX_check();			// wait until full transfer is done
		}

		if (UART4->SR & 0x40){	// we catch transmit complete interrupt flag
			UART4->SR &= ~0x40;			// we clear transmit complete interrupt flag
		}

		UART4TX_Irq = 1;		// Return for end-of-transfer

	return;
}
#endif
