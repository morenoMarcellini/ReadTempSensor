/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "habanero.h"

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

#if 0 /* if 0 it will compile the USART2-DMA version */
int main(void)
{
  int32_t data, unit, decimal;
  uint32_t T;
  float temp, volt;
  const float multiplier = 3.3f/4095.0f;
  const float divider = 1.0f/0.0025f;
  char str[128], C;
  Integer res;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /*  Setup for LED */
  RCC->AHB1ENR |= 1;
  ledOn();
  addition1(SystemCoreClock); // pause
  /* Setup TIM2 for triggering the reading */
  RCC->APB1ENR |= 1;	// clock on APB1ENR bus
  TIM2->PSC = 1600 - 1; // clock 16MHz divided by 1600
  TIM2->ARR = 10000 - 1; /* Autoreload, or maximum value of the counter:
   	   	   	   	   	   	   we have divided the clock by 1600, therefore
   	   	   	   	   	   	   1 s (1 Hz) is equivalent to 10000 internal ticks  */
  TIM2->CNT = 0;		// we reset the count to zero.
  /* We generate the trigger for ADC1 */
  TIM2->CCMR1 = 0x6800; // pwm1  mode, preload enable, define the waveform
  TIM2->CCER = 0x10; 	// ch2 enable
  TIM2->CCR2 = 50-1;	// When the CCR2 and the CNT are equal the ch2 perform an action
  TIM2->CR1 = 1;		// TIM2 starts counting

  ledOff();

  /* Setup ADC1 */
  RCC->APB2ENR |= 0x0100;	// enable ADC1 clock
  /* Turn on the temperature sensor */
  ADC->CCR |= 0x800000; /*!< ADC common control register */
  ADC->CCR &= ~0x400000;
  /* Turn on the A--->D converter */
  ADC1->SMPR1 = 0x4000000;	// sampling time mininum 10 us
  ADC1->SQR3 = 18;	// channel 18th is the temperature sensor
  ADC1->CR2 = 0x13000000; 	// trigger: EXTEN rising edge, EXTSEL 3 = TIM2.2
  ADC1->CR2 |= 1;	// enable ADC1

  /* Initialize USART2 */
  USART2_open();
  USART2_printf("Welcome on board\r\n");

  sprintf(str, "System Bus Clock = %lu\r\n", SystemCoreClock);
  USART2_printf(str);

#if 1
  T = HAL_RCC_GetSysClockFreq();
  sprintf(str, "CPU Clock = %lu\r\n", T);
  USART2_printf(str);
  T = HAL_RCC_GetPCLK1Freq();
  sprintf(str, "Serial bus Clock = %lu\r\n", T);
  USART2_printf(str);
#endif

  USART2_printf("Press a character to start\r\n");
  /* We init in exclusive RX */
  C = USART2_read();

  /* We re-init again in exclusive TX */
  sprintf(str, "%c 0x%x\r\n", C, debug);
  USART2_printf(str);

//  data = TIM2->CNT;		// do I counted out?

  while (1)
  {
	  ledOff();
	  USART2_printf("READY\r\n");
	  while (!(ADC1->SR & 0x2)){
#if 0
		  ledOff();
		  addition();
		  ledOn();
		  fp_addition1(16000000.0f);
#endif
	  };
	  data = ADC1->DR;
	  volt = (float)data * multiplier;
	  /* Temperature (in °C) = {(VSENSE – V25) / Avg_Slope} + 25 */
	  /* V25 = 0.76V, slope = 2.5 mV/C */
	  temp = 25.0f + (volt-0.76f) * divider;
	  floatToInt(temp, &res);
#if 0
	  sprintf(str, "READ %ld, %ld.%ld\r\n", data, res.unit, res.decimal);
#else
	  sprintf(str, "Temperature of CPU  %ld.%ld Celsius\r\n", res.unit, res.decimal);
#endif
	  USART2_printf(str);
	  ledOn();
#if 0
	  unit = (int) temp;
	  temp = temp -(float)unit;
	  temp = temp*1000.0f;
	  decimal = (int)temp;
	  sprintf(str, "READ %ld, %ld.%ld\r\n", data, unit, decimal);
	  USART2_printf(str); //"READ\r\n");
#endif
	  fp_addition1((float)(T)); // to wait longer
  }

}

#else /* DMA VERSION */

#include "DMA-Collector.h"
char str[] = "WaDgo057F9AuOzXeu4EwTgv6s0eJDeiiPQDFZdZWhz8gqp0u3XRbNgcWDI8fzWD7WaDgo057F9AuOzXeu4EwTgv6s0eJDeiiPQDFZdZWhz8gqp0u3XRbNgcWDI8fzWD7\r\n";
char alp[] = "abcdefghijklmnopqrstuwyxzABCDEFGHIJKLMNOPQRSTUVWYXZabcdefghijklmnopqrstuwyxzABCDEFGHIJKLMNOPQRSTUVWYXZabcdefghijklmnopqrstuwyxzA";

int main(void)
{
  int32_t data, unit, decimal;
  uint32_t T, size;

  float temp, volt;
  const float multiplier = 3.3f/4095.0f;
  const float divider = 1.0f/0.0025f;
  char C;
  Integer res;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /*  Setup for LED */
  RCC->AHB1ENR |= 1;
  ledOn();
  addition(); // pause
  /* Setup TIM2 for triggering the reading */
  RCC->APB1ENR |= 1;	// clock on APB1ENR bus
  TIM2->PSC = 1600 - 1; // clock 16MHz divided by 1600
  TIM2->ARR = 10000 - 1; /* Autoreload, or maximum value of the counter:
   	   	   	   	   	   	   we have divided the clock by 1600, therefore
   	   	   	   	   	   	   1 s (1 Hz) is equivalent to 10000 internal ticks  */
  TIM2->CNT = 0;		// we reset the count to zero.
  /* We generate the trigger for ADC1 */
  TIM2->CCMR1 = 0x6800; // pwm1  mode, preload enable, define the waveform
  TIM2->CCER = 0x10; 	// ch2 enable
  TIM2->CCR2 = 50-1;	// When the CCR2 and the CNT are equal the ch2 perform an action
  TIM2->CR1 = 1;		// TIM2 starts counting

  ledOff();

  /* Setup ADC1 */
  RCC->APB2ENR |= 0x0100;	// enable ADC1 clock
  /* Turn on the temperature sensor */
  ADC->CCR |= 0x800000; 	// ADC common control register
  ADC->CCR &= ~0x400000;
  /* Turn on the A--->D converter */
  ADC1->SMPR1 = 0x4000000;	// sampling time mininum 10 us
  ADC1->SQR3 = 18;			// channel 18th is the temperature sensor
  ADC1->CR2 = 0x13000000; 	// trigger: EXTEN rising edge, EXTSEL 3 = TIM2.2
  ADC1->CR2 |= 1;			// enable ADC1

  /* Initialize USART2 only in TX @9600 baud */
  USART2_initTX();
  /* Because we transfer via DMA1-S6 we add an extra call */
  USART2TX_Interrupt_init();
  /* we init the DMA1 */
  DMA1_USART2TX_init();

  /* we can print out */
  sprintf(str, "Wellcome on board\r\n");
  size = strlen(str);
  // while (USART2TX_DMA1_Interrupt == 0) {};
  // USART2TX_DMA1_Interrupt = 1;
  DMA1_USART2TX_write((unsigned int) str, (unsigned int) &USART2->DR, size);
  addition1(SystemCoreClock);

#if 0
  sprintf(str, "Ready\r\n");
  size = strlen(str);
  while (USART2TX_DMA1_Interrupt == 0) {};
  USART2TX_DMA1_Interrupt = 0;
  DMA1_USART2TX_setup(str, (unsigned int) &USART2->DR, size);
  addition();

  /* we print the System Bus Clock */
  sprintf(str, "System Bus Clock = %lu\r\n", SystemCoreClock);
  size = strlen(str);
  while (USART2TX_DMA1_Interrupt == 0) {};
  USART2TX_DMA1_Interrupt = 0;
  DMA1_USART2TX_setup(str, (unsigned int) &USART2->DR, size);
#endif

  /* We print the cpu clock */
  T = HAL_RCC_GetSysClockFreq();
  sprintf(str, "CPU Clock = %lu\r\n", T);
  size = strlen(str);

  // while (USART2TX_DMA1_Interrupt == 0) {};
  // USART2TX_DMA1_Interrupt = 0;
  DMA1_USART2TX_write(str, (unsigned int) &USART2->DR, size);
  addition1(SystemCoreClock);

  /* We print the bus clock of USART2 */
  T = HAL_RCC_GetPCLK1Freq();
  sprintf(str, "Serial bus Clock = %lu\r\n", T);
  size = strlen(str);
  // while (USART2TX_DMA1_Interrupt == 0) {};
  // USART2TX_DMA1_Interrupt = 0;
  DMA1_USART2TX_write(str, (unsigned int) &USART2->DR, size);
  addition1(SystemCoreClock);
  while (1)
  {
	  ledOff();
	  while (!(ADC1->SR & 0x2)){};
	  data = ADC1->DR;
	  volt = (float)data * multiplier;
	  /* Temperature (in °C) = {(VSENSE – V25) / Avg_Slope} + 25 */
	  /* V25 = 0.76V, slope = 2.5 mV/C */
	  temp = 25.0f + (volt-0.76f) * divider;
	  floatToInt(temp, &res);
	  sprintf(str, "Temperature of CPU  %ld.%ld Celsius\r\n", res.unit, res.decimal);
	  // size = strlen(str);
	  // while (USART2TX_DMA1_Interrupt == 0) {};
	  // USART2TX_DMA1_Interrupt = 0;
	  USART2_DMA1_printf(str);
//	  DMA1_USART2TX_write(str, (unsigned int) &USART2->DR, size);
	  ledOn();
	  fp_addition1((float)(T)); // to wait longer
  }

}

#endif
/**
  * @brief System Clock Configuration
  * @retval None
  */
static void __attribute__((noinline)) SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
