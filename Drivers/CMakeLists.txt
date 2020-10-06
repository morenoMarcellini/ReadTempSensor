set (HAL_SOURCES
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c 
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
	STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
	)
	
add_library(hal STATIC ${HAL_SOURCES})

target_include_directories(hal
	SYSTEM
	PUBLIC
	${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
	${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
	${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
	${PROJECT_CONFIG_DIR}
	)
