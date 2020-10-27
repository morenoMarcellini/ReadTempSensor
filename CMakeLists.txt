cmake_minimum_required(VERSION 3.7)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchain.cmake)

set(PROJECT_NAME firmware)

project(${PROJECT_NAME})

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
       "Choose the type of build, options are: Debug Release."
       FORCE)
endif()

set(LDSCRIPT ${CMAKE_SOURCE_DIR}/STM32F446RETX_FLASH.ld)

set(PROJECT_CONFIG_DIR ${CMAKE_SOURCE_DIR}/Core/Inc)

enable_language(C CXX ASM)

set(ARM_FLAGS -DCORE_CM4 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -flto)

add_compile_options(${ARM_FLAGS})
add_compile_options(-Wall -Wextra)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-fdata-sections -ffunction-sections)
add_compile_options(-O0)
add_compile_options(-fsingle-precision-constant)
add_compile_options(-pipe)
add_compile_options(-flto)

set(LDSPECS nano.specs)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")

string(REPLACE ";" " " ARM_LINK_FLAGS "${ARM_FLAGS}")

set(LINK_FLAGS "${ARM_LINK_FLAGS} -v -specs=nosys.specs -Wl,--gc-sections -Wl,--undefined=uxTopUsedPriority,--undefined=freeRTOSMemoryScheme -Wl,-Map=${PROJECT_NAME}.map,--cref")

set(CMAKE_EXE_LINKER_FLAGS "${LINK_FLAGS} -specs=${LDSPECS} -T${LDSCRIPT}")

add_subdirectory(${CMAKE_SOURCE_DIR}/Drivers)

set(FLASH_ADDRESS 0x8000000)

set(STARTUP_SOURCE ${CMAKE_SOURCE_DIR}/Core/Startup/startup_stm32f446retx.s)

set(COMMON_SOURCE 
	${CMAKE_SOURCE_DIR}/Core/Src/main.c  
	${CMAKE_SOURCE_DIR}/Core/Src/habanero.c
        ${CMAKE_SOURCE_DIR}/Core/Src/DMA-Collector.c	
	${CMAKE_SOURCE_DIR}/Core/Src/stm32f4xx_hal_msp.c  
	${CMAKE_SOURCE_DIR}/Core/Src/stm32f4xx_it.c  
	${CMAKE_SOURCE_DIR}/Core/Src/sysmem.c  
	${CMAKE_SOURCE_DIR}/Core/Src/system_stm32f4xx.c
	)
		
add_executable(${PROJECT_NAME} ${COMMON_SOURCE} ${STARTUP_SOURCE})
		
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/Core/Inc)	

set_target_properties(${PROJECT_NAME} PROPERTIES LINK_DEPENDS ${LDSCRIPT})

target_link_libraries(${PROJECT_NAME} hal)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${ARM_SIZE_UTIL} ${PROJECT_NAME}.elf
    COMMAND ${ARM_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    )
    
add_custom_target(flash_ocd)
add_custom_command(
    TARGET flash_ocd
    COMMAND ${OPENOCD} -f board/st_nucleo_f4.cfg -c "program ${PROJECT_NAME}.bin reset exit ${FLASH_ADDRESS}"
) 
 
