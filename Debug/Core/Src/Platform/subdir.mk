################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Platform/platform.c \
../Core/Src/Platform/platform_cf2.c \
../Core/Src/Platform/platform_stm32f4.c 

OBJS += \
./Core/Src/Platform/platform.o \
./Core/Src/Platform/platform_cf2.o \
./Core/Src/Platform/platform_stm32f4.o 

C_DEPS += \
./Core/Src/Platform/platform.d \
./Core/Src/Platform/platform_cf2.d \
./Core/Src/Platform/platform_stm32f4.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Platform/%.o Core/Src/Platform/%.su Core/Src/Platform/%.cyclo: ../Core/Src/Platform/%.c Core/Src/Platform/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DENABLE_UART_DMA -DCONFIG_DEBUG_PRINT_ON_UART=1 -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Lib/STM32F4xx_StdPeriph_Driver/inc" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Utils" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Algo" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Comms" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Drivers" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Interface" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Platform" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Config" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Platform

clean-Core-2f-Src-2f-Platform:
	-$(RM) ./Core/Src/Platform/platform.cyclo ./Core/Src/Platform/platform.d ./Core/Src/Platform/platform.o ./Core/Src/Platform/platform.su ./Core/Src/Platform/platform_cf2.cyclo ./Core/Src/Platform/platform_cf2.d ./Core/Src/Platform/platform_cf2.o ./Core/Src/Platform/platform_cf2.su ./Core/Src/Platform/platform_stm32f4.cyclo ./Core/Src/Platform/platform_stm32f4.d ./Core/Src/Platform/platform_stm32f4.o ./Core/Src/Platform/platform_stm32f4.su

.PHONY: clean-Core-2f-Src-2f-Platform

