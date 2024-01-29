################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/STM32F4xx_StdPeriph_Driver/src/misc.c 

OBJS += \
./Core/Lib/STM32F4xx_StdPeriph_Driver/src/misc.o 

C_DEPS += \
./Core/Lib/STM32F4xx_StdPeriph_Driver/src/misc.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/STM32F4xx_StdPeriph_Driver/src/%.o Core/Lib/STM32F4xx_StdPeriph_Driver/src/%.su Core/Lib/STM32F4xx_StdPeriph_Driver/src/%.cyclo: ../Core/Lib/STM32F4xx_StdPeriph_Driver/src/%.c Core/Lib/STM32F4xx_StdPeriph_Driver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCONFIG_DEBUG_PRINT_ON_UART=1 -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Lib/STM32F4xx_StdPeriph_Driver/inc" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Utils" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Algo" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Comms" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Drivers" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Interface" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Platform" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Config" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-STM32F4xx_StdPeriph_Driver-2f-src

clean-Core-2f-Lib-2f-STM32F4xx_StdPeriph_Driver-2f-src:
	-$(RM) ./Core/Lib/STM32F4xx_StdPeriph_Driver/src/misc.cyclo ./Core/Lib/STM32F4xx_StdPeriph_Driver/src/misc.d ./Core/Lib/STM32F4xx_StdPeriph_Driver/src/misc.o ./Core/Lib/STM32F4xx_StdPeriph_Driver/src/misc.su

.PHONY: clean-Core-2f-Lib-2f-STM32F4xx_StdPeriph_Driver-2f-src

