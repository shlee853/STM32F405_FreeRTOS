################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Algo/estimator.c \
../Core/Src/Algo/peer_localization.c 

OBJS += \
./Core/Src/Algo/estimator.o \
./Core/Src/Algo/peer_localization.o 

C_DEPS += \
./Core/Src/Algo/estimator.d \
./Core/Src/Algo/peer_localization.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Algo/%.o Core/Src/Algo/%.su Core/Src/Algo/%.cyclo: ../Core/Src/Algo/%.c Core/Src/Algo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DENABLE_UART_DMA -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/App" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Lib/STM32F4xx_StdPeriph_Driver/inc" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Utils" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Algo" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Comms" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Drivers" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Interface" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Platform" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Config" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Algo

clean-Core-2f-Src-2f-Algo:
	-$(RM) ./Core/Src/Algo/estimator.cyclo ./Core/Src/Algo/estimator.d ./Core/Src/Algo/estimator.o ./Core/Src/Algo/estimator.su ./Core/Src/Algo/peer_localization.cyclo ./Core/Src/Algo/peer_localization.d ./Core/Src/Algo/peer_localization.o ./Core/Src/Algo/peer_localization.su

.PHONY: clean-Core-2f-Src-2f-Algo

