################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Utils/cfassert.c \
../Core/Src/Utils/console.c \
../Core/Src/Utils/debug.c \
../Core/Src/Utils/debug_printf.c \
../Core/Src/Utils/eprintf.c \
../Core/Src/Utils/queuemonitor.c \
../Core/Src/Utils/sysload.c \
../Core/Src/Utils/usec_time.c \
../Core/Src/Utils/version.c 

OBJS += \
./Core/Src/Utils/cfassert.o \
./Core/Src/Utils/console.o \
./Core/Src/Utils/debug.o \
./Core/Src/Utils/debug_printf.o \
./Core/Src/Utils/eprintf.o \
./Core/Src/Utils/queuemonitor.o \
./Core/Src/Utils/sysload.o \
./Core/Src/Utils/usec_time.o \
./Core/Src/Utils/version.o 

C_DEPS += \
./Core/Src/Utils/cfassert.d \
./Core/Src/Utils/console.d \
./Core/Src/Utils/debug.d \
./Core/Src/Utils/debug_printf.d \
./Core/Src/Utils/eprintf.d \
./Core/Src/Utils/queuemonitor.d \
./Core/Src/Utils/sysload.d \
./Core/Src/Utils/usec_time.d \
./Core/Src/Utils/version.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Utils/%.o Core/Src/Utils/%.su Core/Src/Utils/%.cyclo: ../Core/Src/Utils/%.c Core/Src/Utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DENABLE_UART_DMA -DCONFIG_DEBUG_PRINT_ON_UART=1 -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Lib/STM32F4xx_StdPeriph_Driver/inc" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Utils" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Algo" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Comms" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Drivers" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Interface" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Platform" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Config" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Utils

clean-Core-2f-Src-2f-Utils:
	-$(RM) ./Core/Src/Utils/cfassert.cyclo ./Core/Src/Utils/cfassert.d ./Core/Src/Utils/cfassert.o ./Core/Src/Utils/cfassert.su ./Core/Src/Utils/console.cyclo ./Core/Src/Utils/console.d ./Core/Src/Utils/console.o ./Core/Src/Utils/console.su ./Core/Src/Utils/debug.cyclo ./Core/Src/Utils/debug.d ./Core/Src/Utils/debug.o ./Core/Src/Utils/debug.su ./Core/Src/Utils/debug_printf.cyclo ./Core/Src/Utils/debug_printf.d ./Core/Src/Utils/debug_printf.o ./Core/Src/Utils/debug_printf.su ./Core/Src/Utils/eprintf.cyclo ./Core/Src/Utils/eprintf.d ./Core/Src/Utils/eprintf.o ./Core/Src/Utils/eprintf.su ./Core/Src/Utils/queuemonitor.cyclo ./Core/Src/Utils/queuemonitor.d ./Core/Src/Utils/queuemonitor.o ./Core/Src/Utils/queuemonitor.su ./Core/Src/Utils/sysload.cyclo ./Core/Src/Utils/sysload.d ./Core/Src/Utils/sysload.o ./Core/Src/Utils/sysload.su ./Core/Src/Utils/usec_time.cyclo ./Core/Src/Utils/usec_time.d ./Core/Src/Utils/usec_time.o ./Core/Src/Utils/usec_time.su ./Core/Src/Utils/version.cyclo ./Core/Src/Utils/version.d ./Core/Src/Utils/version.o ./Core/Src/Utils/version.su

.PHONY: clean-Core-2f-Src-2f-Utils

