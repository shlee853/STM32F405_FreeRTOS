################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Comms/comm.c \
../Core/Src/Comms/crtp.c \
../Core/Src/Comms/crtpservice.c \
../Core/Src/Comms/platformservice.c \
../Core/Src/Comms/radiolink.c 

OBJS += \
./Core/Src/Comms/comm.o \
./Core/Src/Comms/crtp.o \
./Core/Src/Comms/crtpservice.o \
./Core/Src/Comms/platformservice.o \
./Core/Src/Comms/radiolink.o 

C_DEPS += \
./Core/Src/Comms/comm.d \
./Core/Src/Comms/crtp.d \
./Core/Src/Comms/crtpservice.d \
./Core/Src/Comms/platformservice.d \
./Core/Src/Comms/radiolink.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Comms/%.o Core/Src/Comms/%.su Core/Src/Comms/%.cyclo: ../Core/Src/Comms/%.c Core/Src/Comms/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DENABLE_UART_DMA -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/App" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Lib/STM32F4xx_StdPeriph_Driver/inc" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Utils" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Algo" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Comms" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Drivers" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Interface" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Platform" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Config" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Comms

clean-Core-2f-Src-2f-Comms:
	-$(RM) ./Core/Src/Comms/comm.cyclo ./Core/Src/Comms/comm.d ./Core/Src/Comms/comm.o ./Core/Src/Comms/comm.su ./Core/Src/Comms/crtp.cyclo ./Core/Src/Comms/crtp.d ./Core/Src/Comms/crtp.o ./Core/Src/Comms/crtp.su ./Core/Src/Comms/crtpservice.cyclo ./Core/Src/Comms/crtpservice.d ./Core/Src/Comms/crtpservice.o ./Core/Src/Comms/crtpservice.su ./Core/Src/Comms/platformservice.cyclo ./Core/Src/Comms/platformservice.d ./Core/Src/Comms/platformservice.o ./Core/Src/Comms/platformservice.su ./Core/Src/Comms/radiolink.cyclo ./Core/Src/Comms/radiolink.d ./Core/Src/Comms/radiolink.o ./Core/Src/Comms/radiolink.su

.PHONY: clean-Core-2f-Src-2f-Comms

