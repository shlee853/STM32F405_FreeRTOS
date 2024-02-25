################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Drivers/ICM20602.c \
../Core/Src/Drivers/bootloader.c \
../Core/Src/Drivers/led.c \
../Core/Src/Drivers/system.c \
../Core/Src/Drivers/usb.c \
../Core/Src/Drivers/usblink.c \
../Core/Src/Drivers/vcp_esc_passthrough.c 

OBJS += \
./Core/Src/Drivers/ICM20602.o \
./Core/Src/Drivers/bootloader.o \
./Core/Src/Drivers/led.o \
./Core/Src/Drivers/system.o \
./Core/Src/Drivers/usb.o \
./Core/Src/Drivers/usblink.o \
./Core/Src/Drivers/vcp_esc_passthrough.o 

C_DEPS += \
./Core/Src/Drivers/ICM20602.d \
./Core/Src/Drivers/bootloader.d \
./Core/Src/Drivers/led.d \
./Core/Src/Drivers/system.d \
./Core/Src/Drivers/usb.d \
./Core/Src/Drivers/usblink.d \
./Core/Src/Drivers/vcp_esc_passthrough.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Drivers/%.o Core/Src/Drivers/%.su Core/Src/Drivers/%.cyclo: ../Core/Src/Drivers/%.c Core/Src/Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DENABLE_UART_DMA -DCONFIG_DEBUG_PRINT_ON_UART=1 -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Lib/STM32F4xx_StdPeriph_Driver/inc" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Utils" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Algo" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Comms" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Drivers" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Interface" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Platform" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Config" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Drivers

clean-Core-2f-Src-2f-Drivers:
	-$(RM) ./Core/Src/Drivers/ICM20602.cyclo ./Core/Src/Drivers/ICM20602.d ./Core/Src/Drivers/ICM20602.o ./Core/Src/Drivers/ICM20602.su ./Core/Src/Drivers/bootloader.cyclo ./Core/Src/Drivers/bootloader.d ./Core/Src/Drivers/bootloader.o ./Core/Src/Drivers/bootloader.su ./Core/Src/Drivers/led.cyclo ./Core/Src/Drivers/led.d ./Core/Src/Drivers/led.o ./Core/Src/Drivers/led.su ./Core/Src/Drivers/system.cyclo ./Core/Src/Drivers/system.d ./Core/Src/Drivers/system.o ./Core/Src/Drivers/system.su ./Core/Src/Drivers/usb.cyclo ./Core/Src/Drivers/usb.d ./Core/Src/Drivers/usb.o ./Core/Src/Drivers/usb.su ./Core/Src/Drivers/usblink.cyclo ./Core/Src/Drivers/usblink.d ./Core/Src/Drivers/usblink.o ./Core/Src/Drivers/usblink.su ./Core/Src/Drivers/vcp_esc_passthrough.cyclo ./Core/Src/Drivers/vcp_esc_passthrough.d ./Core/Src/Drivers/vcp_esc_passthrough.o ./Core/Src/Drivers/vcp_esc_passthrough.su

.PHONY: clean-Core-2f-Src-2f-Drivers

