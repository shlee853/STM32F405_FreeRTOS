################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.o Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.su Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.cyclo: ../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.c Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCRAZYFLIE_FW -D__FPU_PRESENT -DARM_MATH_CM4 -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/App" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Lib/STM32F4xx_StdPeriph_Driver/inc" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Utils" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Algo" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Comms" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Drivers" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Interface" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Platform" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Config" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CDC-2f-Src

clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CDC-2f-Src:
	-$(RM) ./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.cyclo ./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.d ./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.o ./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CDC-2f-Src

