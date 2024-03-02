################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Interface/attitude_pid_controller.c \
../Core/Src/Interface/commander.c \
../Core/Src/Interface/controller.c \
../Core/Src/Interface/controller_brescianini.c \
../Core/Src/Interface/controller_indi.c \
../Core/Src/Interface/controller_mellinger.c \
../Core/Src/Interface/controller_pid.c \
../Core/Src/Interface/filter.c \
../Core/Src/Interface/planner.c \
../Core/Src/Interface/position_controller_indi.c \
../Core/Src/Interface/position_controller_pid.c \
../Core/Src/Interface/pptraj.c \
../Core/Src/Interface/pptraj_compressed.c 

OBJS += \
./Core/Src/Interface/attitude_pid_controller.o \
./Core/Src/Interface/commander.o \
./Core/Src/Interface/controller.o \
./Core/Src/Interface/controller_brescianini.o \
./Core/Src/Interface/controller_indi.o \
./Core/Src/Interface/controller_mellinger.o \
./Core/Src/Interface/controller_pid.o \
./Core/Src/Interface/filter.o \
./Core/Src/Interface/planner.o \
./Core/Src/Interface/position_controller_indi.o \
./Core/Src/Interface/position_controller_pid.o \
./Core/Src/Interface/pptraj.o \
./Core/Src/Interface/pptraj_compressed.o 

C_DEPS += \
./Core/Src/Interface/attitude_pid_controller.d \
./Core/Src/Interface/commander.d \
./Core/Src/Interface/controller.d \
./Core/Src/Interface/controller_brescianini.d \
./Core/Src/Interface/controller_indi.d \
./Core/Src/Interface/controller_mellinger.d \
./Core/Src/Interface/controller_pid.d \
./Core/Src/Interface/filter.d \
./Core/Src/Interface/planner.d \
./Core/Src/Interface/position_controller_indi.d \
./Core/Src/Interface/position_controller_pid.d \
./Core/Src/Interface/pptraj.d \
./Core/Src/Interface/pptraj_compressed.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Interface/%.o Core/Src/Interface/%.su Core/Src/Interface/%.cyclo: ../Core/Src/Interface/%.c Core/Src/Interface/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DENABLE_UART_DMA -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/App" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Lib/STM32F4xx_StdPeriph_Driver/inc" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Utils" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Algo" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Comms" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Drivers" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Interface" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Inc/Platform" -I"/home/swift/workspace/project/STM32F405_FreeRTOS/Core/Config" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Interface

clean-Core-2f-Src-2f-Interface:
	-$(RM) ./Core/Src/Interface/attitude_pid_controller.cyclo ./Core/Src/Interface/attitude_pid_controller.d ./Core/Src/Interface/attitude_pid_controller.o ./Core/Src/Interface/attitude_pid_controller.su ./Core/Src/Interface/commander.cyclo ./Core/Src/Interface/commander.d ./Core/Src/Interface/commander.o ./Core/Src/Interface/commander.su ./Core/Src/Interface/controller.cyclo ./Core/Src/Interface/controller.d ./Core/Src/Interface/controller.o ./Core/Src/Interface/controller.su ./Core/Src/Interface/controller_brescianini.cyclo ./Core/Src/Interface/controller_brescianini.d ./Core/Src/Interface/controller_brescianini.o ./Core/Src/Interface/controller_brescianini.su ./Core/Src/Interface/controller_indi.cyclo ./Core/Src/Interface/controller_indi.d ./Core/Src/Interface/controller_indi.o ./Core/Src/Interface/controller_indi.su ./Core/Src/Interface/controller_mellinger.cyclo ./Core/Src/Interface/controller_mellinger.d ./Core/Src/Interface/controller_mellinger.o ./Core/Src/Interface/controller_mellinger.su ./Core/Src/Interface/controller_pid.cyclo ./Core/Src/Interface/controller_pid.d ./Core/Src/Interface/controller_pid.o ./Core/Src/Interface/controller_pid.su ./Core/Src/Interface/filter.cyclo ./Core/Src/Interface/filter.d ./Core/Src/Interface/filter.o ./Core/Src/Interface/filter.su ./Core/Src/Interface/planner.cyclo ./Core/Src/Interface/planner.d ./Core/Src/Interface/planner.o ./Core/Src/Interface/planner.su ./Core/Src/Interface/position_controller_indi.cyclo ./Core/Src/Interface/position_controller_indi.d ./Core/Src/Interface/position_controller_indi.o ./Core/Src/Interface/position_controller_indi.su ./Core/Src/Interface/position_controller_pid.cyclo ./Core/Src/Interface/position_controller_pid.d ./Core/Src/Interface/position_controller_pid.o ./Core/Src/Interface/position_controller_pid.su ./Core/Src/Interface/pptraj.cyclo ./Core/Src/Interface/pptraj.d ./Core/Src/Interface/pptraj.o ./Core/Src/Interface/pptraj.su ./Core/Src/Interface/pptraj_compressed.cyclo ./Core/Src/Interface/pptraj_compressed.d ./Core/Src/Interface/pptraj_compressed.o ./Core/Src/Interface/pptraj_compressed.su

.PHONY: clean-Core-2f-Src-2f-Interface

