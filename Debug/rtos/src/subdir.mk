################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../rtos/src/croutine.c \
../rtos/src/event_groups.c \
../rtos/src/heap_2.c \
../rtos/src/list.c \
../rtos/src/port.c \
../rtos/src/queue.c \
../rtos/src/tasks.c \
../rtos/src/timers.c 

OBJS += \
./rtos/src/croutine.o \
./rtos/src/event_groups.o \
./rtos/src/heap_2.o \
./rtos/src/list.o \
./rtos/src/port.o \
./rtos/src/queue.o \
./rtos/src/tasks.o \
./rtos/src/timers.o 

C_DEPS += \
./rtos/src/croutine.d \
./rtos/src/event_groups.d \
./rtos/src/heap_2.d \
./rtos/src/list.d \
./rtos/src/port.d \
./rtos/src/queue.d \
./rtos/src/tasks.d \
./rtos/src/timers.d 


# Each subdirectory must supply rules for building sources it contributes
rtos/src/%.o: ../rtos/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32F1 -DNUCLEO_F103RB -DSTM32F103RBTx -DSTM32 -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -I"D:/Work/isol-meter/rev1.0/rev1/inc" -I"D:/Work/isol-meter/rev1.0/rev1/CMSIS/core" -I"D:/Work/isol-meter/rev1.0/rev1/CMSIS/device" -I"D:/Work/isol-meter/rev1.0/rev1/StdPeriph_Driver/inc" -I"D:/Work/isol-meter/rev1.0/rev1/Utilities/STM32F1xx-Nucleo" -I"D:/Work/isol-meter/rev1.0/rev1/rtos/inc" -I"D:/Work/isol-meter/rev1.0/rev1/src" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


