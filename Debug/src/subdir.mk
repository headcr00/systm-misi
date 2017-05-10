################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/array_functions.c \
../src/isol_math.c \
../src/main.c \
../src/syscalls.c \
../src/system_stm32f10x.c \
../src/task_adc.c \
../src/task_measuretimer.c \
../src/task_uart.c 

OBJS += \
./src/array_functions.o \
./src/isol_math.o \
./src/main.o \
./src/syscalls.o \
./src/system_stm32f10x.o \
./src/task_adc.o \
./src/task_measuretimer.o \
./src/task_uart.o 

C_DEPS += \
./src/array_functions.d \
./src/isol_math.d \
./src/main.d \
./src/syscalls.d \
./src/system_stm32f10x.d \
./src/task_adc.d \
./src/task_measuretimer.d \
./src/task_uart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32F1 -DNUCLEO_F103RB -DSTM32F103RBTx -DSTM32 -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -DARM_MATH_CM3 -I"D:/Work/isol-meter/rev1.0/rev1/inc" -I"D:/Work/isol-meter/rev1.0/rev1/CMSIS/core" -I"D:/Work/isol-meter/rev1.0/rev1/CMSIS/device" -I"D:/Work/isol-meter/rev1.0/rev1/StdPeriph_Driver/inc" -I"D:/Work/isol-meter/rev1.0/rev1/Utilities/STM32F1xx-Nucleo" -I"D:/Work/isol-meter/rev1.0/rev1/rtos/inc" -I"D:/Work/isol-meter/rev1.0/rev1/src" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


