################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/auxiliary.c \
../Core/Src/control.c \
../Core/Src/dijkstra.c \
../Core/Src/drive.c \
../Core/Src/eeprom.c \
../Core/Src/interrupt.c \
../Core/Src/main.c \
../Core/Src/mode1.c \
../Core/Src/mode2.c \
../Core/Src/mode3.c \
../Core/Src/mode4.c \
../Core/Src/mode5.c \
../Core/Src/mode6.c \
../Core/Src/mode7.c \
../Core/Src/path.c \
../Core/Src/run.c \
../Core/Src/search.c \
../Core/Src/sensor.c \
../Core/Src/shortest_run_params.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/test_mode.c \
../Core/Src/turn_time_calculator.c 

OBJS += \
./Core/Src/auxiliary.o \
./Core/Src/control.o \
./Core/Src/dijkstra.o \
./Core/Src/drive.o \
./Core/Src/eeprom.o \
./Core/Src/interrupt.o \
./Core/Src/main.o \
./Core/Src/mode1.o \
./Core/Src/mode2.o \
./Core/Src/mode3.o \
./Core/Src/mode4.o \
./Core/Src/mode5.o \
./Core/Src/mode6.o \
./Core/Src/mode7.o \
./Core/Src/path.o \
./Core/Src/run.o \
./Core/Src/search.o \
./Core/Src/sensor.o \
./Core/Src/shortest_run_params.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/test_mode.o \
./Core/Src/turn_time_calculator.o 

C_DEPS += \
./Core/Src/auxiliary.d \
./Core/Src/control.d \
./Core/Src/dijkstra.d \
./Core/Src/drive.d \
./Core/Src/eeprom.d \
./Core/Src/interrupt.d \
./Core/Src/main.d \
./Core/Src/mode1.d \
./Core/Src/mode2.d \
./Core/Src/mode3.d \
./Core/Src/mode4.d \
./Core/Src/mode5.d \
./Core/Src/mode6.d \
./Core/Src/mode7.d \
./Core/Src/path.d \
./Core/Src/run.d \
./Core/Src/search.d \
./Core/Src/sensor.d \
./Core/Src/shortest_run_params.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/test_mode.d \
./Core/Src/turn_time_calculator.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/auxiliary.cyclo ./Core/Src/auxiliary.d ./Core/Src/auxiliary.o ./Core/Src/auxiliary.su ./Core/Src/control.cyclo ./Core/Src/control.d ./Core/Src/control.o ./Core/Src/control.su ./Core/Src/dijkstra.cyclo ./Core/Src/dijkstra.d ./Core/Src/dijkstra.o ./Core/Src/dijkstra.su ./Core/Src/drive.cyclo ./Core/Src/drive.d ./Core/Src/drive.o ./Core/Src/drive.su ./Core/Src/eeprom.cyclo ./Core/Src/eeprom.d ./Core/Src/eeprom.o ./Core/Src/eeprom.su ./Core/Src/interrupt.cyclo ./Core/Src/interrupt.d ./Core/Src/interrupt.o ./Core/Src/interrupt.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mode1.cyclo ./Core/Src/mode1.d ./Core/Src/mode1.o ./Core/Src/mode1.su ./Core/Src/mode2.cyclo ./Core/Src/mode2.d ./Core/Src/mode2.o ./Core/Src/mode2.su ./Core/Src/mode3.cyclo ./Core/Src/mode3.d ./Core/Src/mode3.o ./Core/Src/mode3.su ./Core/Src/mode4.cyclo ./Core/Src/mode4.d ./Core/Src/mode4.o ./Core/Src/mode4.su ./Core/Src/mode5.cyclo ./Core/Src/mode5.d ./Core/Src/mode5.o ./Core/Src/mode5.su ./Core/Src/mode6.cyclo ./Core/Src/mode6.d ./Core/Src/mode6.o ./Core/Src/mode6.su ./Core/Src/mode7.cyclo ./Core/Src/mode7.d ./Core/Src/mode7.o ./Core/Src/mode7.su ./Core/Src/path.cyclo ./Core/Src/path.d ./Core/Src/path.o ./Core/Src/path.su ./Core/Src/run.cyclo ./Core/Src/run.d ./Core/Src/run.o ./Core/Src/run.su ./Core/Src/search.cyclo ./Core/Src/search.d ./Core/Src/search.o ./Core/Src/search.su ./Core/Src/sensor.cyclo ./Core/Src/sensor.d ./Core/Src/sensor.o ./Core/Src/sensor.su ./Core/Src/shortest_run_params.cyclo ./Core/Src/shortest_run_params.d ./Core/Src/shortest_run_params.o ./Core/Src/shortest_run_params.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/test_mode.cyclo ./Core/Src/test_mode.d ./Core/Src/test_mode.o ./Core/Src/test_mode.su ./Core/Src/turn_time_calculator.cyclo ./Core/Src/turn_time_calculator.d ./Core/Src/turn_time_calculator.o ./Core/Src/turn_time_calculator.su

.PHONY: clean-Core-2f-Src

