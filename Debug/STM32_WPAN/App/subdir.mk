################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32_WPAN/App/app_ble.c \
../STM32_WPAN/App/dis_app.c \
../STM32_WPAN/App/dt_client_app.c \
../STM32_WPAN/App/dt_server_app.c \
../STM32_WPAN/App/dts.c \
../STM32_WPAN/App/hrs_app.c 

C_DEPS += \
./STM32_WPAN/App/app_ble.d \
./STM32_WPAN/App/dis_app.d \
./STM32_WPAN/App/dt_client_app.d \
./STM32_WPAN/App/dt_server_app.d \
./STM32_WPAN/App/dts.d \
./STM32_WPAN/App/hrs_app.d 

OBJS += \
./STM32_WPAN/App/app_ble.o \
./STM32_WPAN/App/dis_app.o \
./STM32_WPAN/App/dt_client_app.o \
./STM32_WPAN/App/dt_server_app.o \
./STM32_WPAN/App/dts.o \
./STM32_WPAN/App/hrs_app.o 


# Each subdirectory must supply rules for building sources it contributes
STM32_WPAN/App/%.o: ../STM32_WPAN/App/%.c STM32_WPAN/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB5Mxx -c -I../Core/Inc -I../STM32_WPAN/App -I../USB_Device/App -I../USB_Device/Target -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Middlewares/ST/STM32_WPAN/ble -I../Drivers/CMSIS/Include -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_Calipile" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_AS7341" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_TSL2772" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-STM32_WPAN-2f-App

clean-STM32_WPAN-2f-App:
	-$(RM) ./STM32_WPAN/App/app_ble.d ./STM32_WPAN/App/app_ble.o ./STM32_WPAN/App/dis_app.d ./STM32_WPAN/App/dis_app.o ./STM32_WPAN/App/dt_client_app.d ./STM32_WPAN/App/dt_client_app.o ./STM32_WPAN/App/dt_server_app.d ./STM32_WPAN/App/dt_server_app.o ./STM32_WPAN/App/dts.d ./STM32_WPAN/App/dts.o ./STM32_WPAN/App/hrs_app.d ./STM32_WPAN/App/hrs_app.o

.PHONY: clean-STM32_WPAN-2f-App

