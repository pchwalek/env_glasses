################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Middlewares/Sensirion_Core/src/SensirionCrc.cpp \
../Middlewares/Sensirion_Core/src/SensirionErrors.cpp \
../Middlewares/Sensirion_Core/src/SensirionI2CCommunication.cpp \
../Middlewares/Sensirion_Core/src/SensirionI2CTxFrame.cpp \
../Middlewares/Sensirion_Core/src/SensirionRxFrame.cpp 

OBJS += \
./Middlewares/Sensirion_Core/src/SensirionCrc.o \
./Middlewares/Sensirion_Core/src/SensirionErrors.o \
./Middlewares/Sensirion_Core/src/SensirionI2CCommunication.o \
./Middlewares/Sensirion_Core/src/SensirionI2CTxFrame.o \
./Middlewares/Sensirion_Core/src/SensirionRxFrame.o 

CPP_DEPS += \
./Middlewares/Sensirion_Core/src/SensirionCrc.d \
./Middlewares/Sensirion_Core/src/SensirionErrors.d \
./Middlewares/Sensirion_Core/src/SensirionI2CCommunication.d \
./Middlewares/Sensirion_Core/src/SensirionI2CTxFrame.d \
./Middlewares/Sensirion_Core/src/SensirionRxFrame.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Sensirion_Core/src/%.o: ../Middlewares/Sensirion_Core/src/%.cpp Middlewares/Sensirion_Core/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB5Mxx -c -I../Core/Inc -I../STM32_WPAN/App -I../USB_Device/App -I../USB_Device/Target -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Middlewares/ST/STM32_WPAN/ble -I../Drivers/CMSIS/Include -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_Calipile" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_AS7341" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_TSL2772" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_BME680" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_AdafruitSensor" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_SHT4X" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/Sensirion_Core/src" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_ICM20948" -I"/Users/chwalek/dev/captivatesEnv/Drivers/CMSIS/DSP/Include" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/bsec_2_2_0_0/algo/normal_version/inc" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_SGP41" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/gas-index-algorithm/sensirion_gas_index_algorithm" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/bsec_2_2_0_0" -I../Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc -Og -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Sensirion_Core-2f-src

clean-Middlewares-2f-Sensirion_Core-2f-src:
	-$(RM) ./Middlewares/Sensirion_Core/src/SensirionCrc.d ./Middlewares/Sensirion_Core/src/SensirionCrc.o ./Middlewares/Sensirion_Core/src/SensirionErrors.d ./Middlewares/Sensirion_Core/src/SensirionErrors.o ./Middlewares/Sensirion_Core/src/SensirionI2CCommunication.d ./Middlewares/Sensirion_Core/src/SensirionI2CCommunication.o ./Middlewares/Sensirion_Core/src/SensirionI2CTxFrame.d ./Middlewares/Sensirion_Core/src/SensirionI2CTxFrame.o ./Middlewares/Sensirion_Core/src/SensirionRxFrame.d ./Middlewares/Sensirion_Core/src/SensirionRxFrame.o

.PHONY: clean-Middlewares-2f-Sensirion_Core-2f-src

