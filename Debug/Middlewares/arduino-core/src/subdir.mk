################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Middlewares/arduino-core/src/SensirionCrc.cpp \
../Middlewares/arduino-core/src/SensirionErrors.cpp \
../Middlewares/arduino-core/src/SensirionI2CCommunication.cpp \
../Middlewares/arduino-core/src/SensirionI2CTxFrame.cpp \
../Middlewares/arduino-core/src/SensirionRxFrame.cpp \
../Middlewares/arduino-core/src/SensirionShdlcCommunication.cpp \
../Middlewares/arduino-core/src/SensirionShdlcTxFrame.cpp 

OBJS += \
./Middlewares/arduino-core/src/SensirionCrc.o \
./Middlewares/arduino-core/src/SensirionErrors.o \
./Middlewares/arduino-core/src/SensirionI2CCommunication.o \
./Middlewares/arduino-core/src/SensirionI2CTxFrame.o \
./Middlewares/arduino-core/src/SensirionRxFrame.o \
./Middlewares/arduino-core/src/SensirionShdlcCommunication.o \
./Middlewares/arduino-core/src/SensirionShdlcTxFrame.o 

CPP_DEPS += \
./Middlewares/arduino-core/src/SensirionCrc.d \
./Middlewares/arduino-core/src/SensirionErrors.d \
./Middlewares/arduino-core/src/SensirionI2CCommunication.d \
./Middlewares/arduino-core/src/SensirionI2CTxFrame.d \
./Middlewares/arduino-core/src/SensirionRxFrame.d \
./Middlewares/arduino-core/src/SensirionShdlcCommunication.d \
./Middlewares/arduino-core/src/SensirionShdlcTxFrame.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/arduino-core/src/%.o: ../Middlewares/arduino-core/src/%.cpp Middlewares/arduino-core/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB5Mxx -c -I../Core/Inc -I../STM32_WPAN/App -I../USB_Device/App -I../USB_Device/Target -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Middlewares/ST/STM32_WPAN/ble -I../Drivers/CMSIS/Include -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_Calipile" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_AS7341" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_TSL2772" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_BME680" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_AdafruitSensor" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_SHT4X" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-arduino-2d-core-2f-src

clean-Middlewares-2f-arduino-2d-core-2f-src:
	-$(RM) ./Middlewares/arduino-core/src/SensirionCrc.d ./Middlewares/arduino-core/src/SensirionCrc.o ./Middlewares/arduino-core/src/SensirionErrors.d ./Middlewares/arduino-core/src/SensirionErrors.o ./Middlewares/arduino-core/src/SensirionI2CCommunication.d ./Middlewares/arduino-core/src/SensirionI2CCommunication.o ./Middlewares/arduino-core/src/SensirionI2CTxFrame.d ./Middlewares/arduino-core/src/SensirionI2CTxFrame.o ./Middlewares/arduino-core/src/SensirionRxFrame.d ./Middlewares/arduino-core/src/SensirionRxFrame.o ./Middlewares/arduino-core/src/SensirionShdlcCommunication.d ./Middlewares/arduino-core/src/SensirionShdlcCommunication.o ./Middlewares/arduino-core/src/SensirionShdlcTxFrame.d ./Middlewares/arduino-core/src/SensirionShdlcTxFrame.o

.PHONY: clean-Middlewares-2f-arduino-2d-core-2f-src

