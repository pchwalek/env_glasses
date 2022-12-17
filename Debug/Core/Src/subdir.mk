################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/app_debug.c \
../Core/Src/app_entry.c \
../Core/Src/blink.c \
../Core/Src/circular_buffer.c \
../Core/Src/dma.c \
../Core/Src/freertos_port.c \
../Core/Src/gpio.c \
../Core/Src/hw_timerserver.c \
../Core/Src/i2c.c \
../Core/Src/ipcc.c \
../Core/Src/led.c \
../Core/Src/lp5523.c \
../Core/Src/rf.c \
../Core/Src/rtc.c \
../Core/Src/sai.c \
../Core/Src/spi.c \
../Core/Src/stm32_lpm_if.c \
../Core/Src/stm32wbxx_hal_msp.c \
../Core/Src/stm32wbxx_hal_timebase_tim.c \
../Core/Src/stm32wbxx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32wbxx.c \
../Core/Src/tim.c 

CPP_SRCS += \
../Core/Src/app_freertos.cpp \
../Core/Src/bme.cpp \
../Core/Src/fram.cpp \
../Core/Src/imu.cpp \
../Core/Src/lux.cpp \
../Core/Src/main.cpp \
../Core/Src/mic.cpp \
../Core/Src/packet.cpp \
../Core/Src/sgp.cpp \
../Core/Src/sht.cpp \
../Core/Src/spectrometer.cpp \
../Core/Src/thermopile.cpp 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/app_debug.d \
./Core/Src/app_entry.d \
./Core/Src/blink.d \
./Core/Src/circular_buffer.d \
./Core/Src/dma.d \
./Core/Src/freertos_port.d \
./Core/Src/gpio.d \
./Core/Src/hw_timerserver.d \
./Core/Src/i2c.d \
./Core/Src/ipcc.d \
./Core/Src/led.d \
./Core/Src/lp5523.d \
./Core/Src/rf.d \
./Core/Src/rtc.d \
./Core/Src/sai.d \
./Core/Src/spi.d \
./Core/Src/stm32_lpm_if.d \
./Core/Src/stm32wbxx_hal_msp.d \
./Core/Src/stm32wbxx_hal_timebase_tim.d \
./Core/Src/stm32wbxx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32wbxx.d \
./Core/Src/tim.d 

OBJS += \
./Core/Src/adc.o \
./Core/Src/app_debug.o \
./Core/Src/app_entry.o \
./Core/Src/app_freertos.o \
./Core/Src/blink.o \
./Core/Src/bme.o \
./Core/Src/circular_buffer.o \
./Core/Src/dma.o \
./Core/Src/fram.o \
./Core/Src/freertos_port.o \
./Core/Src/gpio.o \
./Core/Src/hw_timerserver.o \
./Core/Src/i2c.o \
./Core/Src/imu.o \
./Core/Src/ipcc.o \
./Core/Src/led.o \
./Core/Src/lp5523.o \
./Core/Src/lux.o \
./Core/Src/main.o \
./Core/Src/mic.o \
./Core/Src/packet.o \
./Core/Src/rf.o \
./Core/Src/rtc.o \
./Core/Src/sai.o \
./Core/Src/sgp.o \
./Core/Src/sht.o \
./Core/Src/spectrometer.o \
./Core/Src/spi.o \
./Core/Src/stm32_lpm_if.o \
./Core/Src/stm32wbxx_hal_msp.o \
./Core/Src/stm32wbxx_hal_timebase_tim.o \
./Core/Src/stm32wbxx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32wbxx.o \
./Core/Src/thermopile.o \
./Core/Src/tim.o 

CPP_DEPS += \
./Core/Src/app_freertos.d \
./Core/Src/bme.d \
./Core/Src/fram.d \
./Core/Src/imu.d \
./Core/Src/lux.d \
./Core/Src/main.d \
./Core/Src/mic.d \
./Core/Src/packet.d \
./Core/Src/sgp.d \
./Core/Src/sht.d \
./Core/Src/spectrometer.d \
./Core/Src/thermopile.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB5Mxx -c -I../Core/Inc -I../STM32_WPAN/App -I../USB_Device/App -I../USB_Device/Target -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Middlewares/ST/STM32_WPAN/ble -I../Drivers/CMSIS/Include -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_Calipile" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_AS7341" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_TSL2772" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_BME680" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_AdafruitSensor" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_SHT4X" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/Sensirion_Core/src" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_ICM20948" -I"/Users/chwalek/dev/captivatesEnv/Drivers/CMSIS/DSP/Include" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/bsec_2_2_0_0/algo/normal_version/inc" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_SGP41" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/gas-index-algorithm/sensirion_gas_index_algorithm" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/bsec_2_2_0_0" -I../Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB5Mxx -c -I../Core/Inc -I../STM32_WPAN/App -I../USB_Device/App -I../USB_Device/Target -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Middlewares/ST/STM32_WPAN/ble -I../Drivers/CMSIS/Include -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_Calipile" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_AS7341" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_TSL2772" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_BME680" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_AdafruitSensor" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_SHT4X" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/Sensirion_Core/src" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_ICM20948" -I"/Users/chwalek/dev/captivatesEnv/Drivers/CMSIS/DSP/Include" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/bsec_2_2_0_0/algo/normal_version/inc" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/STM32_SGP41" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/gas-index-algorithm/sensirion_gas_index_algorithm" -I"/Users/chwalek/dev/captivatesEnv/Middlewares/bsec_2_2_0_0" -I../Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc -Og -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/app_debug.d ./Core/Src/app_debug.o ./Core/Src/app_entry.d ./Core/Src/app_entry.o ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/blink.d ./Core/Src/blink.o ./Core/Src/bme.d ./Core/Src/bme.o ./Core/Src/circular_buffer.d ./Core/Src/circular_buffer.o ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/fram.d ./Core/Src/fram.o ./Core/Src/freertos_port.d ./Core/Src/freertos_port.o ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/hw_timerserver.d ./Core/Src/hw_timerserver.o ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/ipcc.d ./Core/Src/ipcc.o ./Core/Src/led.d ./Core/Src/led.o ./Core/Src/lp5523.d ./Core/Src/lp5523.o ./Core/Src/lux.d ./Core/Src/lux.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/mic.d ./Core/Src/mic.o ./Core/Src/packet.d ./Core/Src/packet.o ./Core/Src/rf.d ./Core/Src/rf.o ./Core/Src/rtc.d ./Core/Src/rtc.o ./Core/Src/sai.d ./Core/Src/sai.o ./Core/Src/sgp.d ./Core/Src/sgp.o ./Core/Src/sht.d ./Core/Src/sht.o ./Core/Src/spectrometer.d ./Core/Src/spectrometer.o ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/stm32_lpm_if.d ./Core/Src/stm32_lpm_if.o ./Core/Src/stm32wbxx_hal_msp.d ./Core/Src/stm32wbxx_hal_msp.o ./Core/Src/stm32wbxx_hal_timebase_tim.d ./Core/Src/stm32wbxx_hal_timebase_tim.o ./Core/Src/stm32wbxx_it.d ./Core/Src/stm32wbxx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32wbxx.d ./Core/Src/system_stm32wbxx.o ./Core/Src/thermopile.d ./Core/Src/thermopile.o ./Core/Src/tim.d ./Core/Src/tim.o

.PHONY: clean-Core-2f-Src

