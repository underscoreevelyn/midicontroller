################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Device_Library/Class/midi/src/usbd_midi_core.c 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/midi/src/usbd_midi_core.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/midi/src/usbd_midi_core.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/Class/midi/src/%.o Middlewares/ST/STM32_USB_Device_Library/Class/midi/src/%.su: ../Middlewares/ST/STM32_USB_Device_Library/Class/midi/src/%.c Middlewares/ST/STM32_USB_Device_Library/Class/midi/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-midi-2f-src

clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-midi-2f-src:
	-$(RM) ./Middlewares/ST/STM32_USB_Device_Library/Class/midi/src/usbd_midi_core.d ./Middlewares/ST/STM32_USB_Device_Library/Class/midi/src/usbd_midi_core.o ./Middlewares/ST/STM32_USB_Device_Library/Class/midi/src/usbd_midi_core.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-midi-2f-src

