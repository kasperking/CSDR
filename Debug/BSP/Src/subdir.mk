################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/Src/bpf_lpf.c \
../BSP/Src/cal.c \
../BSP/Src/csdr_app.c \
../BSP/Src/diag.c \
../BSP/Src/encoder.c \
../BSP/Src/fsdr_analog.c \
../BSP/Src/menu.c \
../BSP/Src/pe4302.c \
../BSP/Src/sdr_dsp.c \
../BSP/Src/si5351.c \
../BSP/Src/st7789.c \
../BSP/Src/usb_audio.c \
../BSP/Src/usb_cat.c \
../BSP/Src/w25q128.c \
../BSP/Src/wm8731.c 

OBJS += \
./BSP/Src/bpf_lpf.o \
./BSP/Src/cal.o \
./BSP/Src/csdr_app.o \
./BSP/Src/diag.o \
./BSP/Src/encoder.o \
./BSP/Src/fsdr_analog.o \
./BSP/Src/menu.o \
./BSP/Src/pe4302.o \
./BSP/Src/sdr_dsp.o \
./BSP/Src/si5351.o \
./BSP/Src/st7789.o \
./BSP/Src/usb_audio.o \
./BSP/Src/usb_cat.o \
./BSP/Src/w25q128.o \
./BSP/Src/wm8731.o 

C_DEPS += \
./BSP/Src/bpf_lpf.d \
./BSP/Src/cal.d \
./BSP/Src/csdr_app.d \
./BSP/Src/diag.d \
./BSP/Src/encoder.d \
./BSP/Src/fsdr_analog.d \
./BSP/Src/menu.d \
./BSP/Src/pe4302.d \
./BSP/Src/sdr_dsp.d \
./BSP/Src/si5351.d \
./BSP/Src/st7789.d \
./BSP/Src/usb_audio.d \
./BSP/Src/usb_cat.d \
./BSP/Src/w25q128.d \
./BSP/Src/wm8731.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/Src/%.o BSP/Src/%.su BSP/Src/%.cyclo: ../BSP/Src/%.c BSP/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -I"H:/CSDR/BSP/Inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BSP-2f-Src

clean-BSP-2f-Src:
	-$(RM) ./BSP/Src/bpf_lpf.cyclo ./BSP/Src/bpf_lpf.d ./BSP/Src/bpf_lpf.o ./BSP/Src/bpf_lpf.su ./BSP/Src/cal.cyclo ./BSP/Src/cal.d ./BSP/Src/cal.o ./BSP/Src/cal.su ./BSP/Src/csdr_app.cyclo ./BSP/Src/csdr_app.d ./BSP/Src/csdr_app.o ./BSP/Src/csdr_app.su ./BSP/Src/diag.cyclo ./BSP/Src/diag.d ./BSP/Src/diag.o ./BSP/Src/diag.su ./BSP/Src/encoder.cyclo ./BSP/Src/encoder.d ./BSP/Src/encoder.o ./BSP/Src/encoder.su ./BSP/Src/fsdr_analog.cyclo ./BSP/Src/fsdr_analog.d ./BSP/Src/fsdr_analog.o ./BSP/Src/fsdr_analog.su ./BSP/Src/menu.cyclo ./BSP/Src/menu.d ./BSP/Src/menu.o ./BSP/Src/menu.su ./BSP/Src/pe4302.cyclo ./BSP/Src/pe4302.d ./BSP/Src/pe4302.o ./BSP/Src/pe4302.su ./BSP/Src/sdr_dsp.cyclo ./BSP/Src/sdr_dsp.d ./BSP/Src/sdr_dsp.o ./BSP/Src/sdr_dsp.su ./BSP/Src/si5351.cyclo ./BSP/Src/si5351.d ./BSP/Src/si5351.o ./BSP/Src/si5351.su ./BSP/Src/st7789.cyclo ./BSP/Src/st7789.d ./BSP/Src/st7789.o ./BSP/Src/st7789.su ./BSP/Src/usb_audio.cyclo ./BSP/Src/usb_audio.d ./BSP/Src/usb_audio.o ./BSP/Src/usb_audio.su ./BSP/Src/usb_cat.cyclo ./BSP/Src/usb_cat.d ./BSP/Src/usb_cat.o ./BSP/Src/usb_cat.su ./BSP/Src/w25q128.cyclo ./BSP/Src/w25q128.d ./BSP/Src/w25q128.o ./BSP/Src/w25q128.su ./BSP/Src/wm8731.cyclo ./BSP/Src/wm8731.d ./BSP/Src/wm8731.o ./BSP/Src/wm8731.su

.PHONY: clean-BSP-2f-Src

