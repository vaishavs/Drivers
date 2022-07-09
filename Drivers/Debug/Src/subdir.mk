################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/syscalls.c \
../Src/sysmem.c 

CPP_SRCS += \
../Src/gpio.cpp \
../Src/i2c.cpp \
../Src/rcc.cpp \
../Src/spi.cpp \
../Src/usart.cpp \
../Src/usart_demo.cpp 

C_DEPS += \
./Src/syscalls.d \
./Src/sysmem.d 

OBJS += \
./Src/gpio.o \
./Src/i2c.o \
./Src/rcc.o \
./Src/spi.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/usart.o \
./Src/usart_demo.o 

CPP_DEPS += \
./Src/gpio.d \
./Src/i2c.d \
./Src/rcc.d \
./Src/spi.d \
./Src/usart.d \
./Src/usart_demo.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.cpp Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++17 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -c -I../Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu17 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/gpio.d ./Src/gpio.o ./Src/i2c.d ./Src/i2c.o ./Src/rcc.d ./Src/rcc.o ./Src/spi.d ./Src/spi.o ./Src/syscalls.d ./Src/syscalls.o ./Src/sysmem.d ./Src/sysmem.o ./Src/usart.d ./Src/usart.o ./Src/usart_demo.d ./Src/usart_demo.o

.PHONY: clean-Src

