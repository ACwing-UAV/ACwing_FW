################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/%.o Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/%.su Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/%.cyclo: ../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/%.c Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/RTOS2/Include -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/app" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/control" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/iekf" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/platform" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/telemetry" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/drivers/common" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/third_party/cmsis_dsp/Include" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/third_party/st_lsm6dsv16x" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/drivers/imu_lsm6dsv16x" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-CMSIS_RTOS_V2

clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-CMSIS_RTOS_V2:
	-$(RM) ./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.cyclo ./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.d ./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.o ./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-CMSIS_RTOS_V2

