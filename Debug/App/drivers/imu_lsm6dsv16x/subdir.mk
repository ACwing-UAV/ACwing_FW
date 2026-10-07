################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/drivers/imu_lsm6dsv16x/imu_lsm6dsv16x.c 

OBJS += \
./App/drivers/imu_lsm6dsv16x/imu_lsm6dsv16x.o 

C_DEPS += \
./App/drivers/imu_lsm6dsv16x/imu_lsm6dsv16x.d 


# Each subdirectory must supply rules for building sources it contributes
App/drivers/imu_lsm6dsv16x/%.o App/drivers/imu_lsm6dsv16x/%.su App/drivers/imu_lsm6dsv16x/%.cyclo: ../App/drivers/imu_lsm6dsv16x/%.c App/drivers/imu_lsm6dsv16x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/RTOS2/Include -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/app" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/control" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/iekf" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/platform" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/telemetry" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/drivers/common" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/third_party/cmsis_dsp/Include" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/third_party/st_lsm6dsv16x" -I"C:/Users/acirillo/Documents/99 - PersonalProjects/01 - stm32_attitude_iekf/App/drivers/imu_lsm6dsv16x" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-drivers-2f-imu_lsm6dsv16x

clean-App-2f-drivers-2f-imu_lsm6dsv16x:
	-$(RM) ./App/drivers/imu_lsm6dsv16x/imu_lsm6dsv16x.cyclo ./App/drivers/imu_lsm6dsv16x/imu_lsm6dsv16x.d ./App/drivers/imu_lsm6dsv16x/imu_lsm6dsv16x.o ./App/drivers/imu_lsm6dsv16x/imu_lsm6dsv16x.su

.PHONY: clean-App-2f-drivers-2f-imu_lsm6dsv16x

