function msg = statusDiagnosis(S)
%STATUSDIAGNOSIS  Translates a STATUS packet into the first broken link of the chain
%   IMU -> interrupt -> I2C/DMA -> FIFO -> sensor hub -> calibration -> filter.
codes = containers.Map({-1,-2,-3,-4,-5}, ...
    {'I2C bus error towards the LSM6DSV16X (pins PB8/PB9, power, jumpers)', ...
     'wrong LSM6DSV16X WHO_AM_I (address 0x6B? right board?)', ...
     'LIS2MDL does not answer through the sensor hub (J4/J5 not in Mode 3?)', ...
     'sensor-hub timeout (J4/J5 in Mode 3? LIS2MDL powered?)', ...
     'invalid arguments'});
if S.imu_init == 1
    msg = 'IMU: initialisation in progress or stuck';
elseif S.imu_init < 0
    msg = ['IMU: ' codes(S.imu_init)];
elseif S.irq_count == 0
    msg = 'IMU OK but no INT1 interrupts: PA15/EXTI15, label IMU_INT1, NVIC EXTI15_10, debug = Serial Wire';
elseif S.bus_err > 0 && S.n_gyro == 0
    msg = 'Interrupts OK but FIFO reads fail: DMA I2C1_RX, I2C1 event/error interrupts, HAL_I2C_MemRxCpltCallback';
elseif S.n_gyro == 0
    msg = 'Reads OK but no gyro data in the FIFO: FIFO configuration';
elseif S.n_mag == 0
    msg = 'Gyro/acc OK but no mag data from the sensor hub: Mode 3 (J4/J5 on 5-6), n_sh_nack';
elseif ~S.running
    msg = sprintf('Data OK, calibration %d/100: keep the board STILL (restarts if it moves)', S.calib_count);
else
    msg = 'Filter running: STATE packets should be arriving';
end
end
