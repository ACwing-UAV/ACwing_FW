function S = decodeStatus(p)
%DECODESTATUS  Payload TYPE 0x03 (54 bytes) -> struct. Layout = tlm_status_t in C.
p = uint8(p);
u = double(typecast(p(1:48), 'uint32'));
S.uptime_s    = u(1)/1000;
S.irq_count   = u(2);   S.bus_err   = u(3);   S.fifo_ovr  = u(4);
S.tlm_drop    = u(5);   S.uart_drop = u(6);   S.proc_us_max = u(7)/64;   % SYSCLK 64 MHz
S.n_gyro      = u(8);   S.n_acc     = u(9);   S.n_mag     = u(10);
S.n_mag_dup   = u(11);  S.n_sh_nack = u(12);
w = double(typecast(p(49:52), 'uint16'));
S.words_max   = w(1);   S.calib_count = w(2);
S.imu_init    = double(typecast(p(53), 'int8'));
S.running     = double(p(54));
end
