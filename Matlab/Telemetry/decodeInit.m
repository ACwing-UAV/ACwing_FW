function I = decodeInit(p)
%DECODEINIT  Payload TYPE 0x02 -> struct. Layout = tlm_init_t in C (137 bytes;
%            also accepts the older 97- and 113-byte formats).
p = uint8(p(:)).';
nf = floor((numel(p) - 1) / 4);
if numel(p) == 137, nf = 33; end
f = double(typecast(p(1:4*nf), 'single'));
I.q0        = f(1:4).';
I.gRef      = f(5:7).';
I.mRef      = f(8:10).';
I.b0        = f(11:13).';
I.sigmaAcc  = f(14);  I.sigmaMag  = f(15);
I.sigmaGyro = f(16);  I.sigmaBias = f(17);
I.nomAcc    = f(18);  I.nomMag    = f(19);
I.fs        = f(20);
I.accTol    = f(21);  I.magTol    = f(22);
I.p0att     = f(23);  I.p0bias    = f(24);
I.accNisGate = 0; I.magNisGate = 0; I.inflDecay = 0; I.dipTol = 0;
if nf >= 28
    I.accNisGate = f(25); I.magNisGate = f(26); I.inflDecay = f(27); I.dipTol = f(28);
end
I.inflMax = 1e4; I.lockoutPBump = 0; I.persistDecay = 0; I.persistRatio = 0.6; I.persistMin = 0.02;
I.lockoutUpdates = 0; I.graceUpdates = 0;
if numel(p) == 137
    I.inflMax = f(29); I.lockoutPBump = f(30); I.persistDecay = f(31);
    I.persistRatio = f(32); I.persistMin = f(33);
    u = double(typecast(p(133:136), 'uint16'));
    I.lockoutUpdates = u(1); I.graceUpdates = u(2);
end
fl = double(p(end));
I.estimateBias  = bitand(fl, 1)  > 0;
I.adaptive      = bitand(fl, 2)  > 0;
I.tiltComp      = bitand(fl, 4)  > 0;
I.decouple      = bitand(fl, 8)  > 0;
I.resetJacobian = bitand(fl, 16) > 0;
end
