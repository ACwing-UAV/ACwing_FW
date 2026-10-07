function [f, gated] = iekfCoreUpdateMag(f, mag, extraVar)
%IEKFCOREUPDATEMAG  Magnetometer update. Mirror of iekf_update_mag_ex().
if nargin < 3, extraVar = 0; end
gated = false;
nm = norm(mag);
if nm < 1e-9, return; end
y = mag(:) / nm;

if f.cfg.tiltCompMag
    hw = f.R * y;                                   % tilt-compensated compass
    bw = [sqrt(hw(1)^2 + hw(2)^2); 0; hw(3)];
    yhat = f.R.' * bw;
else
    yhat = f.R.' * f.mRef;
end

dev = abs(nm - f.nomMag) / f.nomMag;
f.lastMagDev = dev;
r = f.cfg.sigmaMag^2;
if f.cfg.adaptive, r = r * (1 + (dev/f.cfg.magTol)^2); end
f.lastDipDev = 0;
if f.cfg.dipTol > 0                                 % measured dip vs calibration
    dip = acos(max(-1, min(1, y.' * f.lastAccDir)));
    f.lastDipDev = abs(dip - f.dipRef);
    r = r * (1 + (f.lastDipDev / f.cfg.dipTol)^2);
end
r = r + extraVar;

proj = 0; if f.cfg.decouple, proj = 2; end          % 2 = yaw only
[f, f.lastMagNis, f.magInfl] = iekfVectorUpdate(f, y, yhat, r, f.cfg.magNisGate, f.magInfl, proj);
gated = (dev > f.cfg.magTol) || (f.magInfl > 2) || ...
        (f.cfg.dipTol > 0 && f.lastDipDev > f.cfg.dipTol);
end
