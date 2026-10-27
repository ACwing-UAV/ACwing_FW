function cfg = iekfCoreDefaultConfig()
%IEKFCOREDEFAULTCONFIG  Defaults identical to iekf_default_config() in iekf.c.
cfg.sigmaGyro     = 0.5 * pi/180;
cfg.sigmaBias     = 1e-4;
cfg.sigmaAcc      = 0.05;
cfg.sigmaMag      = 0.05;
cfg.accTol        = 0.08;
cfg.magTol        = 0.08;
cfg.p0Att         = 1e-2;
cfg.p0Bias        = 1e-4;
cfg.estimateBias  = true;
cfg.adaptive      = true;
cfg.tiltCompMag   = false;
cfg.accNisGate    = 0;      % 0 = off
cfg.magNisGate    = 0;
cfg.inflDecay     = 0;
cfg.dipTol        = 0;      % rad, 0 = off
cfg.decouple      = false;
cfg.resetJacobian = true;
cfg.inflMax        = 1e4;   % inflation cap
cfg.lockoutUpdates = 0;     % lockout protection (0 = off)
cfg.lockoutPBump   = 0;
cfg.graceUpdates   = 0;
cfg.persistDecay   = 0;
cfg.persistRatio   = 0.6;
cfg.persistMin     = 0.02;
end
