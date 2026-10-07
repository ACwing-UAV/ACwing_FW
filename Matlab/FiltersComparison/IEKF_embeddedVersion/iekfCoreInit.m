function f = iekfCoreInit(cfg, seed)
%IEKFCOREINIT  Initialises the filter state. 1:1 mirror of iekf_init() in iekf.c.
%
%   cfg  : see iekfCoreDefaultConfig.m (same fields as iekf_config_t)
%   seed : .R0 (3x3) .b0 (3x1) .gRef .mRef (3x1) .nomAcc .nomMag
%
%   The state is a struct (value semantics): every iekfCore* function receives
%   it and returns it updated.

f.cfg = cfg;
f.R   = so3Orthonormalize(seed.R0);
f.b   = zeros(3,1);
if cfg.estimateBias, f.b = seed.b0(:); end
f.gRef = seed.gRef(:) / norm(seed.gRef);
f.mRef = seed.mRef(:) / norm(seed.mRef);
f.nomAcc = seed.nomAcc; if f.nomAcc <= 1e-6, f.nomAcc = 1; end
f.nomMag = seed.nomMag; if f.nomMag <= 1e-6, f.nomMag = 1; end
f.accInfl = 1; f.magInfl = 1;
f.dipRef  = acos(max(-1, min(1, f.gRef.' * f.mRef)));
f.lastAccDir = f.R.' * f.gRef;
f.accTrusted = false;
f.lastAccDev = 0; f.lastMagDev = 0; f.lastDipDev = 0;
f.lastAccNis = 0; f.lastMagNis = 0;
f.Elast = eye(3);
f.accGatedRun = 0; f.accGrace = 0; f.lockouts = 0;
f.innovWMean = zeros(3,1); f.innovWPow = 0; f.lastAccInnovB = zeros(3,1);
f.P = diag([cfg.p0Att*[1 1 1], cfg.estimateBias*cfg.p0Bias*[1 1 1]]);
end
