function [Rest, bEst, Phist, info] = iekfAttitudeEmb(gyro, acc, mag, dt, gRef, mRef, opts)
%IEKFATTITUDEEMB  The original iekfAttitude.m filter, adapted to what is implemented in C.
%
%   [Rest, bEst, Phist] = iekfAttitudeEmb(gyro, acc, mag, dt, gRef, mRef, opts)
%
%   The original iekfAttitude.m stays UNCHANGED: it is the reference for the
%   comparison with the embedded filter (compareEmbeddedVsMatlab.m). This version
%   has the same signature and options (just swap the function name in
%   runSyntheticBenchmark/runRealData) plus the firmware extensions; the maths runs
%   through the iekfCore*.m functions, a line-by-line mirror of iekf.c.
%   Compared with the original:
%     - covariance: Joseph form + error reset after the retraction
%       + symmetrisation and diagonal floor (as in C);
%     - R re-orthonormalised at every step (as in C);
%     - new options (all off by default, except resetJacobian):
%         .accNisGate   NIS gate to inflate R_acc (C: IEKF_ACC_NIS_GATE, e.g. 4)
%         .magNisGate   same for the mag (not recommended)
%         .inflTau      inflation memory [s] (C: IEKF_INFL_TAU_S, e.g. 0.5)
%         .dipTol       tolerance on the magnetic dip [rad] (C: IEKF_DIP_TOL)
%         .decouple     acc tilt-only, mag yaw-only
%         .resetJacobian (default true)
%         .nomAcc/.nomMag  nominal magnitudes (default: first sample, as before)
%         .inflMax      inflation cap (C: IEKF_INFL_MAX, e.g. 100)
%         .lockoutTime  lockout protection [s] (C: IEKF_LOCKOUT_S, e.g. 0.5; 0 = off)
%         .lockoutPBump, .graceTime, .persistTau, .persistRatio, .persistMin
%       Time constants are converted per ACC UPDATE (as in the firmware); the
%       exact per-update values can be given instead:
%         .inflDecay, .lockoutUpdates, .graceUpdates, .persistDecay
%       Per-sample inputs produced by the firmware pipeline (optional, 1xN):
%         .accTrusted   stillness-detector flag for the acc update of sample k
%         .magExtraVar  extra mag variance of sample k (mag-age uncertainty)
%
%   The firmware propagates with angle increments at 960 Hz and averages the acc
%   over rotated windows (accwin): here the step is that of the input data, with
%   dtheta = gyro(:,k-1)*dt(k) as in the original version.
%
%   Outputs: Rest 3x3xN, bEst 3xN, Phist nxnxN (n = 6, or 3 without bias estimation),
%            info: .lockouts (lockout-protection events), .accInflMax (max acc inflation).

if nargin < 7, opts = struct(); end
d = struct('R0', eye(3), 'b0', zeros(3,1), 'estimateBias', true, 'P0', 1.0, ...
           'sigmaGyro', deg2rad(0.5), 'sigmaBias', 1e-3, 'sigmaAcc', 0.05, 'sigmaMag', 0.05, ...
           'adaptive', true, 'accTol', 0.05, 'magTol', 0.05, 'tiltCompensatedMag', false, ...
           'accNisGate', 0, 'magNisGate', 0, 'inflTau', 0, 'dipTol', 0, ...
           'decouple', false, 'resetJacobian', true, 'nomAcc', [], 'nomMag', [], ...
           'inflMax', 1e4, 'lockoutTime', 0, 'lockoutPBump', deg2rad(10)^2, 'graceTime', 0.25, ...
           'persistTau', 1.0, 'persistRatio', 0.6, 'persistMin', 0.02, ...
           'inflDecay', [], 'lockoutUpdates', [], 'graceUpdates', [], 'persistDecay', [], ...
           'accTrusted', [], 'magExtraVar', []);
fn = fieldnames(d);
for i = 1:numel(fn)
    if ~isfield(opts, fn{i}), opts.(fn{i}) = d.(fn{i}); end
end

N = size(gyro, 2);
useMag = ~isempty(mag);
if isscalar(dt), dtv = repmat(dt, 1, N); else, dtv = dt(:).'; end
okdt = isfinite(dtv) & dtv > 0;
hMed = median(dtv(okdt));
% The inflation memory and the lockout counters run PER ACC UPDATE (as in C):
% derive them from the interval between acc updates, not between samples
% (they coincide when every sample carries an acc reading).
tAll = cumsum([0, dtv(2:end) .* okdt(2:end)]);
iAcc = find(vecnorm(acc) > eps);
hAcc = hMed;
if numel(iAcc) > 1, hAcc = median(diff(tAll(iAcc))); end

cfg = iekfCoreDefaultConfig();
cfg.sigmaGyro = opts.sigmaGyro;  cfg.sigmaBias = opts.sigmaBias;
cfg.sigmaAcc  = opts.sigmaAcc;   cfg.sigmaMag  = opts.sigmaMag;
cfg.accTol    = opts.accTol;     cfg.magTol    = opts.magTol;
cfg.estimateBias = logical(opts.estimateBias);
cfg.adaptive     = logical(opts.adaptive);
cfg.tiltCompMag  = logical(opts.tiltCompensatedMag);
cfg.accNisGate = opts.accNisGate;  cfg.magNisGate = opts.magNisGate;
cfg.inflDecay  = 0;
if opts.inflTau > 0, cfg.inflDecay = exp(-hAcc / opts.inflTau); end
cfg.dipTol = opts.dipTol;  cfg.decouple = logical(opts.decouple);
cfg.inflMax = opts.inflMax;
if opts.lockoutTime > 0
    cfg.lockoutUpdates = round(opts.lockoutTime / hAcc);
    cfg.graceUpdates   = round(opts.graceTime / hAcc);
    cfg.lockoutPBump   = opts.lockoutPBump;
end
cfg.persistDecay = exp(-hAcc / opts.persistTau);
cfg.persistRatio = opts.persistRatio;  cfg.persistMin = opts.persistMin;
% exact per-update values (e.g. taken from the firmware INIT packet) win
if ~isempty(opts.inflDecay),      cfg.inflDecay      = opts.inflDecay;      end
if ~isempty(opts.lockoutUpdates), cfg.lockoutUpdates = opts.lockoutUpdates; end
if ~isempty(opts.graceUpdates),   cfg.graceUpdates   = opts.graceUpdates;   end
if ~isempty(opts.persistDecay),   cfg.persistDecay   = opts.persistDecay;   end
if cfg.lockoutUpdates > 0 && cfg.lockoutPBump == 0, cfg.lockoutPBump = opts.lockoutPBump; end
cfg.resetJacobian = logical(opts.resetJacobian);
if isscalar(opts.P0)
    cfg.p0Att = opts.P0; cfg.p0Bias = opts.P0;
else
    cfg.p0Att = 1; cfg.p0Bias = 1;          % overwritten below
end

seed.R0 = opts.R0;  seed.b0 = opts.b0(:);
seed.gRef = gRef(:);
if useMag, seed.mRef = mRef(:); else, seed.mRef = [1;0;0]; end
seed.nomAcc = opts.nomAcc; if isempty(seed.nomAcc), seed.nomAcc = norm(acc(:,1)); end
seed.nomMag = opts.nomMag;
if isempty(seed.nomMag), if useMag, seed.nomMag = norm(mag(:,1)); else, seed.nomMag = 1; end, end

f = iekfCoreInit(cfg, seed);
if ~isscalar(opts.P0)
    P = zeros(6); n0 = size(opts.P0, 1); P(1:n0, 1:n0) = opts.P0; f.P = P;
end

n = 3 + 3*cfg.estimateBias;
Rest = zeros(3,3,N); bEst = zeros(3,N); Phist = zeros(n,n,N);
Rest(:,:,1) = f.R; bEst(:,1) = f.b; Phist(:,:,1) = f.P(1:n,1:n);
info = struct('lockouts', 0, 'accInflMax', 1);

for k = 2:N
    h = dtv(k);
    if ~isfinite(h) || h <= 0, h = hMed; end
    f = iekfCorePropagate(f, gyro(:,k-1) * h, h);
    if ~isempty(opts.accTrusted), f.accTrusted = logical(opts.accTrusted(k)); end
    f = iekfCoreUpdateAcc(f, acc(:,k));
    if useMag
        xv = 0; if ~isempty(opts.magExtraVar), xv = opts.magExtraVar(k); end
        f = iekfCoreUpdateMag(f, mag(:,k), xv);
    end
    Rest(:,:,k) = f.R; bEst(:,k) = f.b; Phist(:,:,k) = f.P(1:n,1:n);
    info.accInflMax = max(info.accInflMax, f.accInfl);
end
info.lockouts = f.lockouts;
end
