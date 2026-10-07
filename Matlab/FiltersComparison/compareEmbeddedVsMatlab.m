function out = compareEmbeddedVsMatlab(captureFile, doPlot)
%COMPAREEMBEDDEDVSMATLAB  Compares the embedded IEKF (C, float32, on the board)
%   with the project's ORIGINAL MATLAB filter iekfAttitude.m, on the same inputs.
%
%   out = compareEmbeddedVsMatlab('iekf_cap_YYYYMMDD_HHMMSS.bin')
%
%   HOW IT WORKS
%   For every 240 Hz window the firmware (APP_TLM_REPLAY = 1) records the EXACT
%   inputs passed to the filter (960 Hz angle increments, acc mean, mag
%   measurement) and the resulting state (q, bias, diag P). Here those inputs are
%   rearranged into the format of the original iekfAttitude.m:
%     - every propagation becomes a sample: gyro(:,k-1) = dtheta/h, dt(k) = h
%       (the original propagates exactly with gyro(:,k-1)*dt(k));
%     - acc and mag sit in the samples where the firmware performed the update
%       and are 0 elsewhere: the original skips the update when |v| < eps;
%     - a mag arriving after the end-of-window acc update goes into an auxiliary
%       sample (1 ns propagation), so the end-of-window state is compared at the
%       same instant the firmware recorded it;
%     - sample 1 is never used for an update: it only gives the original the
%       nominal magnitudes (nomAcc, nomMag) used by the firmware.
%   Then iekfAttitude(...) is called UNMODIFIED and the attitude at the end of
%   every window is compared.
%
%   TWO MODES (chosen by the firmware, read from the INIT packet)
%   - APP_COMPARE_ORIGINAL = 1: C extensions disabled. Embedded and original
%     must match up to float32 precision -> PASS/FAIL.
%   - APP_COMPARE_ORIGINAL = 0: full configuration. The embedded filter is
%     validated against the EXTENDED MATLAB filter iekfAttitudeEmb.m (all the
%     improvements: NIS inflation, mag dip, stillness, error reset, mag age,
%     lockout) -> PASS/FAIL, with a coverage line telling which extensions the
%     capture exercised. The original is also run, to show how much the
%     extensions change the estimate.
%
%   The .bin is the raw capture written by tlmCapture.m (or by iekfLiveViewer
%   next to its .mat). Start the capture BEFORE resetting the board (window 0
%   is needed). Path: run Matlab/initPath.m (the original
%   iekfAttitude.m, expSO3.m, skewSym.m live in FiltersComparison/IEKF_originalVersion).
if nargin < 2, doPlot = true; end

% ---- dependencies: the ORIGINAL filter (iekfAttitude.m + expSO3.m + skewSym.m)
%      lives in FiltersComparison/IEKF_originalVersion
need = {'iekfAttitude', 'expSO3', 'skewSym'};
miss = need(cellfun(@(n) exist(n, 'file') ~= 2, need));
if ~isempty(miss)
    error(['Not found on the path: %s.\nAdd the folder of the ORIGINAL MATLAB ' ...
           'implementation (the one with iekfAttitude.m, expSO3.m, skewSym.m), e.g. ' ...
           'run Matlab/initPath.m.\n' ...
           'IEKF_embeddedVersion only contains iekfAttitudeEmb.m, the version ' ...
           'extended as in C.'], strjoin(miss, ', '));
end
w = which('iekfAttitude');
fprintf('Reference filter: %s\n', w);
here = fullfile(fileparts(mfilename('fullpath')), 'IEKF_originalVersion');
if ~strcmp(fileparts(w), here)
    warning(['iekfAttitude.m is not the one in IEKF_originalVersion (%s): another copy ' ...
             'shadows the original. Check the path (which -all iekfAttitude).'], here);
end

fid = fopen(captureFile, 'r');
if fid < 0, error('Cannot open %s', captureFile); end
bytes = fread(fid, Inf, 'uint8=>uint8').';
fclose(fid);

[frames, ~] = parseTelemetry(bytes);
types = double([frames.type]);

% ---------------- capture integrity ----------------
fprintf('\nCapture: %d bytes, %d valid frames\n', numel(bytes), numel(frames));
fprintf('  STATE %d   INIT %d   STATUS %d   REPLAY %d\n', ...
        sum(types==1), sum(types==2), sum(types==3), sum(types==4));
iSt = find(types == 3);
upt = zeros(1, numel(iSt));
for k = 1:numel(iSt), S = decodeStatus(frames(iSt(k)).payload); upt(k) = S.uptime_s; end
if ~isempty(iSt)
    S = decodeStatus(frames(iSt(end)).payload);
    fprintf('  firmware (last STATUS): frames dropped UART %d, telemetry queue %d\n', ...
            S.uart_drop, S.tlm_drop);
end

iRep = find(types == 4);
if isempty(iRep), error('No REPLAY packets: firmware built with APP_TLM_REPLAY = 1?'); end
% last window 0 in the capture = last filter start
first = 0;
for k = fliplr(iRep)
    r = decodeReplay(frames(k).payload);
    if r.win == 0, first = k; break; end
end
if first == 0, error('Window 0 missing: start the capture BEFORE resetting the board.'); end

% the INIT must belong to the SAME boot: after the last reset (uptime restarting
% from zero in the STATUS packets) and before window 0
resetIdx = 0;
for k = 2:numel(iSt)
    if iSt(k) < first && upt(k) < upt(k-1), resetIdx = iSt(k); end
end
iInit = find(types == 2 & (1:numel(types)) > resetIdx & (1:numel(types)) < first, 1, 'last');
if isempty(iInit)
    error(['The INIT of this boot was lost (UART saturated): without the right initial ' ...
           'conditions the comparison is meaningless. Repeat the capture.']);
end
I = decodeInit(frames(iInit).payload);
% consistency: all INITs after window 0 must be identical to this one
iLater = find(types == 2 & (1:numel(types)) > first);
for k = iLater
    if ~isequal(frames(k).payload, frames(iInit).payload)
        fprintf(2, '  WARNING: different INITs in the capture (reboot halfway?).\n');
        break;
    end
end

% ---------------- contiguous windows from 0 ----------------
reps = iRep(iRep >= first);
W = cell(1, numel(reps)); nW = 0; gaps = 0; firstGap = -1;
for n = 1:numel(reps)
    r = decodeReplay(frames(reps(n)).payload);
    if r.win ~= nW
        if firstGap < 0, firstGap = nW; end
        gaps = gaps + 1;
        break;            % the filter state cannot be rebuilt beyond a gap
    end
    nW = nW + 1; W{nW} = r;
end
W = W(1:nW);
wAll = zeros(1, numel(reps));
for n = 1:numel(reps), rr = decodeReplay(frames(reps(n)).payload); wAll(n) = rr.win; end
lastWin = max(wAll);
nLost = lastWin + 1 - numel(unique(wAll));
fprintf('  REPLAY windows from 0: %d received, %d lost\n', numel(unique(wAll)), nLost);
if firstGap >= 0
    fprintf(2, ['  Window %d lost: comparison limited to the first %d windows (%.1f s).\n' ...
                '  The filter state cannot be rebuilt beyond a gap.\n'], ...
            firstGap, nW, nW/240);
end
if nW < 240
    error('Less than 1 s of contiguous windows: capture too disturbed, repeat it.');
end

% ---------------- inputs in the original's format ----------------
nP = 0; nMg = 0;
for n = 1:nW
    ops = [W{n}.ops.op];
    nP = nP + sum(ops == 'P'); nMg = nMg + sum(ops == 'M');
end
M = nP + nMg + 8;
gyro = zeros(3,M); acc = zeros(3,M); mag = zeros(3,M); dt = zeros(1,M);
accTr = false(1,M); magXV = zeros(1,M);  % per-sample inputs for the extended filter
acc(:,1) = [0; 0; I.nomAcc];             % only for the nominal magnitudes
mag(:,1) = [I.nomMag; 0; 0];
s = 1; winSample = zeros(1,nW); tWin = zeros(1,nW); tacc = 0;
nExtra = 0; nTrusted = 0; nTiny = 0;
for n = 1:nW
    for o = W{n}.ops
        switch o.op
            case 'P'
                s = s + 1; gyro(:,s-1) = o.v(1:3) / o.v(4); dt(s) = o.v(4); tacc = tacc + o.v(4);
            case 'A'
                acc(:,s) = o.v(:); accTr(s) = o.trusted; nTrusted = nTrusted + o.trusted;
            case 'M'
                % The mag needs its own sample if the current one is sample 1,
                % already holds a mag, or holds the acc update that closes a
                % window: the firmware records the state RIGHT AFTER the acc,
                % and a mag arriving later belongs to the next window.
                % The auxiliary sample propagates for 1 ns: no effect.
                if s == 1 || any(mag(:,s) ~= 0) || any(acc(:,s) ~= 0)
                    s = s + 1; gyro(:,s-1) = 0; dt(s) = 1e-9; nTiny = nTiny + 1;
                end
                mag(:,s) = o.v(1:3); magXV(s) = o.v(4); nExtra = nExtra + (o.v(4) ~= 0);
        end
    end
    winSample(n) = s; tWin(n) = tacc;
end
gyro = gyro(:,1:s); acc = acc(:,1:s); mag = mag(:,1:s); dt = dt(1:s);
accTr = accTr(1:s); magXV = magXV(1:s);

R0 = quat2rotmLocal(I.q0);
if I.estimateBias
    P0 = diag([I.p0att*[1 1 1], I.p0bias*[1 1 1]]);
else
    P0 = I.p0att * eye(3);
end
opts = struct('R0', R0, 'b0', I.b0(:), 'estimateBias', I.estimateBias, 'P0', P0, ...
              'sigmaGyro', I.sigmaGyro, 'sigmaBias', I.sigmaBias, ...
              'sigmaAcc', I.sigmaAcc, 'sigmaMag', I.sigmaMag, ...
              'adaptive', I.adaptive, 'accTol', I.accTol, 'magTol', I.magTol, ...
              'tiltCompensatedMag', I.tiltComp);

extActive = I.accNisGate > 0 || I.magNisGate > 0 || I.dipTol > 0 || ...
            I.resetJacobian || I.decouple || nExtra > 0 || I.lockoutUpdates > 0;

% ---------------- ORIGINAL MATLAB filter ----------------
[Ro, bo, Po] = iekfAttitude(gyro, acc, mag, dt, I.gRef(:), I.mRef(:), opts);

ext = [];
dAng = zeros(1,nW); dBias = zeros(1,nW); dP = zeros(1,nW); gatedA = false(1,nW);
zAng = zeros(1,nW); zBias = zeros(1,nW);   % differences in units of the filter sigma
for n = 1:nW
    k = winSample(n);
    dAng(n)  = quatAngleDeg(W{n}.q(:), so3ToQuat(Ro(:,:,k)));
    dBias(n) = max(abs(bo(:,k) - W{n}.b(:))) * 180/pi;
    pd = diag(Po(:,:,k)); np = numel(pd);
    dP(n)    = max(abs(pd - W{n}.Pd(1:np)) ./ max(abs(pd), 1e-30));
    gatedA(n) = W{n}.accInfl > 2;
    zAng(n)  = deg2rad(dAng(n)) / sqrt(min(pd(1:3)));
    if np == 6
        zBias(n) = max(abs(bo(:,k) - W{n}.b(:)) ./ sqrt(pd(4:6)));
    end
end

fprintf('\nEmbedded (C, float32) vs ORIGINAL iekfAttitude.m: %d windows, %.1f s\n', nW, tWin(end));
fprintf('  inputs  : %d samples (%d propagations, %d auxiliary mag samples)\n', s, nP, nTiny);
fprintf('  attitude: max %.2e deg   RMS %.2e deg\n', max(dAng), sqrt(mean(dAng.^2)));
fprintf('  bias    : max %.2e deg/s\n', max(dBias));
fprintf('  diag(P) : max relative error %.2e\n', max(dP));

% CRITERION. Absolute thresholds do not work: float32 and double drift apart
% slowly (the bias estimate integrates the round-off), the more so the longer and
% more dynamic the capture. What matters is whether the difference is negligible
% COMPARED WITH THE FILTER'S OWN UNCERTAINTY: it is measured in sigma
% (sqrt of the diagonal of P) and accepted up to 10%.
%   < 0.10 sigma  -> match (numerical precision only)
%   > 0.5  sigma  -> different inputs or initial conditions (e.g. wrong INIT)
%   in between    -> to inspect (equations differing at specific points?)
TH = struct('z', 0.10, 'zBad', 0.5, 'P', 1e-3);
zA = max(zAng); zB = max(zBias);
fprintf('  in filter sigma: attitude max %.3f, bias max %.3f (threshold %.2f)\n', zA, zB, TH.z);
if ~extActive
    ok = zA < TH.z && zB < TH.z && max(dP) < TH.P;
    if ok
        fprintf('  RESULT: MATCH: differences < %.0f%% of the filter uncertainty,\n', 100*TH.z);
        fprintf('          due only to float32 vs double. The C code implements the same equations.\n');
    else
        fprintf(2, '  RESULT: DIVERGE beyond numerical precision.\n');
    end
    adapt = replayAdapted(I, W);
    fprintf('  Check: same calls replayed with the MATLAB core written after the C code:\n');
    fprintf('    attitude max %.2e deg, bias max %.2e deg/s, diag(P) max rel %.2e\n', ...
            max(adapt.dAng), max(adapt.dBias), max(adapt.dP));
    if ~ok
        bad = find(zAng > TH.z | zBias > TH.z | dP > TH.P, 1);
        fprintf('  First window above threshold: %d (t = %.2f s), operations: %s\n', ...
                bad - 1, tWin(bad), [W{bad}.ops.op]);
        if max(zA, zB) > TH.zBad && bad <= 5
            fprintf(2, '  DIAGNOSIS: large divergence from the start -> INIT/initial conditions.\n');
        elseif max(zA, zB) > TH.zBad
            fprintf(2, '  DIAGNOSIS: large divergence halfway -> inspect that window.\n');
        else
            fprintf(2, '  DIAGNOSIS: small, growing divergence -> numerical drift.\n');
        end
    end
else
    fprintf('  FULL configuration (extensions active): differences w.r.t. the ORIGINAL are expected.\n');
    fprintf('    during acc inflation (%d windows): max %.3f deg\n', sum(gatedA), maxOr0(dAng(gatedA)));
    fprintf('    rest of the time                  : max %.3f deg, RMS %.3f deg\n', ...
            maxOr0(dAng(~gatedA)), sqrt(mean(dAng(~gatedA).^2)));
    adapt = [];

    % ---------------- EXTENDED MATLAB filter (iekfAttitudeEmb) ----------------
    % Same batch inputs as the original, plus the extension parameters from the
    % INIT packet (exact per-update constants) and the per-sample inputs produced
    % by the firmware pipeline (stillness flag, mag-age variance).
    optsE = opts;
    optsE.accNisGate = I.accNisGate;  optsE.magNisGate = I.magNisGate;
    optsE.dipTol = I.dipTol;          optsE.decouple = I.decouple;
    optsE.resetJacobian = I.resetJacobian;
    optsE.nomAcc = I.nomAcc;          optsE.nomMag = I.nomMag;
    optsE.inflMax = I.inflMax;        optsE.inflDecay = I.inflDecay;
    optsE.lockoutUpdates = I.lockoutUpdates;  optsE.graceUpdates = I.graceUpdates;
    optsE.lockoutPBump = I.lockoutPBump;      optsE.persistDecay = I.persistDecay;
    optsE.persistRatio = I.persistRatio;      optsE.persistMin = I.persistMin;
    optsE.accTrusted = accTr;         optsE.magExtraVar = magXV;
    [Rx, bx, Px, infoX] = iekfAttitudeEmb(gyro, acc, mag, dt, I.gRef(:), I.mRef(:), optsE);
    ext = windowMetrics(W, winSample, Rx, bx, Px);
    zAx = max(ext.zAng); zBx = max(ext.zBias);
    fprintf('\nEmbedded (C, float32) vs EXTENDED iekfAttitudeEmb.m (MATLAB, double):\n');
    fprintf('  attitude: max %.2e deg   RMS %.2e deg\n', max(ext.dAng), sqrt(mean(ext.dAng.^2)));
    fprintf('  bias    : max %.2e deg/s\n', max(ext.dBias));
    fprintf('  diag(P) : max relative error %.2e\n', max(ext.dP));
    fprintf('  in filter sigma: attitude max %.3f, bias max %.3f (threshold %.2f)\n', zAx, zBx, TH.z);
    fprintf(['  coverage: %d acc updates stillness-trusted, %d windows with acc inflated ' ...
             '(max x%.0f), %d lockout events, %d mag updates with age variance\n'], ...
            sum(accTr), sum(gatedA), infoX.accInflMax, infoX.lockouts, nExtra);
    ok = zAx < TH.z && zBx < TH.z && max(ext.dP) < TH.P;
    if ok
        fprintf('  RESULT: MATCH: the C code implements the extended filter (all improvements)\n');
        fprintf('          up to float32 vs double.\n');
    else
        fprintf(2, '  RESULT: DIVERGE beyond numerical precision.\n');
        bad = find(ext.zAng > TH.z | ext.zBias > TH.z | ext.dP > TH.P, 1);
        fprintf('  First window above threshold: %d (t = %.2f s), operations: %s\n', ...
                bad - 1, tWin(bad), [W{bad}.ops.op]);
    end
end

% ZYX Euler angles [roll; pitch; yaw], in degrees. The REFERENCE is the filter
% the embedded one must match: the original in mode 1, the extended in mode 0.
% No unwrap: near pitch = +-90 deg (gimbal lock) roll and yaw jump by 180 deg
% and unwrap would accumulate them into multiples of 360 deg. Angles in
% [-180, 180], difference wrapped into the same interval.
if isempty(ext), Rref = Ro(:,:,winSample); refName = 'original iekfAttitude.m';
else,            Rref = ext.R;             refName = 'extended iekfAttitudeEmb.m';
end
eE = zeros(3, nW); eO = zeros(3, nW); eR = zeros(3, nW);
for n = 1:nW
    eE(:,n) = eulZYX(quat2rotmLocal(W{n}.q));
    eO(:,n) = eulZYX(Ro(:,:,winSample(n)));
    eR(:,n) = eulZYX(Rref(:,:,n));
end
eE = rad2deg(eE);  eO = rad2deg(eO);  eR = rad2deg(eR);
dE = mod(eE - eR + 180, 360) - 180;
nearLock = abs(eE(2,:)) > 80;            % roll/yaw poorly defined here
fprintf('  angles  : max |embedded - %s| roll %.1e, pitch %.1e, yaw %.1e deg\n', ...
        refName, max(abs(dE), [], 2));

if doPlot
    nm = {'roll', 'pitch', 'yaw'};
    figure('Name', ['Angles: embedded vs ' refName], 'Color', 'w');
    for i = 1:3
        subplot(4,1,i); hold on; grid on;
        hs = plot(tWin, breakJumps(eE(i,:)), 'LineWidth', 1.6);
        hs(2) = plot(tWin, breakJumps(eR(i,:)), '--', 'LineWidth', 1.2);
        lg = {'embedded', refName};
        if ~isempty(ext)
            hs(end+1) = plot(tWin, breakJumps(eO(i,:)), ':', 'LineWidth', 1.0); %#ok<AGROW>
            lg{end+1} = 'original iekfAttitude.m'; %#ok<AGROW>
        end
        if i ~= 2 && any(nearLock)
            hs(end+1) = plot(tWin(nearLock), eE(i,nearLock), 'k.', 'MarkerSize', 3); %#ok<AGROW>
            lg{end+1} = '|pitch| > 80 deg (gimbal lock)'; %#ok<AGROW>
        end
        ylim([-180 180] / (1 + (i == 2)));  yticks(-180:90:180);
        ylabel([nm{i} ' [deg]']);
        if i == 1
            title(['Embedded (C, float32, board) vs ' refName ' (MATLAB, double)']);
            legend(hs, lg, 'Location', 'best');
        end
    end
    subplot(4,1,4); hold on; grid on;
    plot(tWin, dE.', 'LineWidth', 1.0);
    ylabel('difference [deg]'); xlabel('filter time [s]');
    legend('roll', 'pitch', 'yaw', 'Location', 'best');
    title(sprintf('Embedded - %s: max |diff| %.1e deg', refName, max(abs(dE(:)))));

    % --- quaternions with sign made continuous: no jumps, no gimbal lock ---
    qE = zeros(4, nW); qR = zeros(4, nW); qO = zeros(4, nW);
    for n = 1:nW
        qE(:,n) = W{n}.q(:);
        qR(:,n) = so3ToQuat(Rref(:,:,n));
        qO(:,n) = so3ToQuat(Ro(:,:,winSample(n)));
    end
    qE = continuousQuat(qE);
    qR = qR .* sign(sum(qR .* qE, 1));          % same sign as the embedded one
    qO = qO .* sign(sum(qO .* qE, 1));
    qn = {'w', 'x', 'y', 'z'};
    figure('Name', ['Quaternion: embedded vs ' refName], 'Color', 'w');
    for i = 1:4
        subplot(4,1,i); hold on; grid on;
        plot(tWin, qE(i,:), 'LineWidth', 1.6);
        plot(tWin, qR(i,:), '--', 'LineWidth', 1.2);
        if ~isempty(ext), plot(tWin, qO(i,:), ':', 'LineWidth', 1.0); end
        ylabel(['q_' qn{i}]); ylim([-1 1]);
        if i == 1
            title('Body->world quaternion (continuous, no singularity)');
            if isempty(ext)
                legend('embedded', refName, 'Location', 'best');
            else
                legend('embedded', refName, 'original iekfAttitude.m', 'Location', 'best');
            end
        end
    end
    xlabel('filter time [s]');

    figure('Name', 'Embedded vs MATLAB', 'Color', 'w');
    subplot(2,1,1);
    semilogy(tWin, max(dAng, 1e-9), 'LineWidth', 1.1); hold on; grid on;
    if ~isempty(ext), semilogy(tWin, max(ext.dAng, 1e-9), 'LineWidth', 1.1); end
    if any(gatedA)
        yl = get(gca, 'YLim'); tg = tWin(gatedA);
        plot(tg, yl(2)*ones(size(tg)), 'r.', 'MarkerSize', 4);
    end
    ylabel('attitude difference [deg]');
    if isempty(ext)
        title('Embedded vs original iekfAttitude.m');
    else
        title('Embedded vs original (blue) and vs extended (orange); red = acc de-weighted');
    end
    subplot(2,1,2);
    bEmb = zeros(3, nW); for n = 1:nW, bEmb(:,n) = W{n}.b(:); end
    plot(tWin, rad2deg(bEmb).', 'LineWidth', 1.1);
    hold on; grid on; set(gca, 'ColorOrderIndex', 1);
    if isempty(ext), bRef = bo(:, winSample); else, bRef = bx(:, winSample); end
    plot(tWin, rad2deg(bRef).', '--', 'LineWidth', 1.0);
    xlabel('filter time [s]'); ylabel('bias [deg/s]');
    legend('emb x', 'emb y', 'emb z', 'ref x', 'ref y', 'ref z');
end

out = struct('eulEmb', eE, 'eulMatlab', eO, 'eulRef', eR, 't', tWin, ...
             'dAng', dAng, 'dBias', dBias, 'dP', dP, 'ok', ok, ...
             'extensions', extActive, 'adapted', adapt, 'extended', ext, 'init', I);
end

% =========================================================================
function A = replayAdapted(I, W)
cfg = iekfCoreDefaultConfig();
cfg.sigmaGyro = I.sigmaGyro; cfg.sigmaBias = I.sigmaBias;
cfg.sigmaAcc  = I.sigmaAcc;  cfg.sigmaMag  = I.sigmaMag;
cfg.accTol    = I.accTol;    cfg.magTol    = I.magTol;
cfg.p0Att     = I.p0att;     cfg.p0Bias    = I.p0bias;
cfg.estimateBias = I.estimateBias; cfg.adaptive = I.adaptive; cfg.tiltCompMag = I.tiltComp;
cfg.accNisGate = I.accNisGate; cfg.magNisGate = I.magNisGate;
cfg.inflDecay  = I.inflDecay;  cfg.dipTol = I.dipTol;
cfg.decouple   = I.decouple;   cfg.resetJacobian = I.resetJacobian;
cfg.inflMax = I.inflMax; cfg.lockoutUpdates = I.lockoutUpdates; cfg.lockoutPBump = I.lockoutPBump;
cfg.graceUpdates = I.graceUpdates; cfg.persistDecay = I.persistDecay;
cfg.persistRatio = I.persistRatio; cfg.persistMin = I.persistMin;
seed = struct('R0', quat2rotmLocal(I.q0), 'b0', I.b0(:), 'gRef', I.gRef(:), ...
              'mRef', I.mRef(:), 'nomAcc', I.nomAcc, 'nomMag', I.nomMag);
f = iekfCoreInit(cfg, seed);
n = numel(W);
A.dAng = zeros(1,n); A.dBias = zeros(1,n); A.dP = zeros(1,n);
for w = 1:n
    for o = W{w}.ops
        switch o.op
            case 'P', f = iekfCorePropagate(f, o.v(1:3), o.v(4));
            case 'A', f.accTrusted = o.trusted; f = iekfCoreUpdateAcc(f, o.v);
            case 'M', f = iekfCoreUpdateMag(f, o.v(1:3), o.v(4));
        end
    end
    A.dAng(w)  = quatAngleDeg(W{w}.q(:), so3ToQuat(f.R));
    A.dBias(w) = max(abs(f.b - W{w}.b(:))) * 180/pi;
    pd = diag(f.P);
    A.dP(w)    = max(abs(pd - W{w}.Pd(:)) ./ max(abs(pd), 1e-30));
end
end

function m = maxOr0(x)
if isempty(x), m = 0; else, m = max(x); end
end

function a = quatAngleDeg(q1, q2)
%QUATANGLEDEG  Angle between two attitudes. Do NOT use 2*acos(|q1.q2|): near 1
%   acos amplifies the firmware's float32 round-off (~0.05 deg of spurious error).
w1 = q1(1); v1 = q1(2:4); w2 = q2(1); v2 = q2(2:4);
w = w1*w2 + v1.'*v2;
v = w1*v2 - w2*v1 - cross(v1, v2);
a = 2 * atan2(norm(v), abs(w)) * 180/pi;
end

function r = ternary(c, a, b)
if c, r = a; else, r = b; end
end

function e = eulZYX(R)
%EULZYX  R -> [roll; pitch; yaw] (ZYX sequence), radians.
e = [atan2(R(3,2), R(3,3)); asin(max(-1, min(1, -R(3,1)))); atan2(R(2,1), R(1,1))];
end

function y = breakJumps(y)
%BREAKJUMPS  Breaks the line where the angle jumps by more than 180 deg (+180 to
%   -180 crossing, or roll/yaw flip at gimbal lock): no spurious vertical segments
%   in the plot. The data are left untouched.
j = find(abs(diff(y)) > 180);
y(j+1) = NaN;
end

function q = continuousQuat(q)
%CONTINUOUSQUAT  q and -q are the same rotation: picks the sign of each sample
%   to stay close to the previous one, so the curves are continuous.
for n = 2:size(q,2)
    if q(:,n).' * q(:,n-1) < 0, q(:,n) = -q(:,n); end
end
end

function m = windowMetrics(W, winSample, Rx, bx, Px)
%WINDOWMETRICS  Embedded state vs a MATLAB filter output at the end of every window.
n = numel(W);
m.dAng = zeros(1,n); m.dBias = zeros(1,n); m.dP = zeros(1,n);
m.zAng = zeros(1,n); m.zBias = zeros(1,n); m.R = zeros(3,3,n);
for w = 1:n
    k = winSample(w);
    m.R(:,:,w) = Rx(:,:,k);
    m.dAng(w)  = quatAngleDeg(W{w}.q(:), so3ToQuat(Rx(:,:,k)));
    m.dBias(w) = max(abs(bx(:,k) - W{w}.b(:))) * 180/pi;
    pd = diag(Px(:,:,k)); np = numel(pd);
    m.dP(w)    = max(abs(pd - W{w}.Pd(1:np)) ./ max(abs(pd), 1e-30));
    m.zAng(w)  = deg2rad(m.dAng(w)) / sqrt(min(pd(1:3)));
    if np == 6, m.zBias(w) = max(abs(bx(:,k) - W{w}.b(:)) ./ sqrt(pd(4:6))); end
end
end
