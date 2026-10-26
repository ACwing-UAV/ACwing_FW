function out = magCheck(logFile, hardIronFw)
%MAGCHECK  Is the magnetometer calibration still valid? (diagnosis from a viewer log)
%
%   out = magCheck('iekf_log_....mat')
%   out = magCheck('iekf_log_....mat', [100.31 -0.56 -166.07])   % MAG_HARD_IRON in the firmware
%
%   The log holds the field AFTER MAG_AXIS_MAP and MAG_HARD_IRON (body frame).
%   With a valid calibration, rotating the board gives:
%     - |m| about constant (Earth field, 25-65 uT) -> samples on a sphere centred at 0;
%     - the direction of m turning in the body frame by the same angle as the gyro.
%   If MAG_HARD_IRON no longer matches the board (a magnet, the motors, a battery or
%   a screwdriver moved near the sensor, or a calibration made in a different place),
%   the sphere centre moves away from 0. When the offset is much larger than the
%   Earth field, m stays almost FIXED in the body frame. The filter then keeps the
%   same body axis on "North" whatever the orientation, and de-weights the mag
%   because |m| and the dip change as the board tilts.
%
%   Acquisition: firmware as is, iekfLiveViewer, board still for 2 s, then ~30 s of
%   slow rotations about ALL axes, away from metal objects.
%
%   out.verdict: 'ok', 'hard-iron-mismatch', 'no-data' or 'no-rotation'.
%
%   Requires expSO3.m (01 - Tools/00 - Matlab/FiltersComparison/IEKF_originalVersion, see initPath.m).

if nargin < 2, hardIronFw = []; end
S = load(logFile); L = S.L;
m = L.mag;  ok = all(isfinite(m), 1);
N = size(m, 2);

% ---- 1) sphere fit on the corrected field: residual offset and |B|
A = [2*m(:,ok).', ones(nnz(ok),1)];
p = A \ sum(m(:,ok).^2, 1).';
d = p(1:3);  rad = sqrt(p(4) + sum(d.^2));
nrm = vecnorm(m(:,ok));
fprintf('|m| logged      : mean %.1f uT, min %.1f, max %.1f (std %.1f)\n', ...
        mean(nrm), min(nrm), max(nrm), std(nrm));
fprintf('sphere fit      : residual offset [%.1f %.1f %.1f] uT (|.| = %.1f), |B| = %.1f uT\n', ...
        d, norm(d), rad);

% ---- 2) does m turn with the body as the gyro says?
t = double(L.t_us) * 1e-6;  dt = [0, diff(t)];
fs = 240; if isfield(L,'init') && ~isempty(L.init), fs = L.init.fs; end
bad = ~(dt > 0.2/fs & dt < 5/fs); dt(bad) = 1/fs; dt(1) = 0;
g = L.gyro - mean(L.gyro(:, 1:min(N, round(fs))), 2);   % bias from the first still second
R = eye(3); gyroAng = 0; magAng = 0; mPrev = m(:,1); k0 = 1;
for k = 2:N
    R = R * expSO3(g(:,k) * dt(k));
    if t(k) - t(k0) >= 0.25                    % compare every 0.25 s
        if all(isfinite([mPrev; m(:,k)]))
            gyroAng = gyroAng + norm(logm3(R));
            magAng  = magAng + acos(max(-1, min(1, dot(mPrev, m(:,k)) / (norm(mPrev)*norm(m(:,k))))));
        end
        R = eye(3); mPrev = m(:,k); k0 = k;
    end
end
ratio = magAng / max(gyroAng, eps);
fprintf('rotation        : gyro %.0f deg, mag direction %.0f deg (ratio %.2f; 0.5-1 expected, rotations about the field do not move it)\n', ...
        rad2deg(gyroAng), rad2deg(magAng), ratio);

% ---- 3) verdict
out = struct('residualOffset', d, 'B', rad, 'ratio', ratio, 'normMean', mean(nrm), ...
             'gyroAngle', gyroAng, 'magAngle', magAng, 'newHardIron', [], 'verdict', '');
if max(std(m(:,ok), 0, 2)) < 1 && gyroAng > pi
    out.verdict = 'no-data';
    fprintf(['\nVERDICT: the logged field is CONSTANT while the board rotates: the LIS2MDL ' ...
             'delivers no new data\n  (raw = 0 or frozen). Check n_mag / n_mag_dup / ' ...
             'n_sh_nack in STATUS, power-cycle the board.\n']);
elseif gyroAng < pi
    out.verdict = 'no-rotation';
    fprintf('\nVERDICT: not enough rotation in the log (%.0f deg): rotate the board about all axes.\n', ...
            rad2deg(gyroAng));
elseif norm(d) > 0.3 * rad || ratio < 0.4 || rad < 20 || rad > 80
    out.verdict = 'hard-iron-mismatch';
    fprintf('\nVERDICT: MAG_HARD_IRON no longer matches the board (offset %.0f uT vs field %.0f uT).\n', ...
            norm(d), rad);
    if ~isempty(hardIronFw)
        h = hardIronFw(:) + d;
        out.newHardIron = h;
        fprintf('  Corrected value (MAG_AXIS_MAP unchanged):\n');
        fprintf('  #define MAG_HARD_IRON  { %.2ff, %.2ff, %.2ff }\n', h);
    else
        fprintf('  New MAG_HARD_IRON = current MAG_HARD_IRON + [%.2f %.2f %.2f]\n', d);
    end
    fprintf(['  If the offset moved because something magnetic was brought near the board, ' ...
             'remove it or\n  recalibrate in the final set-up (calibMagAxes.m).\n']);
else
    out.verdict = 'ok';
    fprintf('\nVERDICT: calibration consistent (residual offset %.1f uT, |B| %.1f uT).\n', norm(d), rad);
end
end

function w = logm3(R)
c = max(-1, min(1, (trace(R) - 1) / 2)); a = acos(c);
v = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
if a < 1e-9, w = v/2; else, w = a / (2*sin(a)) * v; end
end
