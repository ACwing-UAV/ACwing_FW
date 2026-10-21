function out = calibMagAxes(logFile, gapSec)
%CALIBMAGAXES  Magnetometer orientation w.r.t. the gyroscope + hard-iron, from data.
%
%   out = calibMagAxes('iekf_log_....mat')
%
%   PRINCIPLE: a world-fixed vector (the magnetic field), seen from a rotating
%   body, changes exactly as the gyroscope says:
%       m(t_j) = D_ij' * m(t_i),    D_ij = rotation integrated from the gyro between i and j
%   If the magnetometer axes are rotated w.r.t. the gyro axes
%   (mag_body = M * mag_sensor) this relation holds only with the right M.
%   All 24 "signed permutation" M are tried (LIS2MDL axes parallel to the
%   LSM6DSV16X axes, as on the IKS4A1) and for each one the hard-iron offset h is
%   estimated by least squares:
%       (I - D') * M * h = M*m_j - D'*M*m_i
%   The M with the smallest residual wins, and it must win by a clear margin.
%
%   ACQUISITION: firmware with MAG_AXIS_MAP = identity and MAG_HARD_IRON = 0,
%   iekfLiveViewer, board still for 2 s, then ~40 s of slow/moderate rotations
%   about ALL axes (tilts included), away from metal objects.
%
%   Requires expSO3.m (Matlab/FiltersComparison/IEKF_originalVersion, see Matlab/initPath.m).

if nargin < 2, gapSec = 0.2; end
S = load(logFile); L = S.L;
N = numel(L.seq);

% gyro bias: the initial one from the static calibration (independent of the mag)
if ~isempty(L.init), b = L.init.b0(:); else, b = L.bias(:,1); end
g = L.gyro - b;
m = L.mag;

t  = double(L.t_us) * 1e-6;
dt = [0, diff(t)];
fs = 240; if ~isempty(L.init), fs = L.init.fs; end
bad = ~(dt > 0.2/fs & dt < 5/fs); dt(bad) = 1/fs; dt(1) = 0;

% integrated absolute rotation (only the relative one between close pairs is needed)
R = zeros(3,3,N); R(:,:,1) = eye(3);
for k = 2:N
    R(:,:,k) = R(:,:,k-1) * expSO3(g(:,k) * dt(k));
end

% pairs (i, j) of NEW mag samples ~gapSec apart
idx = find(bitand(L.flags, 2) > 0);
K = round(gapSec * fs);
P = zeros(0,2);
for a = 1:numel(idx)
    jj = idx(idx >= idx(a) + K);
    if ~isempty(jj), P(end+1,:) = [idx(a), jj(1)]; end %#ok<AGROW>
end
if size(P,1) < 50, error('Too few samples: the log is too short or has no motion.'); end
Dt = zeros(3,3,size(P,1));
for p = 1:size(P,1), Dt(:,:,p) = (R(:,:,P(p,1)).' * R(:,:,P(p,2))).'; end

% angular excursion: without rotation about several axes M and h are not identifiable
ang = zeros(1,size(P,1));
for p = 1:size(P,1), ang(p) = acosd(max(-1,min(1,(trace(Dt(:,:,p))-1)/2))); end
fprintf('Pairs used: %d   mean rotation per pair: %.1f deg\n', size(P,1), mean(ang));

cands = signedPermutations();
res = zeros(1,numel(cands)); hs = zeros(3,numel(cands));
for c = 1:numel(cands)
    M = cands{c};
    A = zeros(3*size(P,1),3); r = zeros(3*size(P,1),1);
    for p = 1:size(P,1)
        D = Dt(:,:,p); i = P(p,1); j = P(p,2);
        A(3*p-2:3*p,:) = (eye(3) - D) * M;
        r(3*p-2:3*p)   = M*m(:,j) - D*M*m(:,i);
    end
    h = A \ r;
    hs(:,c) = h;
    res(c) = sqrt(mean((A*h - r).^2));
end
[res, ord] = sort(res);
Mb = cands{ord(1)}; hb = hs(:,ord(1));
hBody = Mb * hb;

fprintf('\nResidual (uT) of the best 4 hypotheses out of 24:\n');
for k = 1:4, fprintf('  %d) %.2f\n', k, res(k)); end
ratio = res(2)/res(1);
if ratio < 2
    warning(['The best hypothesis is not clearly separated from the second (ratio %.1f): ' ...
             'repeat with more rotations about all axes.'], ratio);
end

fprintf('\nmag_body = M * mag_sensor, with\nM =\n'); disp(Mb);
fprintf('offset hard-iron (body frame): %.2f %.2f %.2f uT\n', hBody);
fprintf('\nIn app_config.h:\n');
fprintf('#define MAG_AXIS_MAP   { %gf, %gf, %gf,   %gf, %gf, %gf,   %gf, %gf, %gf }\n', Mb.');
fprintf('#define MAG_HARD_IRON  { %.2ff, %.2ff, %.2ff }\n', hBody);

out = struct('M', Mb, 'hardIronBody', hBody, 'residuals', res, 'ratio', ratio);
end

function C = signedPermutations()
%SIGNEDPERMUTATIONS  The 24 rotation matrices with a single +-1 per row and column.
C = {};
pr = perms(1:3);
for a = 1:size(pr,1)
    for s = 0:7
        sg = 1 - 2*[bitand(s,1), bitand(s,2)>0, bitand(s,4)>0];
        M = zeros(3);
        for r = 1:3, M(r, pr(a,r)) = sg(r); end
        if det(M) > 0, C{end+1} = M; end %#ok<AGROW>
    end
end
end
