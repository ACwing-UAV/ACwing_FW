function [f, nis, infl] = iekfVectorUpdate(f, y, yhat, r, gate, infl, proj)
%IEKFVECTORUPDATE  Common vector correction. Mirror of vector_update() in iekf.c:
%   NIS with nominal R -> inflation with memory -> gain -> projection ->
%   retraction onto SO(3) -> Joseph form -> error reset -> conditioning.
%   proj: 0 none, 1 tilt only (acc), 2 yaw only (mag).
H = [skewLocal(yhat), zeros(3)];
PHt = f.P * H.';
S0 = H * PHt;
innov = y(:) - yhat(:);

Si = inv(S0 + r*eye(3));
nis = innov.' * Si * innov;

k = 1;
if gate > 0
    inst = nis / gate;
    g = infl * f.cfg.inflDecay;
    if g < 1, g = 1; end
    if inst > g, g = inst; end
    if g > f.cfg.inflMax, g = f.cfg.inflMax; end
    infl = g;
    k = g;
end
re = r * k;
if k ~= 1, Si = inv(S0 + re*eye(3)); end
K = PHt * Si;
K = projectGain(f, K, proj);

dx = K * innov;
f.R = so3Orthonormalize(f.R * so3ExpLocal(dx(1:3)));
if f.cfg.estimateBias, f.b = f.b + dx(4:6); end

% Joseph form: valid for any K (also projected or with inflated R)
IKH = eye(6) - K*H;
f.P = IKH * f.P * IKH.' + re * (K * K.');
f.P = (f.P + f.P.') / 2;

% Error reset after the retraction: P_aa <- J P_aa J', P_ab <- J P_ab
if f.cfg.resetJacobian
    J = eye(3) - 0.5 * skewLocal(dx(1:3));
    f.P(1:3,1:3) = J * f.P(1:3,1:3) * J.';
    if f.cfg.estimateBias
        f.P(1:3,4:6) = J * f.P(1:3,4:6);
        f.P(4:6,1:3) = f.P(1:3,4:6).';
    end
end
f = iekfConditionCov(f);
end

function K = projectGain(f, K, proj)
if proj == 0, return; end
a = f.R.' * f.gRef;  a = a / norm(a);
blocks = {1:3};
if f.cfg.estimateBias, blocks{end+1} = 4:6; end
for bk = 1:numel(blocks)
    rows = blocks{bk};
    for col = 1:3
        d = a.' * K(rows, col);
        if proj == 1
            K(rows, col) = K(rows, col) - d * a;
        else
            K(rows, col) = d * a;
        end
    end
end
end
