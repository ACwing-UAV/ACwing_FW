function f = iekfCorePropagate(f, dtheta, h)
%IEKFCOREPROPAGATE  Propagation with an angle increment. Mirror of iekf_propagate_dtheta().
%   dtheta = sum of omega*dt over the step samples (bias NOT removed), h = duration [s].
phi = dtheta(:) - f.b * h;
E = so3ExpLocal(phi);
f.R = so3Orthonormalize(f.R * E);
f.Elast = E;

F = eye(6);
F(1:3,1:3) = E.';                       % exp(-phi): depends only on the measured gyro
if f.cfg.estimateBias, F(1:3,4:6) = -h * eye(3); end
f.P = F * f.P * F.';
qg = f.cfg.sigmaGyro^2 * h;
qb = f.cfg.estimateBias * f.cfg.sigmaBias^2 * h;
f.P(1:3,1:3) = f.P(1:3,1:3) + qg * eye(3);
f.P(4:6,4:6) = f.P(4:6,4:6) + qb * eye(3);
f = iekfConditionCov(f);
end
