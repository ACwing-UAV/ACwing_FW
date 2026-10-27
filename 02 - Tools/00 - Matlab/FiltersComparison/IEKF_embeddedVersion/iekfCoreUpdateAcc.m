function [f, gated] = iekfCoreUpdateAcc(f, acc)
%IEKFCOREUPDATEACC  Accelerometer update. Mirror of iekf_update_acc().
gated = false;
na = norm(acc);
if na < 1e-9, return; end
y = acc(:) / na;
f.lastAccDir = y;
yhat = f.R.' * f.gRef;

[r, f.lastAccDev] = adaptiveVar(f.cfg.sigmaAcc^2, na, f.nomAcc, f.cfg.accTol, f.cfg.adaptive);
f.lastAccInnovB = y - yhat;
trusted = f.accTrusted || f.accGrace > 0;
if f.accGrace > 0, f.accGrace = f.accGrace - 1; end
if trusted, f.accInfl = 1; end
gate = f.cfg.accNisGate; if trusted, gate = 0; end
proj = 0; if f.cfg.decouple, proj = 1; end            % 1 = tilt only
[f, f.lastAccNis, f.accInfl] = iekfVectorUpdate(f, y, yhat, r, gate, f.accInfl, proj);
f.accTrusted = false;

% persistence of the innovation in the world frame (updated estimate, as in C)
ew = f.R * f.lastAccInnovB;
a = f.cfg.persistDecay;
f.innovWMean = a*f.innovWMean + (1 - a)*ew;
f.innovWPow  = a*f.innovWPow  + (1 - a)*(ew.'*ew);

% lockout protection (mirror of iekf_update_acc)
if f.cfg.lockoutUpdates > 0
    m2 = f.innovWMean.' * f.innovWMean;
    persistent_ = (f.innovWPow > 0) && (m2 > f.cfg.persistRatio * f.innovWPow) && ...
                  (m2 > f.cfg.persistMin^2);
    if f.accInfl > 2
        f.accGatedRun = f.accGatedRun + 1;
        if f.accGatedRun >= f.cfg.lockoutUpdates && persistent_
            f.P(1:3,1:3) = f.P(1:3,1:3) + f.cfg.lockoutPBump * eye(3);
            f.accInfl = 1;
            f.accGatedRun = 0;
            f.accGrace = f.cfg.graceUpdates;
            f.lockouts = f.lockouts + 1;
        end
    else
        f.accGatedRun = 0;
    end
end
gated = (f.lastAccDev > f.cfg.accTol) || (f.accInfl > 2);
end

function [r, dev] = adaptiveVar(r0, n, nom, tol, adaptive)
dev = abs(n - nom) / nom;
r = r0;
if adaptive, r = r0 * (1 + (dev/tol)^2); end
end
