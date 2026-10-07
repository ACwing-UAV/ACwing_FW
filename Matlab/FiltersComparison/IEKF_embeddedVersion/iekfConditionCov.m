function f = iekfConditionCov(f)
%IEKFCONDITIONCOV  Symmetry and diagonal floor. Mirror of condition_covariance().
f.P = (f.P + f.P.') / 2;
for i = 1:3
    if f.P(i,i) < 1e-12, f.P(i,i) = 1e-12; end
end
if f.cfg.estimateBias
    for i = 4:6
        if f.P(i,i) < 1e-16, f.P(i,i) = 1e-16; end
    end
end
end
