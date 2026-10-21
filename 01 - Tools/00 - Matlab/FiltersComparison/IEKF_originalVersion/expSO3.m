function R = expSO3(phi)
%EXPSO3  Exponential map from the Lie algebra so(3) to the Lie group SO(3).
%
%   R = expSO3(phi) maps a rotation vector phi (3x1, axis-angle: direction is
%   the rotation axis, norm is the rotation angle in radians) to the
%   corresponding rotation matrix, using Rodrigues' formula:
%
%       R = I + sin(t)*K + (1-cos(t))*K^2,   K = skewSym(phi/t),  t = norm(phi)
%
%   For small angles a first-order approximation is used to avoid the 0/0
%   singularity at t = 0.
%
%   See also LOGSO3, SKEWSYM.
 
phi = phi(:);
t   = norm(phi);
 
if t < 1e-9
    % First-order expansion, numerically safe near the identity.
    R = eye(3) + skewSym(phi);
else
    K = skewSym(phi/t);
    R = eye(3) + sin(t)*K + (1-cos(t))*(K*K);
end
end