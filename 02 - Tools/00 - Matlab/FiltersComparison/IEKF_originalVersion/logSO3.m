function phi = logSO3(R)
%LOGSO3  Logarithm map from the Lie group SO(3) to the Lie algebra so(3).
%
%   phi = logSO3(R) returns the rotation vector (3x1) such that expSO3(phi) = R.
%   norm(phi) is the geodesic distance on SO(3) between R and the identity,
%   i.e. the total rotation angle in radians. This is the natural metric used
%   throughout for attitude error evaluation.
%
%   See also EXPSO3, SKEWSYM.
 
% Clamp for numerical safety: rounding can push the trace slightly out of range.
c = (trace(R) - 1) / 2;
c = max(min(c, 1), -1);
t = acos(c);
 
if t < 1e-9
    phi = zeros(3,1);
elseif abs(t - pi) < 1e-6
    % Near-pi case: sin(t) -> 0, the generic formula loses precision.
    % Recover the axis from the diagonal of (R + I)/2.
    A = (R + eye(3)) / 2;
    ax = sqrt(max(diag(A), 0));
    [~, imax] = max(ax);
    if ax(imax) > 0
        ax = ax * sign(signedAxisRef(R, imax));
    end
    phi = t * ax(:) / norm(ax);
else
    phi = (t / (2*sin(t))) * [ R(3,2) - R(2,3) ;
                               R(1,3) - R(3,1) ;
                               R(2,1) - R(1,2) ];
end
end
 
function s = signedAxisRef(R, i)
%SIGNEDAXISREF  Helper to disambiguate the axis sign in the near-pi case.
v = [ R(3,2) - R(2,3) ; R(1,3) - R(3,1) ; R(2,1) - R(1,2) ];
if abs(v(i)) > 1e-12
    s = v(i);
else
    s = 1;
end
end