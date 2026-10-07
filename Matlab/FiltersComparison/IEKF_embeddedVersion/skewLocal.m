function S = skewLocal(v)
%SKEWLOCAL  Skew-symmetric matrix [v]x (as skewSym.m / so3_skew()).
S = [ 0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0 ];
end
