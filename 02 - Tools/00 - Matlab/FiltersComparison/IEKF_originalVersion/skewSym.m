function S = skewSym(v)
%SKEWSYM  Skew-symmetric matrix [v]_x associated with a 3-vector.
%
%   S = skewSym(v) returns the 3x3 matrix such that skewSym(v)*w = cross(v,w).
%   This is the isomorphism between R^3 and the Lie algebra so(3).
%
%   See also EXPSO3, LOGSO3.
 
v = v(:);
S = [    0   -v(3)   v(2) ;
       v(3)     0   -v(1) ;
      -v(2)   v(1)     0  ];
end