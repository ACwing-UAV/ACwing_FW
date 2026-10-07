function R = so3ExpLocal(phi)
%SO3EXPLOCAL  Rodrigues with the same Taylor series as so3_exp() in so3.c.
phi = phi(:);
t2 = phi.' * phi;
if t2 < 2.5e-3
    a = 1 - t2/6  + t2^2/120;
    b = 0.5 - t2/24 + t2^2/720;
else
    t = sqrt(t2);
    a = sin(t)/t;
    b = (1 - cos(t))/t2;
end
R = eye(3) + a*skewLocal(phi) + b*(phi*phi.' - t2*eye(3));
end
