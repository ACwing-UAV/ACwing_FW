function R = so3Orthonormalize(R)
%SO3ORTHONORMALIZE  Gram-Schmidt on the columns, as so3_orthonormalize() in so3.c.
c0 = R(:,1); c1 = R(:,2);
n0 = norm(c0); if n0 < 1e-12, return; end
c0 = c0 / n0;
c1 = c1 - (c0.'*c1) * c0;
n1 = norm(c1); if n1 < 1e-12, return; end
c1 = c1 / n1;
R = [c0, c1, cross(c0, c1)];
end
