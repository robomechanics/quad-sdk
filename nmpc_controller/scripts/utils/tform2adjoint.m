function [ret] = tform2adjoint(g)
%tform2adjoint maps the rigid body transformation g, in homogeneous
%coordinates, to the transformation adjoint matrix, Ad_g
%   g: rigid body transformation in homogeneous coordinates, ret:
%   transformation adjoint matrix

R=g(1:3, 1:3);
p=g(1:3, 4);
p_hat=angvel2skew(p);

ret = [R, p_hat*R;
    zeros(3, 3), R];

end

