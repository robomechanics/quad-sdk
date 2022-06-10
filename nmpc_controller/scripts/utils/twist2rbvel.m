function [ret] = twist2rbvel(x)
%twist2rbvel maps the 6-vector twist xi to the 4x4 rigid body velocity matrix in
%homogeneous coordinates xi_hat
%   x: 6x1 column vector twist [v; w], ret: 4x4 rigid body velocity matrix
%   in homogeneous coordinates

v = x(1:3);
w = x(4:6);
w_hat = angvel2skew(w);
ret = [w_hat, v;
    0, 0, 0, 0];

end

