function [ret] = rbvel2twist(x)
%twist2rbvel maps the 4x4 rigid body velocity matrix in homogeneous
%coordinates xi_hat to the 6-vector twist xi
%   ret: 4x4 rigid body velocity matrix in homogeneous coordinates, x: 6x1
%   column vector twist [v; w]

v = x(1:3, 4);
w_hat = x(1:3, 1:3);
w = skew2angvel(w_hat);
ret = [v; w];

end

