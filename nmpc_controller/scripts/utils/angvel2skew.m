function [ret] = angvel2skew(w)
%angvel2skew maps the 3-vector w to the 3x3 skew-symmetric matrix w_hat
%   w: 3x1 column vector, ret: 3x3 skew-symmetric matrix

ret = [0, -w(3), w(2);
    w(3), 0, -w(1);
    -w(2), w(1), 0];

end

