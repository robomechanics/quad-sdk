function [ret] = skew2angvel(w)
%angvel2skew maps the 3x3 skew-symmetric matrix w_hat to the 3-vector w
%   w: 3x3 skew-symmetric matrix, ret: 3x1 column vector

ret = [w(3, 2); w(1, 3); w(2, 1)];

end

