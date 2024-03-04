function varargout = collocation_interpolators(varargin)
    %COLLOCATION_INTERPOLATORS [INTERNAL] 
    %
    %  [{[double]} OUTPUT, [double] OUTPUT] = COLLOCATION_INTERPOLATORS([double] tau)
    %
    %Obtain collocation interpolating matrices.
    %
    %A collocation method poses a polynomial Pi that interpolates exactly 
    %
    %through an initial state (0,X_0) and helper states at collocation 
    %points 
    %(tau_j,X:collPoint(j)).
    %
    %This function computes the linear mapping between dPi/dt and 
    %coefficients 
    %Z=[X_0 X:collPoints].
    %
    %Parameters:
    %-----------
    %
    %tau: 
    %location of collocation points, as obtained from collocation_points
    %
    %C: 
    %interpolating coefficients to obtain derivatives. Length: order+1, 
    %
    %order+1
    %
    %
    %
    %::
    %
    %dPi/dt @Z_j = (1/h) Sum_i C[j][i]*Z_i,
    %
    %
    %
    %with h the length of the integration interval.
    %
    %Parameters:
    %-----------
    %
    %D: 
    %interpolating coefficients to obtain end state. Length: order+1
    %
    %
    %
    %::
    %
    %Pi @X_f = Sum_i D[i]*Z_i
    %
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1sp
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L189
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L189-L231
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(973, varargin{:});
end
