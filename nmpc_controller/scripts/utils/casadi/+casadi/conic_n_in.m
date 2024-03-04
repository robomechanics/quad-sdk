function varargout = conic_n_in(varargin)
    %CONIC_N_IN [INTERNAL] 
    %
    %  int = CONIC_N_IN()
    %
    %Get the number of QP solver inputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ei
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L102
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L102-L104
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(820, varargin{:});
end
