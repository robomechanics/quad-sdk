function varargout = expm_n_in(varargin)
    %EXPM_N_IN [INTERNAL] 
    %
    %  int = EXPM_N_IN()
    %
    %Get the number of expm solver inputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_rs
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/expm.hpp#L49
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/expm.cpp#L49-L51
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(878, varargin{:});
end
