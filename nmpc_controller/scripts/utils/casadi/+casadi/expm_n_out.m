function varargout = expm_n_out(varargin)
    %EXPM_N_OUT [INTERNAL] 
    %
    %  int = EXPM_N_OUT()
    %
    %Get the number of expm solver outputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_rt
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/expm.hpp#L53
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/expm.cpp#L53-L55
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(879, varargin{:});
end
