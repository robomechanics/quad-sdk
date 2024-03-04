function varargout = conic_n_out(varargin)
    %CONIC_N_OUT [INTERNAL] 
    %
    %  int = CONIC_N_OUT()
    %
    %Get the number of QP solver outputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ej
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L106
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L106-L108
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(821, varargin{:});
end
