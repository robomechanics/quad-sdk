function varargout = dple_n_out(varargin)
    %DPLE_N_OUT [INTERNAL] 
    %
    %  int = DPLE_N_OUT()
    %
    %Get the number of QP solver outputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1nh
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.hpp#L135
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.cpp#L135-L137
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(873, varargin{:});
end
