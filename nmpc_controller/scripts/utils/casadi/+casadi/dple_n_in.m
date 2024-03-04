function varargout = dple_n_in(varargin)
    %DPLE_N_IN [INTERNAL] 
    %
    %  int = DPLE_N_IN()
    %
    %Get the number of QP solver inputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ng
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.hpp#L131
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.cpp#L131-L133
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(872, varargin{:});
end
