function varargout = rootfinder_n_in(varargin)
    %ROOTFINDER_N_IN [INTERNAL] 
    %
    %  int = ROOTFINDER_N_IN()
    %
    %Number of rootfinder inputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1u2
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L64
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L64-L66
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(844, varargin{:});
end
