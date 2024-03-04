function varargout = rootfinder_n_out(varargin)
    %ROOTFINDER_N_OUT [INTERNAL] 
    %
    %  int = ROOTFINDER_N_OUT()
    %
    %Number of rootfinder outputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1u3
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L68
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L68-L70
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(845, varargin{:});
end
