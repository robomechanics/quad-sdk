function varargout = dyn_n_out(varargin)
    %DYN_N_OUT [INTERNAL] 
    %
    %  int = DYN_N_OUT()
    %
    %Get the number of simulator outputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_25u
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L227
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L227-L229
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(815, varargin{:});
end
