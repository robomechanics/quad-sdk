function varargout = dyn_n_in(varargin)
    %DYN_N_IN [INTERNAL] 
    %
    %  int = DYN_N_IN()
    %
    %Get the number of simulator inputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_25t
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L223
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L223-L225
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(814, varargin{:});
end
