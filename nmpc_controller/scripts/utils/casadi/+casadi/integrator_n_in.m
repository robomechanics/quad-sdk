function varargout = integrator_n_in(varargin)
    %INTEGRATOR_N_IN [INTERNAL] 
    %
    %  int = INTEGRATOR_N_IN()
    %
    %Get the number of integrator inputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_7f
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L199
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L199-L201
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(810, varargin{:});
end
