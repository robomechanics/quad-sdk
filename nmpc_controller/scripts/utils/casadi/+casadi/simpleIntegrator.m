function varargout = simpleIntegrator(varargin)
    %SIMPLEINTEGRATOR [INTERNAL] 
    %
    %  Function = SIMPLEINTEGRATOR(Function f, char integrator, struct integrator_options)
    %
    %Simplified wrapper for the  Integrator class.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1st
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.hpp#L356
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integration_tools.cpp#L356-L395
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(977, varargin{:});
end
