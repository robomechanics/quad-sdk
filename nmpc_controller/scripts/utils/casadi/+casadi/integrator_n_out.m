function varargout = integrator_n_out(varargin)
    %INTEGRATOR_N_OUT [INTERNAL] 
    %
    %  int = INTEGRATOR_N_OUT()
    %
    %Get the number of integrator outputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_7g
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L203
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L203-L205
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(811, varargin{:});
end
