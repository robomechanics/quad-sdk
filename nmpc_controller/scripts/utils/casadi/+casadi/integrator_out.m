function varargout = integrator_out(varargin)
    %INTEGRATOR_OUT [INTERNAL] 
    %
    %  {char} = INTEGRATOR_OUT()
    %  char = INTEGRATOR_OUT(int ind)
    %
    %Get output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_7e
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L185
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L185-L197
    %
    %
    %
    %.......
    %
    %::
    %
    %  INTEGRATOR_OUT(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_7e
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L185
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L185-L197
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  INTEGRATOR_OUT()
    %
    %
    %
    %[INTERNAL] 
    %Get integrator output scheme of integrators.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_7c
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L165
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L165-L169
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(809, varargin{:});
end
