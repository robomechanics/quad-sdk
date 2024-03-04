function varargout = integrator_in(varargin)
    %INTEGRATOR_IN [INTERNAL] 
    %
    %  {char} = INTEGRATOR_IN()
    %  char = INTEGRATOR_IN(int ind)
    %
    %Get integrator input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_7d
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L171
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L171-L183
    %
    %
    %
    %.......
    %
    %::
    %
    %  INTEGRATOR_IN()
    %
    %
    %
    %[INTERNAL] 
    %Get input scheme of integrators.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_7b
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L159
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L159-L163
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
    %  INTEGRATOR_IN(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get integrator input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_7d
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L171
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L171-L183
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(808, varargin{:});
end
