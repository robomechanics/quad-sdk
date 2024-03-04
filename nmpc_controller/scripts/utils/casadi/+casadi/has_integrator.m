function varargout = has_integrator(varargin)
    %HAS_INTEGRATOR [INTERNAL] 
    %
    %  bool = HAS_INTEGRATOR(char name)
    %
    %Check if a particular plugin is available.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L97
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L97-L99
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(805, varargin{:});
end
