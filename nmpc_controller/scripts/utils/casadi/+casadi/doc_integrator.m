function varargout = doc_integrator(varargin)
    %DOC_INTEGRATOR [INTERNAL] 
    %
    %  char = DOC_INTEGRATOR(char name)
    %
    %Get the documentation string for a plugin.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L105
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L105-L107
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(807, varargin{:});
end
