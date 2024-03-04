function varargout = load_integrator(varargin)
    %LOAD_INTEGRATOR [INTERNAL] 
    %
    %  LOAD_INTEGRATOR(char name)
    %
    %Explicitly load a plugin dynamically.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L101
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L101-L103
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(806, varargin{:});
end
