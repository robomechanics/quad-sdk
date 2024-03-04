function varargout = load_conic(varargin)
    %LOAD_CONIC [INTERNAL] 
    %
    %  LOAD_CONIC(char name)
    %
    %Explicitly load a plugin dynamically.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L35
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L35-L37
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(826, varargin{:});
end
