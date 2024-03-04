function varargout = load_expm(varargin)
    %LOAD_EXPM [INTERNAL] 
    %
    %  LOAD_EXPM(char name)
    %
    %Explicitly load a plugin dynamically.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/expm.hpp#L36
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/expm.cpp#L36-L38
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(881, varargin{:});
end
