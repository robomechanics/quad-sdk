function varargout = load_dple(varargin)
    %LOAD_DPLE [INTERNAL] 
    %
    %  LOAD_DPLE(char name)
    %
    %Explicitly load a plugin dynamically.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.hpp#L35
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.cpp#L35-L37
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(875, varargin{:});
end
