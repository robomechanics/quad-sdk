function varargout = load_linsol(varargin)
    %LOAD_LINSOL [INTERNAL] 
    %
    %  LOAD_LINSOL(char name)
    %
    %Explicitly load a plugin dynamically.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L209
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L209-L211
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(867, varargin{:});
end
