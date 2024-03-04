function varargout = load_interpolant(varargin)
    %LOAD_INTERPOLANT [INTERNAL] 
    %
    %  LOAD_INTERPOLANT(char name)
    %
    %Explicitly load a plugin dynamically.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.hpp#L38
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/interpolant.cpp#L38-L40
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(885, varargin{:});
end
