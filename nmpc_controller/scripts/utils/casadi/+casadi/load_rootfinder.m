function varargout = load_rootfinder(varargin)
    %LOAD_ROOTFINDER [INTERNAL] 
    %
    %  LOAD_ROOTFINDER(char name)
    %
    %Explicitly load a plugin dynamically.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L88
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L88-L90
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(850, varargin{:});
end
