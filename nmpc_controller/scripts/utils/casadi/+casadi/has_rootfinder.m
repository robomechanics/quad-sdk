function varargout = has_rootfinder(varargin)
    %HAS_ROOTFINDER [INTERNAL] 
    %
    %  bool = HAS_ROOTFINDER(char name)
    %
    %Check if a particular plugin is available.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L84
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L84-L86
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(849, varargin{:});
end
