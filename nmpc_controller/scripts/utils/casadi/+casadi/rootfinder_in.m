function varargout = rootfinder_in(varargin)
    %ROOTFINDER_IN [INTERNAL] 
    %
    %  {char} = ROOTFINDER_IN()
    %  char = ROOTFINDER_IN(int ind)
    %
    %Get rootfinder input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1u0
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L47
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L47-L54
    %
    %
    %
    %.......
    %
    %::
    %
    %  ROOTFINDER_IN()
    %
    %
    %
    %[INTERNAL] 
    %Get rootfinder input scheme.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ty
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L35
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L35-L39
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  ROOTFINDER_IN(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get rootfinder input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1u0
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L47
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L47-L54
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(842, varargin{:});
end
