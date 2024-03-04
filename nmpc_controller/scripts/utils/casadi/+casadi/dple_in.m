function varargout = dple_in(varargin)
    %DPLE_IN [INTERNAL] 
    %
    %  {char} = DPLE_IN()
    %  char = DPLE_IN(int ind)
    %
    %Get DPLE input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ne
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.hpp#L114
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.cpp#L114-L121
    %
    %
    %
    %.......
    %
    %::
    %
    %  DPLE_IN()
    %
    %
    %
    %[INTERNAL] 
    %Get input scheme of DPLE solvers.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1nc
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.hpp#L102
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.cpp#L102-L106
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
    %  DPLE_IN(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get DPLE input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ne
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.hpp#L114
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.cpp#L114-L121
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(870, varargin{:});
end
