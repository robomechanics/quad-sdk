function varargout = conic_in(varargin)
    %CONIC_IN [INTERNAL] 
    %
    %  {char} = CONIC_IN()
    %  char = CONIC_IN(int ind)
    %
    %Get QP solver input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1eg
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L72
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L72-L89
    %
    %
    %
    %.......
    %
    %::
    %
    %  CONIC_IN()
    %
    %
    %
    %[INTERNAL] 
    %Get input scheme of QP solvers.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ee
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L60
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L60-L64
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
    %  CONIC_IN(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get QP solver input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1eg
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L72
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L72-L89
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(818, varargin{:});
end
