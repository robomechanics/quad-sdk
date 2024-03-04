function varargout = dple_out(varargin)
    %DPLE_OUT [INTERNAL] 
    %
    %  {char} = DPLE_OUT()
    %  char = DPLE_OUT(int ind)
    %
    %Get DPLE output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1nf
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.hpp#L123
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.cpp#L123-L129
    %
    %
    %
    %.......
    %
    %::
    %
    %  DPLE_OUT(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get DPLE output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1nf
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.hpp#L123
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.cpp#L123-L129
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
    %  DPLE_OUT()
    %
    %
    %
    %[INTERNAL] 
    %Get output scheme of DPLE solvers.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1nd
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.hpp#L108
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/dple.cpp#L108-L112
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(871, varargin{:});
end
