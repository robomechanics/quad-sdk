function varargout = conic_out(varargin)
    %CONIC_OUT [INTERNAL] 
    %
    %  {char} = CONIC_OUT()
    %  char = CONIC_OUT(int ind)
    %
    %Get output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1eh
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L91
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L91-L100
    %
    %
    %
    %.......
    %
    %::
    %
    %  CONIC_OUT(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1eh
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L91
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L91-L100
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
    %  CONIC_OUT()
    %
    %
    %
    %[INTERNAL] 
    %Get QP solver output scheme of QP solvers.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ef
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L66
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L66-L70
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(819, varargin{:});
end
