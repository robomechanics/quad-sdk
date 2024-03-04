function varargout = dyn_in(varargin)
    %DYN_IN [INTERNAL] 
    %
    %  {char} = DYN_IN()
    %  char = DYN_IN(int ind)
    %
    %Get simulator input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_25r
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L215
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L215-L217
    %
    %
    %
    %.......
    %
    %::
    %
    %  DYN_IN()
    %
    %
    %
    %[INTERNAL] 
    %Get input scheme of simulators.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_25p
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L207
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L207-L209
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
    %  DYN_IN(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get simulator input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_25r
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L215
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L215-L217
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(812, varargin{:});
end
