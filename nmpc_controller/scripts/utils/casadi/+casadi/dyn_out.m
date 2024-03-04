function varargout = dyn_out(varargin)
    %DYN_OUT [INTERNAL] 
    %
    %  {char} = DYN_OUT()
    %  char = DYN_OUT(int ind)
    %
    %Get output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_25s
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L219
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L219-L221
    %
    %
    %
    %.......
    %
    %::
    %
    %  DYN_OUT(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_25s
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L219
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L219-L221
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
    %  DYN_OUT()
    %
    %
    %
    %[INTERNAL] 
    %Get simulator output scheme of simulators.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_25q
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.hpp#L211
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/integrator.cpp#L211-L213
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(813, varargin{:});
end
