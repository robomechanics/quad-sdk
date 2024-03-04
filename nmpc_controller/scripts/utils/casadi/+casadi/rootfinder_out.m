function varargout = rootfinder_out(varargin)
    %ROOTFINDER_OUT [INTERNAL] 
    %
    %  {char} = ROOTFINDER_OUT()
    %  char = ROOTFINDER_OUT(int ind)
    %
    %Get rootfinder output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1u1
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L56
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L56-L62
    %
    %
    %
    %.......
    %
    %::
    %
    %  ROOTFINDER_OUT()
    %
    %
    %
    %[INTERNAL] 
    %Get rootfinder output scheme.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1tz
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L41
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L41-L45
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
    %  ROOTFINDER_OUT(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get rootfinder output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1u1
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L56
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L56-L62
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(843, varargin{:});
end
