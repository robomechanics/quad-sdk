function varargout = rootfinder_options(varargin)
    %ROOTFINDER_OPTIONS [INTERNAL] 
    %
    %  {char} = ROOTFINDER_OPTIONS(char name)
    %
    %Get all options for a plugin.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1u4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L72
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L72-L74
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(846, varargin{:});
end
