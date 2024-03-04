function varargout = rootfinder_option_type(varargin)
    %ROOTFINDER_OPTION_TYPE [INTERNAL] 
    %
    %  char = ROOTFINDER_OPTION_TYPE(char name, char op)
    %
    %Get type info for a particular option.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1u5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L76
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L76-L78
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(847, varargin{:});
end
