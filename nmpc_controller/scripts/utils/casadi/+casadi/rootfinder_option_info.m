function varargout = rootfinder_option_info(varargin)
    %ROOTFINDER_OPTION_INFO [INTERNAL] 
    %
    %  char = ROOTFINDER_OPTION_INFO(char name, char op)
    %
    %Get documentation for a particular option.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1u6
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.hpp#L80
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/rootfinder.cpp#L80-L82
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(848, varargin{:});
end
