function varargout = nlpsol_option_info(varargin)
    %NLPSOL_OPTION_INFO [INTERNAL] 
    %
    %  char = NLPSOL_OPTION_INFO(char name, char op)
    %
    %Get documentation for a particular option.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1t7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.hpp#L908
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/nlpsol.cpp#L908-L910
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(837, varargin{:});
end
