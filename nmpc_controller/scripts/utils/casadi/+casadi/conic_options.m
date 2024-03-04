function varargout = conic_options(varargin)
    %CONIC_OPTIONS [INTERNAL] 
    %
    %  {char} = CONIC_OPTIONS(char name)
    %
    %Get all options for a plugin.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ek
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.hpp#L542
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/conic.cpp#L542-L544
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(822, varargin{:});
end
