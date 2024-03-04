function varargout = has_expm(varargin)
    %HAS_EXPM [INTERNAL] 
    %
    %  bool = HAS_EXPM(char name)
    %
    %Check if a particular plugin is available.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/expm.hpp#L32
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/expm.cpp#L32-L34
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(880, varargin{:});
end
